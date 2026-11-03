import { expect, test, type Page } from "@playwright/test";
import {
  drawBetween,
  expectNodeDrawnAt,
  fit,
  frameOf,
  nodeAt,
  openBoard,
  pinch,
  pinchDrag,
  shot,
  touchDrag,
  zoomAt,
  type Client,
  type Frame,
} from "./helpers";

const LIFT = 90;
const RADIUS = 60;

// Mirrors the lens placement in web/loupe.ts: above the finger unless the
// top edge is too close, clamped so the disc stays on the canvas.
function lensCentre(frame: Frame, finger: Client): Client {
  const local = { x: finger.x - frame.left, y: finger.y - frame.top };
  const above = local.y - LIFT;
  const y = above - RADIUS >= 0 ? above : local.y + LIFT;
  const clamp = (v: number, low: number, high: number) => Math.max(low, Math.min(high, v));
  return {
    x: frame.left + clamp(local.x, RADIUS, frame.width - RADIUS),
    y: frame.top + clamp(y, RADIUS, frame.height - RADIUS),
  };
}

// Counts ring-amber samples on the lens circumference around a client point.
// The ring is 2 px wide, so each angle checks the 3 by 3 block around the
// sample; a centre on a pixel fraction otherwise blends half the samples.
function ringSamplesAround(page: Page, centre: Client): Promise<number> {
  return page.evaluate(
    ([x, y, radius]) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d");
      if (!ctx) return 0;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      let hits = 0;
      for (let k = 0; k < 8; k++) {
        const angle = (Math.PI * 2 * k) / 8;
        const px = Math.round((x - rect.left + radius * Math.cos(angle)) * dpr);
        const py = Math.round((y - rect.top + radius * Math.sin(angle)) * dpr);
        const { data } = ctx.getImageData(px - 1, py - 1, 3, 3);
        for (let i = 0; i < data.length; i += 4) {
          const [r, g, b] = [data[i]!, data[i + 1]!, data[i + 2]!];
          if (r > 200 && g > 150 && g < 220 && b < 100) {
            hits++;
            break;
          }
        }
      }
      return hits;
    },
    [centre.x, centre.y, RADIUS] as const,
  );
}

// Counts ring-amber pixels anywhere on the canvas.
function ringPixels(page: Page): Promise<number> {
  return page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const ctx = canvas.getContext("2d");
    if (!ctx) return 0;
    const { data } = ctx.getImageData(0, 0, canvas.width, canvas.height);
    let hits = 0;
    for (let i = 0; i < data.length; i += 4) {
      const r = data[i] ?? 0;
      const g = data[i + 1] ?? 0;
      const b = data[i + 2] ?? 0;
      if (r > 200 && g > 150 && g < 220 && b < 100) hits++;
    }
    return hits;
  });
}

// True when every canvas pixel within the radius of a client point is board
// background, so nothing drawn there can trigger the lens.
function isClear(page: Page, at: Client, radius: number): Promise<boolean> {
  return page.evaluate(
    ([x, y, radius]) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d");
      if (!ctx) return false;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      const cx = (x - rect.left) * dpr;
      const cy = (y - rect.top) * dpr;
      const span = radius * dpr;
      const { data, width } = ctx.getImageData(
        Math.round(cx - span),
        Math.round(cy - span),
        Math.round(span * 2),
        Math.round(span * 2),
      );
      for (let i = 0; i < data.length; i += 4) {
        const px = ((i / 4) % width) - span;
        const py = Math.floor(i / 4 / width) - span;
        if (px * px + py * py > span * span) continue;
        if (Math.abs((data[i] ?? 0) - 0x1a) > 8) return false;
      }
      return true;
    },
    [at.x, at.y, radius] as const,
  );
}

// The AI's reply lands somewhere unpredictable, so the test looks for a spot
// with nothing drawn within 45 px rather than assuming one.
async function clearSpot(page: Page, frame: Frame): Promise<Client> {
  const inset = 70;
  const candidates: Client[] = [
    { x: frame.left + inset, y: frame.top + inset },
    { x: frame.left + frame.width - inset, y: frame.top + inset },
    { x: frame.left + inset, y: frame.top + frame.height - inset },
    { x: frame.left + frame.width - inset, y: frame.top + frame.height - inset },
  ];
  for (const candidate of candidates) {
    if (await isClear(page, candidate, 45)) return candidate;
  }
  throw new Error("no clear spot on the board");
}

// A point 10 px inside the first chord, three tenths of the way along it.
function besideFirstChord(frame: Frame): Client {
  const t = fit(frame);
  const a = nodeAt(frame, t, 0);
  const b = nodeAt(frame, t, 1);
  const along = { x: a.x + (b.x - a.x) * 0.3, y: a.y + (b.y - a.y) * 0.3 };
  const length = Math.hypot(b.x - a.x, b.y - a.y);
  const inward = { x: -(b.y - a.y) / length, y: (b.x - a.x) / length };
  return { x: along.x + inward.x * 10, y: along.y + inward.y * 10 };
}

async function oneExchange(page: Page): Promise<void> {
  const mid = await drawBetween(page, 0, 1);
  await page.waitForTimeout(350);
  await page.mouse.click(mid.x, mid.y);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
}

test.beforeEach(async ({ page }) => {
  await openBoard(page);
  await oneExchange(page);
});

test("a touch stroke near an existing line shows the loupe and clears away from it", async ({
  page,
}) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const start = nodeAt(frame, fit(frame), 3);
  const near = besideFirstChord(frame);
  const clear = await clearSpot(page, frame);
  const cdp = await page.context().newCDPSession(page);

  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...start, id: 0 }],
  });
  await touchDrag(cdp, start, near);
  await expect
    .poll(() => ringSamplesAround(page, lensCentre(frame, near)))
    .toBeGreaterThanOrEqual(6);
  await shot(page, "loupe-near-line");

  await touchDrag(cdp, near, clear);
  await expect.poll(() => ringPixels(page)).toBe(0);
  await shot(page, "loupe-clear");

  await touchDrag(cdp, clear, near);
  await expect
    .poll(() => ringSamplesAround(page, lensCentre(frame, near)))
    .toBeGreaterThanOrEqual(6);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
  await expect.poll(() => ringPixels(page)).toBe(0);
});

test("a mouse stroke along the same path shows no loupe", async ({ page }) => {
  const frame = await frameOf(page);
  const start = nodeAt(frame, fit(frame), 3);
  const near = besideFirstChord(frame);
  await page.mouse.move(start.x, start.y);
  await page.mouse.down();
  await page.mouse.move(near.x, near.y, { steps: 20 });
  const mid = { x: (start.x + near.x) / 2, y: (start.y + near.y) / 2 };
  await expect.poll(() => strokeDrawnAt(page, mid)).toBe(true);
  expect(await ringPixels(page)).toBe(0);
  await shot(page, "loupe-mouse-none");
  await page.mouse.up();
});

// Samples a client point for the in-progress stroke blue.
function strokeDrawnAt(page: Page, at: Client): Promise<boolean> {
  return page.evaluate(
    ([x, y]) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d");
      if (!ctx) return false;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      for (let dx = -2; dx <= 2; dx++) {
        for (let dy = -2; dy <= 2; dy++) {
          const px = Math.round((x - rect.left + dx) * dpr);
          const py = Math.round((y - rect.top + dy) * dpr);
          const [r, , b] = ctx.getImageData(px, py, 1, 1).data;
          if (r !== undefined && b !== undefined && r < 100 && b > 200) return true;
        }
      }
      return false;
    },
    [at.x, at.y] as const,
  );
}

test("a stroke from a node with a line opens no lens until the finger is clear of it", async ({
  page,
}) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const start = nodeAt(frame, fit(frame), 0);
  const out = (px: number): Client => ({ x: start.x + px, y: start.y });
  const near = besideFirstChord(frame);
  const cdp = await page.context().newCDPSession(page);

  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...start, id: 0 }],
  });
  await touchDrag(cdp, start, out(20));
  await expect.poll(() => strokeDrawnAt(page, out(14))).toBe(true);
  expect(await ringPixels(page)).toBe(0);

  await touchDrag(cdp, out(20), out(60));
  await expect.poll(() => strokeDrawnAt(page, out(45))).toBe(true);
  expect(await ringPixels(page)).toBe(0);

  await touchDrag(cdp, out(60), near);
  await expect
    .poll(() => ringSamplesAround(page, lensCentre(frame, near)))
    .toBeGreaterThanOrEqual(6);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
});

test("the lens flips below the finger near the top edge", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const centre = { x: frame.left + frame.width / 2, y: frame.top + frame.height / 2 };
  await page.touchscreen.tap(centre.x, centre.y);
  await page.touchscreen.tap(centre.x, centre.y);
  let t = zoomAt(frame, fit(frame), centre, 2.5);
  await expectNodeDrawnAt(page, nodeAt(frame, t, 3));

  const lift = nodeAt(frame, t, 3).y - (frame.top + 70);
  const fingers: [Client, Client] = [
    { x: centre.x - 40, y: centre.y + 100 },
    { x: centre.x + 40, y: centre.y + 100 },
  ];
  const moved: [Client, Client] = [
    { x: fingers[0].x, y: fingers[0].y - lift },
    { x: fingers[1].x, y: fingers[1].y - lift },
  ];
  const cdp = await page.context().newCDPSession(page);
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [
      { ...fingers[0], id: 1 },
      { ...fingers[1], id: 2 },
    ],
  });
  await pinchDrag(cdp, fingers, moved);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
  t = pinch(frame, t, fingers, moved);
  const top = nodeAt(frame, t, 3);
  await expectNodeDrawnAt(page, top);

  const start = nodeAt(frame, t, 2);
  const near = { x: top.x, y: top.y - 12 };
  const lens = lensCentre(frame, near);
  expect(lens.y).toBeGreaterThan(near.y);
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...start, id: 0 }],
  });
  await touchDrag(cdp, start, near);
  await expect.poll(() => ringSamplesAround(page, lens)).toBeGreaterThanOrEqual(6);
  await shot(page, "loupe-flipped");
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
});
