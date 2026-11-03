import { expect, test, type Page } from "@playwright/test";
import {
  drawBetween,
  fit,
  frameOf,
  nodeAt,
  openBoard,
  shot,
  touchDrag,
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
        const [r, g, b] = ctx.getImageData(px, py, 1, 1).data;
        if (r === undefined || g === undefined || b === undefined) continue;
        if (r > 200 && g > 150 && g < 220 && b < 100) hits++;
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
  await expect.poll(() => strokeDrawnBetween(page, start, near)).toBe(true);
  expect(await ringPixels(page)).toBe(0);
  await shot(page, "loupe-mouse-none");
  await page.mouse.up();
});

// Samples the midpoint of the stroke for the in-progress stroke blue.
function strokeDrawnBetween(page: Page, a: Client, b: Client): Promise<boolean> {
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
    [(a.x + b.x) / 2, (a.y + b.y) / 2] as const,
  );
}
