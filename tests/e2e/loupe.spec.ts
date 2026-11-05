import { expect, test, type Page } from "@playwright/test";
import { LOUPE_RING } from "../../web/loupe";
import {
  alongLine,
  boardNodes,
  colourCentroid,
  expectNodeDrawnAt,
  frameOf,
  nodeOnScreen,
  openBoard,
  playExchange,
  ringPixels,
  shot,
  touchDrag,
  type Client,
  type Frame,
} from "./helpers";

const LIFT = 90;
const RADIUS = 60;

// Mirrors the lens placement in web/loupe.ts: LOUPE_LIFT above the finger,
// swinging to the given side as the finger nears the top, clamped so the disc
// stays on the canvas.
function lensCentre(frame: Frame, finger: Client, side: 1 | -1 = 1): Client {
  const local = { x: finger.x - frame.left, y: finger.y - frame.top };
  const y = Math.max(local.y - LIFT, RADIUS);
  const dy = local.y - y;
  const dx = Math.sqrt(Math.max(0, LIFT * LIFT - dy * dy));
  const clamp = (v: number, low: number, high: number) => Math.max(low, Math.min(high, v));
  return {
    x: frame.left + clamp(local.x + side * dx, RADIUS, frame.width - RADIUS),
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

// A point 10 px beside the first line, three tenths of the way along it,
// on the board as the engine settled it.
async function besideFirstChord(page: Page): Promise<Client> {
  const { at, tangent } = await alongLine(page, 0, 0.3);
  return { x: at.x - tangent.y * 10, y: at.y + tangent.x * 10 };
}

test.beforeEach(async ({ page }) => {
  await openBoard(page);
  await playExchange(page);
});

test("a touch stroke near an existing line shows the loupe and clears away from it", async ({
  page,
}) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const start = await nodeOnScreen(page, 3, 4);
  const near = await besideFirstChord(page);
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
  const start = await nodeOnScreen(page, 3, 4);
  const near = await besideFirstChord(page);
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
  const start = await nodeOnScreen(page, 0, 4);
  const out = (px: number): Client => ({ x: start.x + px, y: start.y });
  const near = await besideFirstChord(page);
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

test("the lens slides beside the finger across the top edge without a jump", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  // The highest node on the settled board sits within the lens's lift and
  // radius of the top edge at the fitted view, so a stroke that climbs past
  // it crosses the zone where the lens leaves its spot above the finger.
  const nodes = await boardNodes(page);
  const highest = [...nodes.entries()].reduce((best, entry) =>
    entry[1].y < best[1].y ? entry : best,
  );
  const top = await nodeOnScreen(page, highest[0], nodes.size);
  await expectNodeDrawnAt(page, top);
  const side: 1 | -1 = top.x < frame.left + frame.width / 2 ? 1 : -1;
  const cdp = await page.context().newCDPSession(page);

  const start = await nodeOnScreen(page, highest[0] === 0 ? 3 : 0, nodes.size);
  const below = { x: top.x, y: Math.min(top.y + 40, frame.top + frame.height - 20) };
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...start, id: 0 }],
  });
  await touchDrag(cdp, start, below);

  // The finger climbs 8 px at a time to 12 px above the node. At every step
  // the ring sits where the placement says, the lens keeps its lift from the
  // finger, and the measured centre moves no further than the finger did
  // plus the swing, never a flip.
  let finger = below;
  let last: Client | null = null;
  const steps: string[] = [];
  while (finger.y > top.y - 12) {
    const next = { x: finger.x, y: Math.max(finger.y - 8, top.y - 12) };
    await touchDrag(cdp, finger, next, 0, 2);
    finger = next;
    const want = lensCentre(frame, finger, side);
    await expect.poll(() => ringSamplesAround(page, want)).toBeGreaterThanOrEqual(6);
    const { at } = await colourCentroid(page, LOUPE_RING);
    if (last) expect(Math.hypot(at.x - last.x, at.y - last.y)).toBeLessThan(8 + 2 * 8);
    const lift = Math.hypot(at.x - finger.x, at.y - finger.y);
    expect(lift).toBeGreaterThan(LIFT - 4);
    expect(lift).toBeLessThan(LIFT + 4);
    steps.push(
      `${Math.round(finger.y - frame.top)}:${Math.round(at.x - frame.left)},${Math.round(at.y - frame.top)}`,
    );
    last = at;
  }
  test.info().annotations.push({ type: "lens-centres", description: steps.join(" ") });
  expect(lensCentre(frame, finger, side).x).not.toBe(finger.x);
  await shot(page, "loupe-top-slide");
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
});
