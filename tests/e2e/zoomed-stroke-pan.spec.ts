import { expect, test, type CDPSession, type Page } from "@playwright/test";
import {
  boardSettled,
  clampToViewport,
  colourPixelsNear,
  ctrlWheel,
  expectNodeDrawnAt,
  fit,
  frameOf,
  latestStroke,
  mouseStroke,
  nextFrame,
  nodeAt,
  nodeWorld,
  openBoard,
  pinchDrag,
  PLACING_HINT,
  shot,
  startGame,
  toClient,
  touchDrag,
  zoomAt,
  type Client,
  type Frame,
  type Transform,
} from "./helpers";

const NODES = 12;
const TOUCH_HINT = "Pinch or two-finger drag to move the board";
const MOUSE_HINT = "Hold Space and drag, or scroll, to move the board";
const RELEASE_HINT = "Release near a node to connect";
const PAUSED_HINT = "The line stopped where the board moved it. Pick it up from its end to go on.";
// The colour web/render.ts draws a live stroke in.
const STROKE_COLOUR = "#3B82F6";
// A drawn stroke's samples sit at most this far apart on screen; a pan
// that bridged tail to pointer would leave a gap the size of the pan.
const MAX_GAP_PX = 12;
// Pointer steps stay under this, so a gap above MAX_GAP_PX is the view's.
const STEP_PX = 8;
// A stroke paused by a wheel picks up within 16 px of its tail, and the
// drawn tail may lag the pointer's last sample by the 5 px filter in
// web/main.ts, so the first segment after a pickup is at most this.
const PICKUP_GAP_PX = 16 + 5;

const status = (page: Page) => page.locator("#status");

function add(a: Client, b: Client): Client {
  return { x: a.x + b.x, y: a.y + b.y };
}

function scaled(v: Client, k: number): Client {
  return { x: v.x * k, y: v.y * k };
}

function length(v: Client): number {
  return Math.hypot(v.x, v.y);
}

// How far a client point sits outside the canvas, 0 or less when inside.
function overshoot(frame: Frame, at: Client): number {
  return Math.max(
    frame.left - at.x,
    at.x - (frame.left + frame.width),
    frame.top - at.y,
    at.y - (frame.top + frame.height),
  );
}

// Distance from a point to a segment, all in world units.
function segmentDistance(p: Client, a: Client, b: Client): number {
  const ab = { x: b.x - a.x, y: b.y - a.y };
  const t = Math.max(
    0,
    Math.min(1, ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / (ab.x * ab.x + ab.y * ab.y)),
  );
  return Math.hypot(p.x - a.x - ab.x * t, p.y - a.y - ab.y * t);
}

interface Plan {
  // The zoomed view, anchored on node 0.
  t: Transform;
  // Node 0 and the stroke's resting point toward the target, on screen.
  a: Client;
  rest: Client;
  // The node that sits off the canvas after the zoom.
  target: number;
  // What the fingers move by to bring the target 40 px onto the canvas.
  pan: Client;
  // The view after the pan and where the target sits on it.
  after: Transform;
  end: Client;
}

// The nearest point to `at` that sits `margin` px inside the canvas.
function within(frame: Frame, at: Client, margin: number): Client {
  return {
    x: Math.min(frame.left + frame.width - margin, Math.max(frame.left + margin, at.x)),
    y: Math.min(frame.top + frame.height - margin, Math.max(frame.top + margin, at.y)),
  };
}

// A zoomed stroke from node 0 toward a node the zoom pushed off the canvas,
// and the pan that brings that node onto the canvas with node 0 still on it.
function plan(frame: Frame, zoom: number): Plan {
  const fitted = fit(frame);
  const a = nodeAt(frame, fitted, 0, NODES);
  const t = clampToViewport(frame, zoomAt(frame, fitted, a, zoom));
  const worlds = Array.from({ length: NODES }, (_, i) => nodeWorld(i, NODES, frame.board));
  for (let target = 1; target < NODES; target++) {
    const b = nodeAt(frame, t, target, NODES);
    if (overshoot(frame, b) < 40) continue;
    const clear = worlds.every(
      (w, i) => i === 0 || i === target || segmentDistance(w, worlds[0]!, worlds[target]!) > 60,
    );
    if (!clear) continue;
    const end = within(frame, b, 40);
    const pan = { x: end.x - b.x, y: end.y - b.y };
    if (length(pan) < 120 || overshoot(frame, add(a, pan)) > -30) continue;
    const after = { ...t, tx: t.tx + pan.x, ty: t.ty + pan.y };
    const clamped = clampToViewport(frame, after);
    if (Math.abs(clamped.tx - after.tx) > 0.5 || Math.abs(clamped.ty - after.ty) > 0.5) continue;
    const u = scaled({ x: b.x - a.x, y: b.y - a.y }, 1 / length({ x: b.x - a.x, y: b.y - a.y }));
    let reach = 0;
    while (overshoot(frame, add(a, scaled(u, reach + 1))) < -40) reach++;
    if (reach < 60) continue;
    return { t, a, rest: add(a, scaled(u, reach)), target, pan, after, end };
  }
  throw new Error("no node sits off the zoomed canvas within reach of a pan");
}

// A point of the page beside the canvas, where a second finger misses it.
function beside(frame: Frame): Client {
  return { x: Math.max(2, frame.left / 2), y: frame.top + frame.height / 2 };
}

// A spot on the canvas at least 35 px from every node, where a press
// places the dot without touching anything else.
function clearSpot(frame: Frame, t: Transform): Client {
  const nodes = Array.from({ length: NODES }, (_, i) => nodeAt(frame, t, i, NODES));
  for (let y = 30; y < frame.height - 30; y += 20) {
    for (let x = 30; x < frame.width - 30; x += 20) {
      const at = { x: frame.left + x, y: frame.top + y };
      if (nodes.every((n) => Math.hypot(n.x - at.x, n.y - at.y) >= 35)) return at;
    }
  }
  throw new Error("no clear spot on the canvas");
}

// The longest gap between consecutive points of the applied stroke, in
// screen px at the given scale.
async function longestGap(page: Page, scale: number): Promise<{ gap: number; points: number }> {
  await expect.poll(() => latestStroke(page), { timeout: 15_000 }).not.toBeNull();
  const points = (await latestStroke(page))!;
  let gap = 0;
  for (let i = 1; i < points.length; i++) {
    gap = Math.max(
      gap,
      Math.hypot(points[i]!.x - points[i - 1]!.x, points[i]!.y - points[i - 1]!.y),
    );
  }
  return { gap: gap * scale, points: points.length };
}

// Distance from a client point to the nearest stroke pixel within 20 px,
// or Infinity when none is drawn there.
async function inkGap(page: Page, at: Client): Promise<number> {
  await nextFrame(page);
  return page.evaluate(
    ({ x, y, hex }) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d")!;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      const want = [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
      const half = 20;
      const size = Math.round(2 * half * dpr);
      const left = Math.round((x - rect.left - half) * dpr);
      const top = Math.round((y - rect.top - half) * dpr);
      const { data } = ctx.getImageData(left, top, size, size);
      let best = Infinity;
      for (let py = 0; py < size; py++) {
        for (let px = 0; px < size; px++) {
          const i = (py * size + px) * 4;
          const [r, g, b] = [data[i]!, data[i + 1]!, data[i + 2]!];
          if (Math.hypot(r - want[0]!, g - want[1]!, b - want[2]!) >= 40) continue;
          best = Math.min(best, Math.hypot(px / dpr - half, py / dpr - half));
        }
      }
      return best;
    },
    { x: at.x, y: at.y, hex: STROKE_COLOUR },
  );
}

// The 8 px disc fills most of a 16 px box; the ring sampler in helpers
// reads the label's antialiasing at dpr 1 and undercounts a zoomed disc.
async function expectDiscAt(page: Page, at: Client): Promise<void> {
  await expect.poll(() => colourPixelsNear(page, at, 8, "#3B82F6")).toBeGreaterThan(80);
}

function record(name: string, value: unknown): void {
  test.info().annotations.push({ type: name, description: String(value) });
}

async function startZoomedGame(page: Page): Promise<Frame> {
  await startGame(page, NODES);
  await expect(status(page)).toHaveText("Draw a line between two nodes");
  await boardSettled(page);
  await nextFrame(page);
  return frameOf(page);
}

function stepsFor(a: Client, b: Client): number {
  return Math.max(1, Math.ceil(length({ x: b.x - a.x, y: b.y - a.y }) / STEP_PX));
}

// An axis and sign with room for a pan of `px` at a transform, as the
// wheel delta the page turns into that pan.
function roomFor(frame: Frame, t: Transform, px: number): Client {
  const board = frame.board * t.scale;
  if (t.tx - px >= frame.width - board) return { x: px, y: 0 };
  if (t.tx + px <= 0) return { x: -px, y: 0 };
  if (t.ty - px >= frame.height - board) return { x: 0, y: px };
  if (t.ty + px <= 0) return { x: 0, y: -px };
  throw new Error(`no room for a ${px} px pan`);
}

async function doubleTap(page: Page, at: Client): Promise<void> {
  await page.touchscreen.tap(at.x, at.y);
  await page.touchscreen.tap(at.x, at.y);
}

async function touchStart(cdp: CDPSession, points: (Client & { id: number })[]): Promise<void> {
  await cdp.send("Input.dispatchTouchEvent", { type: "touchStart", touchPoints: points });
}

async function touchEnd(cdp: CDPSession, points: (Client & { id: number })[]): Promise<void> {
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: points });
}

test.beforeEach(async ({ page }) => {
  await openBoard(page);
});

test.describe("touch", () => {
  test.skip(({ hasTouch }) => !hasTouch, "touch only");

  test("a second finger beside the canvas pans the board under a live stroke", async ({ page }) => {
    const frame = await startZoomedGame(page);
    const p = plan(frame, 2.5);
    await doubleTap(page, p.a);
    await expectNodeDrawnAt(page, p.a);

    const cdp = await page.context().newCDPSession(page);
    await touchStart(cdp, [{ ...p.a, id: 1 }]);
    await touchDrag(cdp, p.a, p.rest, 1, stepsFor(p.a, p.rest));
    await shot(page, "pan-stroke-before");

    const other = beside(frame);
    await touchStart(cdp, [
      { ...p.rest, id: 1 },
      { ...other, id: 2 },
    ]);
    const restAfter = add(p.rest, p.pan);
    await pinchDrag(cdp, [p.rest, other], [restAfter, add(other, p.pan)], 12);
    await touchEnd(cdp, [{ ...add(other, p.pan), id: 2 }]);
    await shot(page, "pan-stroke-panned");

    record("pan px", length(p.pan).toFixed(0));
    await expectNodeDrawnAt(page, add(p.a, p.pan));
    await expectNodeDrawnAt(page, p.end);
    const gap = await inkGap(page, restAfter);
    record("tail to finger px", gap.toFixed(1));
    expect(gap).toBeLessThanOrEqual(3);

    await touchDrag(cdp, restAfter, p.end, 1, stepsFor(restAfter, p.end));
    await touchEnd(cdp, []);
    await expect(status(page)).toHaveText(PLACING_HINT);
    await shot(page, "pan-stroke-locked");

    const spot = clearSpot(frame, p.after);
    await page.touchscreen.tap(spot.x, spot.y);
    const stroke = await longestGap(page, p.after.scale);
    record("stroke points", stroke.points);
    record("longest gap px", stroke.gap.toFixed(1));
    expect(stroke.gap).toBeLessThan(MAX_GAP_PX);
  });

  test("pinching out to fit mid-stroke keeps the tail under the finger", async ({ page }) => {
    const frame = await startZoomedGame(page);
    const fitted = fit(frame);
    const p = plan(frame, 2.5);
    await doubleTap(page, p.a);
    await expectNodeDrawnAt(page, p.a);

    const cdp = await page.context().newCDPSession(page);
    await touchStart(cdp, [{ ...p.a, id: 1 }]);
    await touchDrag(cdp, p.a, p.rest, 1, stepsFor(p.a, p.rest));
    // The second finger closes in until the scale caps at fit, where the
    // bounded view would have to move the tail away from the finger.
    const other = beside(frame);
    const closer = add(p.rest, scaled({ x: other.x - p.rest.x, y: other.y - p.rest.y }, 0.3));
    await touchStart(cdp, [
      { ...p.rest, id: 1 },
      { ...other, id: 2 },
    ]);
    await pinchDrag(cdp, [p.rest, other], [p.rest, closer], 10);
    await touchEnd(cdp, [{ ...closer, id: 2 }]);
    await shot(page, "pan-stroke-fit-unbounded");
    const gap = await inkGap(page, p.rest);
    record("tail to finger px", gap.toFixed(1));
    expect(gap).toBeLessThanOrEqual(3);

    // At fit about the finger: the tail's world point stays under p.rest.
    const tailWorld = {
      x: (p.rest.x - frame.left - p.t.tx) / p.t.scale,
      y: (p.rest.y - frame.top - p.t.ty) / p.t.scale,
    };
    const unbounded = {
      scale: fitted.scale,
      tx: p.rest.x - frame.left - tailWorld.x * fitted.scale,
      ty: p.rest.y - frame.top - tailWorld.y * fitted.scale,
    };
    expect(Math.hypot(unbounded.tx - fitted.tx, unbounded.ty - fitted.ty)).toBeGreaterThan(10);
    const end = nodeAt(frame, unbounded, p.target, NODES);
    await touchDrag(cdp, p.rest, end, 1, stepsFor(p.rest, end));
    await touchEnd(cdp, []);
    await expect(status(page)).toHaveText(PLACING_HINT);
    await expectNodeDrawnAt(page, nodeAt(frame, fitted, p.target, NODES));
    await shot(page, "pan-stroke-fit-locked");

    const spot = clearSpot(frame, fitted);
    await page.touchscreen.tap(spot.x, spot.y);
    const stroke = await longestGap(page, fitted.scale);
    record("stroke points", stroke.points);
    record("longest gap px", stroke.gap.toFixed(1));
    expect(stroke.gap).toBeLessThan(MAX_GAP_PX);
  });

  test("the drawing finger lifting during a pinch ends the stroke there", async ({ page }) => {
    const frame = await startZoomedGame(page);
    const p = plan(frame, 2.5);
    await doubleTap(page, p.a);
    await expectNodeDrawnAt(page, p.a);

    const cdp = await page.context().newCDPSession(page);
    await touchStart(cdp, [{ ...p.a, id: 1 }]);
    await touchDrag(cdp, p.a, p.rest, 1);
    const other = beside(frame);
    const nudge = scaled(p.pan, 40 / length(p.pan));
    await touchStart(cdp, [
      { ...p.rest, id: 1 },
      { ...other, id: 2 },
    ]);
    await pinchDrag(cdp, [p.rest, other], [add(p.rest, nudge), add(other, nudge)], 6);
    await touchEnd(cdp, [{ ...add(p.rest, nudge), id: 1 }]);
    await expect(status(page)).toHaveText(RELEASE_HINT);

    const node = add(p.a, nudge);
    await touchDrag(cdp, add(other, nudge), node, 2);
    await touchEnd(cdp, []);
    await nextFrame(page);
    await expect(status(page)).toHaveText(RELEASE_HINT);
    await expect(page.locator("#cancelBtn")).toBeHidden();
    await shot(page, "pan-stroke-finger-lifted");
  });

  test("a zoomed stroke at the edge gets the pan hint once, and none at fit", async ({ page }) => {
    const frame = await startZoomedGame(page);
    const a = nodeAt(frame, fit(frame), 0, NODES);
    await doubleTap(page, a);
    await expectNodeDrawnAt(page, a);
    await page.evaluate((text) => {
      const el = document.getElementById("status")!;
      const counter = window as unknown as { hintCount: number };
      counter.hintCount = 0;
      new MutationObserver(() => {
        if (el.textContent === text) counter.hintCount++;
      }).observe(el, { childList: true });
    }, TOUCH_HINT);
    const hints = () => page.evaluate(() => (window as unknown as { hintCount: number }).hintCount);

    const edge = { x: frame.left + frame.width - 4, y: a.y };
    const cdp = await page.context().newCDPSession(page);
    await touchStart(cdp, [{ ...a, id: 1 }]);
    await touchDrag(cdp, a, edge, 1);
    await expect(status(page)).toHaveText(TOUCH_HINT);
    await shot(page, "pan-hint");
    await touchDrag(cdp, edge, a, 1);
    await touchDrag(cdp, a, edge, 1);
    await touchEnd(cdp, []);
    await expect(status(page)).toHaveText(RELEASE_HINT);
    expect(await hints()).toBe(1);

    await page.waitForTimeout(400);
    await doubleTap(page, a);
    await expectNodeDrawnAt(page, a);
    await touchStart(cdp, [{ ...a, id: 1 }]);
    await touchDrag(cdp, a, edge, 1);
    await nextFrame(page);
    await expect(status(page)).toHaveText(RELEASE_HINT);
    await touchEnd(cdp, []);
    expect(await hints()).toBe(1);
  });
});

test.describe("mouse", () => {
  test.skip(({ hasTouch }) => hasTouch, "mouse only");

  // Zooms at node 0 a notch at a time until some node sits off the canvas
  // within reach of a pan.
  async function zoomAtNode0(page: Page, frame: Frame): Promise<Plan> {
    const a = nodeAt(frame, fit(frame), 0, NODES);
    await page.evaluate(() => (document.activeElement as HTMLElement | null)?.blur());
    let zoom = 1;
    for (let notch = 0; notch < 3; notch++) {
      zoom *= 2 ** (-(await ctrlWheel(page, a, -100)) / 100);
      try {
        const p = plan(frame, zoom);
        await expectNodeDrawnAt(page, p.a);
        return p;
      } catch {
        // Nothing is off the canvas yet at this zoom.
      }
    }
    throw new Error("no zoom pushes a node off the canvas within reach of a pan");
  }

  test("a wheel pan under a live stroke pauses it until the cursor reaches the tail", async ({
    page,
  }) => {
    const frame = await startZoomedGame(page);
    const p = await zoomAtNode0(page, frame);
    const u = scaled(
      { x: p.rest.x - p.a.x, y: p.rest.y - p.a.y },
      1 / length({ x: p.rest.x - p.a.x, y: p.rest.y - p.a.y }),
    );
    let edge = p.rest;
    while (overshoot(frame, add(edge, u)) < -4) edge = add(edge, u);

    await page.mouse.move(p.a.x, p.a.y);
    await page.mouse.down();
    await page.mouse.move(edge.x, edge.y, { steps: stepsFor(p.a, edge) });
    await expect(status(page)).toHaveText(MOUSE_HINT);
    await shot(page, "pan-wheel-hint");

    // One notch carries the whole pan.
    await page.mouse.wheel(-p.pan.x, -p.pan.y);
    const after = { ...p.t, tx: p.t.tx + p.pan.x, ty: p.t.ty + p.pan.y };
    const end = toClient(frame, after, nodeWorld(p.target, NODES, frame.board));
    await expectDiscAt(page, end);
    const tail = add(edge, p.pan);
    const gap = await inkGap(page, tail);
    record("tail moved with board, off by px", gap.toFixed(1));
    expect(gap).toBeLessThanOrEqual(3);
    await shot(page, "pan-wheel-panned");

    await page.mouse.move(tail.x, tail.y, { steps: stepsFor(edge, tail) });
    await page.mouse.move(end.x, end.y, { steps: stepsFor(tail, end) });
    await page.mouse.up();
    await expect(status(page)).toHaveText(PLACING_HINT);
    await shot(page, "pan-wheel-locked");

    const spot = clearSpot(frame, after);
    await page.mouse.click(spot.x, spot.y);
    const stroke = await longestGap(page, after.scale);
    record("stroke points", stroke.points);
    record("longest gap px", stroke.gap.toFixed(1));
    expect(stroke.gap).toBeLessThanOrEqual(PICKUP_GAP_PX);
  });

  test("a paused stroke takes nothing until the cursor reaches the tail", async ({ page }) => {
    const frame = await startZoomedGame(page);
    const p = await zoomAtNode0(page, frame);
    await page.mouse.move(p.a.x, p.a.y);
    await page.mouse.down();
    await page.mouse.move(p.rest.x, p.rest.y, { steps: stepsFor(p.a, p.rest) });
    await page.mouse.wheel(-p.pan.x, -p.pan.y);
    const tail = add(p.rest, p.pan);

    // A detour 60 px to the side of the tail draws nothing.
    const big = roomFor(frame, p.after, 60);
    const aside = add(p.rest, { x: big.y, y: big.x });
    await page.mouse.move(aside.x, aside.y, { steps: stepsFor(p.rest, aside) });
    expect(await inkGap(page, aside)).toBe(Infinity);
    expect(await inkGap(page, p.rest)).toBe(Infinity);
    await shot(page, "pan-wheel-detour");

    await page.mouse.move(tail.x, tail.y, { steps: stepsFor(aside, tail) });
    await page.mouse.move(p.end.x, p.end.y, { steps: stepsFor(tail, p.end) });
    await page.mouse.up();
    await expect(status(page)).toHaveText(PLACING_HINT);

    const spot = clearSpot(frame, p.after);
    await page.mouse.click(spot.x, spot.y);
    const stroke = await longestGap(page, p.after.scale);
    record("longest gap px", stroke.gap.toFixed(1));
    expect(stroke.gap).toBeLessThanOrEqual(PICKUP_GAP_PX);
  });

  test("releasing on a node without picking the paused stroke up drops it and says so", async ({
    page,
  }) => {
    const frame = await startZoomedGame(page);
    const p = await zoomAtNode0(page, frame);
    await page.mouse.move(p.a.x, p.a.y);
    await page.mouse.down();
    await page.mouse.move(p.rest.x, p.rest.y, { steps: stepsFor(p.a, p.rest) });
    await page.mouse.wheel(-p.pan.x, -p.pan.y);
    const tail = add(p.rest, p.pan);
    // Straight to the node, clear of the carried tail the whole way.
    expect(segmentDistance(tail, p.rest, p.end)).toBeGreaterThan(20);
    await page.mouse.move(p.end.x, p.end.y, { steps: stepsFor(p.rest, p.end) });
    await page.mouse.up();
    await expect(status(page)).toHaveText(PAUSED_HINT);
    await expect(page.locator("#cancelBtn")).toBeHidden();
    expect(await latestStroke(page)).toBeNull();
    await shot(page, "pan-wheel-dropped");

    await mouseStroke(page, add(p.a, p.pan), p.end);
    await expect(status(page)).toHaveText(PLACING_HINT);
  });

  test("space drag under a live stroke moves the board with the tail", async ({ page }) => {
    const frame = await startZoomedGame(page);
    const p = await zoomAtNode0(page, frame);
    await page.mouse.move(p.a.x, p.a.y);
    await page.mouse.down();
    await page.mouse.move(p.rest.x, p.rest.y, { steps: stepsFor(p.a, p.rest) });

    await page.keyboard.down("Space");
    const restAfter = add(p.rest, p.pan);
    await page.mouse.move(restAfter.x, restAfter.y, { steps: 8 });
    await page.keyboard.up("Space");
    await expectDiscAt(page, add(p.a, p.pan));
    await expectDiscAt(page, p.end);
    const gap = await inkGap(page, restAfter);
    record("tail to cursor px", gap.toFixed(1));
    expect(gap).toBeLessThanOrEqual(3);
    await shot(page, "pan-space-panned");

    await page.mouse.move(p.end.x, p.end.y, { steps: stepsFor(restAfter, p.end) });
    await page.mouse.up();
    await expect(status(page)).toHaveText(PLACING_HINT);

    const spot = clearSpot(frame, p.after);
    await page.mouse.click(spot.x, spot.y);
    const stroke = await longestGap(page, p.after.scale);
    record("stroke points", stroke.points);
    record("longest gap px", stroke.gap.toFixed(1));
    expect(stroke.gap).toBeLessThan(MAX_GAP_PX);
  });
});
