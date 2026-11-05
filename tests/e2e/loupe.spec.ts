import { readFileSync } from "node:fs";
import { expect, test, type Page } from "@playwright/test";
import { LOUPE_FOLLOW } from "../../web/loupe";
import type { Fixture } from "../../web/record";
import {
  alongLine,
  boardNodes,
  expectNodeDrawnAt,
  frameOf,
  nextFrame,
  nodeOnScreen,
  openBoard,
  playExchange,
  recordNodesOnScreen,
  ringPixels,
  shot,
  startGame,
  touchDrag,
  type Client,
  type Frame,
} from "./helpers";

const LIFT = 90;
const RADIUS = 60;

// The lens for a finger clear of the edges: LIFT straight above it. Where the
// top edge makes it swing, the tests read the ring off the canvas instead.
function lensCentre(finger: Client): Client {
  return { x: finger.x, y: finger.y - LIFT };
}

// Where the ring is drawn, in canvas-local px: the midpoint of the amber
// pixels sideways, and the ring's bottom edge less the radius, since the top
// of the ring can touch the canvas edge.
function lensBox(): Client | null {
  const canvas = document.getElementById("canvas") as HTMLCanvasElement;
  const ctx = canvas.getContext("2d")!;
  const rect = canvas.getBoundingClientRect();
  const dpr = canvas.width / rect.width;
  const { data, width } = ctx.getImageData(0, 0, canvas.width, canvas.height);
  let minX = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (let i = 0; i < data.length; i += 4) {
    if (Math.hypot(data[i]! - 0xfb, data[i + 1]! - 0xbf, data[i + 2]! - 0x24) >= 40) continue;
    const x = (i / 4) % width;
    const y = Math.floor(i / 4 / width);
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }
  if (maxY < 0) return null;
  return { x: (minX + maxX) / 2 / dpr, y: maxY / dpr - 61 };
}

async function ringCentre(page: Page): Promise<Client> {
  const at = await page.evaluate(lensBox);
  expect(at).not.toBeNull();
  return at!;
}

// The ring once it has stopped sliding: two frames in a row at one spot.
async function settledLens(page: Page): Promise<Client> {
  let last = await ringCentre(page);
  for (let i = 0; i < 150; i++) {
    await nextFrame(page);
    const now = await ringCentre(page);
    if (Math.hypot(now.x - last.x, now.y - last.y) < 0.5) return now;
    last = now;
  }
  throw new Error("the lens did not settle");
}

// One frame of the recording: the ring and the finger, canvas-local.
interface LensFrame {
  lens: Client | null;
  finger: Client | null;
}

// Reads the ring's position every frame until stopped, with the finger's last
// pointer position, so a jump between two touch samples shows up as one
// large frame-to-frame move.
function startLensRecorder(page: Page): Promise<void> {
  return page.evaluate((read) => {
    const w = window as unknown as { lensFrames: LensFrame[]; lensStop: boolean };
    w.lensFrames = [];
    w.lensStop = false;
    const box = new Function(`return (${read})()`) as () => Client | null;
    const canvas = document.getElementById("canvas")!;
    let finger: Client | null = null;
    canvas.addEventListener("pointermove", (e) => {
      const rect = canvas.getBoundingClientRect();
      finger = { x: e.clientX - rect.left, y: e.clientY - rect.top };
    });
    const tick = (): void => {
      if (w.lensStop) return;
      w.lensFrames.push({ lens: box(), finger });
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }, lensBox.toString());
}

function stopLensRecorder(page: Page): Promise<LensFrame[]> {
  return page.evaluate(() => {
    const w = window as unknown as { lensFrames: LensFrame[]; lensStop: boolean };
    w.lensStop = true;
    return w.lensFrames;
  });
}

// Which side of the finger the lens is on, or 0 while it passes above or below.
function sideOf(lens: Client, finger: Client): -1 | 0 | 1 {
  const dx = lens.x - finger.x;
  return Math.abs(dx) < RADIUS / 2 ? 0 : dx > 0 ? 1 : -1;
}

// True when the lens is within 30 degrees of straight below the finger.
function isUnder(lens: Client, finger: Client): boolean {
  const angle = Math.atan2(lens.x - finger.x, finger.y - lens.y);
  return Math.abs(angle) > (Math.PI * 5) / 6;
}

interface Recording {
  largest: number;
  slides: number;
  flips: number;
  longestUnder: number;
}

// No frame moves the lens further than the finger's 8 px sample plus the arc
// the follower allows for it, so a flip to the other side of the finger fails
// here. While a side fits the lens never rests under the finger, and a pass
// under it on the way to the other side lasts a few frames at most.
function expectNoJump(frame: Frame, frames: LensFrame[]): Recording {
  let largest = 0;
  let slides = 0;
  let flips = 0;
  let longestUnder = 0;
  let under = 0;
  let side: -1 | 0 | 1 = 0;
  for (let i = 1; i < frames.length; i++) {
    const a = frames[i - 1]!;
    const b = frames[i]!;
    if (!a.lens || !b.lens) continue;
    const moved = Math.hypot(b.lens.x - a.lens.x, b.lens.y - a.lens.y);
    largest = Math.max(largest, moved);
    const fingerMoved =
      a.finger && b.finger ? Math.hypot(b.finger.x - a.finger.x, b.finger.y - a.finger.y) : 0;
    if (fingerMoved < 0.5 && moved > 2) slides++;
    if (!b.finger) continue;
    const now = sideOf(b.lens, b.finger);
    if (now !== 0 && side !== 0 && now !== side) flips++;
    if (now !== 0) side = now;
    if (isUnder(b.lens, b.finger) && aSideFits(frame, b.finger)) {
      under++;
      longestUnder = Math.max(longestUnder, under);
      if (fingerMoved < 0.5 && moved < 0.5)
        throw new Error(`the lens rested under the finger at frame ${i}`);
    } else {
      under = 0;
    }
  }
  expect(largest).toBeLessThanOrEqual(8 + 8 * LOUPE_FOLLOW + 1);
  expect(longestUnder).toBeLessThanOrEqual(18);
  return { largest, slides, flips, longestUnder };
}

// Whether a lens beside the finger fits on either side at the finger's height.
function aSideFits(frame: Frame, local: Client): boolean {
  const c = Math.max(-1, Math.min(1, (local.y - RADIUS) / LIFT));
  const span = LIFT * Math.sqrt(1 - c * c);
  return local.x + span <= frame.width - RADIUS + 1 || local.x - span >= RADIUS - 1;
}

// A settled lens keeps its lift from the finger, the whole disc on the canvas,
// and near the top edge sits beside the finger, not under the hand, whenever
// a side fits.
function expectLensPlaced(frame: Frame, lens: Client, finger: Client): void {
  const local = { x: finger.x - frame.left, y: finger.y - frame.top };
  const lift = Math.hypot(lens.x - local.x, lens.y - local.y);
  expect(lift).toBeGreaterThan(LIFT - 4);
  expect(lift).toBeLessThan(LIFT + 4);
  // The 10 degree side preference can hold the lens 16 px below the highest spot.
  if (local.y < LIFT + RADIUS && aSideFits(frame, local)) {
    expect(lens.y).toBeLessThanOrEqual(Math.max(local.y + RADIUS / 2, RADIUS + 16));
  }
  expect(lens.x).toBeGreaterThanOrEqual(RADIUS - 2);
  expect(lens.x).toBeLessThanOrEqual(frame.width - RADIUS + 2);
  expect(lens.y).toBeGreaterThanOrEqual(RADIUS - 2);
  expect(lens.y).toBeLessThanOrEqual(frame.height - RADIUS + 2);
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
  await expect.poll(() => ringSamplesAround(page, lensCentre(near))).toBeGreaterThanOrEqual(6);
  await shot(page, "loupe-near-line");

  await touchDrag(cdp, near, clear);
  await expect.poll(() => ringPixels(page)).toBe(0);
  await shot(page, "loupe-clear");

  await touchDrag(cdp, clear, near);
  await expect.poll(() => ringSamplesAround(page, lensCentre(near))).toBeGreaterThanOrEqual(6);
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
  await expect.poll(() => ringSamplesAround(page, lensCentre(near))).toBeGreaterThanOrEqual(6);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
});

// A recorded game replayed as drawn, so its geometry does not depend on
// the settle: node 5 sits 151 units from the top, within the lens's lift
// and radius of the top edge at every viewport.
const TOP_NODE_GAME = "fixtures/games/false-game-over-4n-1000.json";

test("the lens slides beside the finger across the top edge without a jump", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  // The board after the eighth move, with the human to move and the stroke
  // free to start.
  const fixture = JSON.parse(readFileSync(TOP_NODE_GAME, "utf8")) as Fixture;
  const record = { firstPlayer: "human", cursor: 8, ...fixture, moves: fixture.moves.slice(0, 8) };
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(record));
  await page.goto("/");
  await expect(page.locator("#moveList li")).toHaveCount(8, { timeout: 60_000 });
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await nextFrame(page);
  const frame = await frameOf(page);
  // A stroke that climbs past the highest node crosses the zone where the
  // lens leaves its spot above the finger.
  const onScreen = await recordNodesOnScreen(page, record);
  const highest = onScreen.reduce((best, at, id) => (at.y < onScreen[best]!.y ? id : best), 0);
  const top = onScreen[highest]!;
  expect(top.y - frame.top).toBeLessThan(LIFT + RADIUS);
  await expectNodeDrawnAt(page, top);
  const cdp = await page.context().newCDPSession(page);

  // The lowest node with a line left starts the stroke, clear of the climb.
  const live = (
    await page
      .locator("#nodeTargets button")
      .evaluateAll((buttons) => buttons.map((b) => b.getAttribute("aria-label") ?? ""))
  )
    .map((name) => Number(/Node (\d+)/.exec(name)?.[1]))
    .filter(Number.isFinite);
  expect(live.length).toBeGreaterThan(0);
  const lowest = live.reduce((best, id) => (onScreen[id]!.y > onScreen[best]!.y ? id : best));
  const start = onScreen[lowest]!;
  const below = { x: top.x, y: Math.min(top.y + 40, frame.top + frame.height - 20) };
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...start, id: 0 }],
  });
  await touchDrag(cdp, start, below);

  // The finger climbs 8 px at a time to 12 px above the node.
  await startLensRecorder(page);
  let finger = below;
  let lens: Client | null = null;
  const steps: string[] = [];
  while (finger.y > top.y - 12) {
    const next = { x: finger.x, y: Math.max(finger.y - 8, top.y - 12) };
    await touchDrag(cdp, finger, next, 0, 2);
    finger = next;
    await expect.poll(() => ringPixels(page)).toBeGreaterThan(0);
    lens = await settledLens(page);
    expectLensPlaced(frame, lens, finger);
    steps.push(`${Math.round(finger.y - frame.top)}:${Math.round(lens.x)},${Math.round(lens.y)}`);
  }
  const { largest } = expectNoJump(frame, await stopLensRecorder(page));
  test.info().annotations.push({
    type: "lens-centres",
    description: `${steps.join(" ")} largest frame move ${largest.toFixed(1)}`,
  });
  expect(Math.abs(lens!.x - (finger.x - frame.left))).toBeGreaterThan(RADIUS);
  await shot(page, "loupe-top-slide");
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
});

test("the lens keeps its lift and stays on the canvas along the whole top edge", async ({
  page,
}) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  await startGame(page, 4);
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  const frame = await frameOf(page);
  const cdp = await page.context().newCDPSession(page);

  // A human line that runs along the top edge, so a finger tracing it keeps
  // the lens open from one side of the board to the other.
  const nodes = await boardNodes(page);
  const ids = [...nodes.keys()];
  const screen = new Map<number, Client>();
  for (const id of ids) screen.set(id, await nodeOnScreen(page, id, ids.length));
  const leftmost = ids.reduce((a, b) => (screen.get(a)!.x < screen.get(b)!.x ? a : b));
  const rightmost = ids.reduce((a, b) => (screen.get(a)!.x > screen.get(b)!.x ? a : b));
  const a = screen.get(leftmost)!;
  const b = screen.get(rightmost)!;
  const high = frame.top + 20;
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(a.x, high, { steps: 10 });
  await page.mouse.move(b.x, high, { steps: 20 });
  await page.mouse.move(b.x, b.y, { steps: 10 });
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(/Drag the dot/);
  await page.mouse.click(frame.left + frame.width / 2, frame.top + frame.height - 20);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });

  // The run of that line through the zone where the top edge swings the
  // lens aside, wherever the settle left it, resampled every 8 px, traced
  // 12 px below it from left to right.
  const samples: Client[] = [];
  for (let i = 0; i <= 300; i++) samples.push((await alongLine(page, 0, i / 300)).at);
  const highest = Math.min(...samples.map((p) => p.y));
  expect(highest - frame.top).toBeLessThan(LIFT + RADIUS - 40);
  const run: Client[] = samples
    .filter((p) => p.y - frame.top < LIFT + RADIUS - 24)
    .map((p) => ({ x: p.x, y: p.y + 12 }));
  run.sort((p, q) => p.x - q.x);
  const path: Client[] = [run[0]!];
  for (const p of run) {
    const prev = path[path.length - 1]!;
    if (Math.hypot(p.x - prev.x, p.y - prev.y) >= 8) path.push(p);
  }
  expect(path.length).toBeGreaterThan(20);

  const bottom = [...(await boardNodes(page)).keys()].reduce((best, id) => {
    const p = nodes.get(id);
    return p && p.y > (nodes.get(best)?.y ?? -Infinity) ? id : best;
  }, ids[0]!);
  const start = await nodeOnScreen(page, bottom, (await boardNodes(page)).size);
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...start, id: 0 }],
  });
  await touchDrag(cdp, start, path[0]!);
  await startLensRecorder(page);
  let finger = path[0]!;
  for (const [i, next] of path.entries()) {
    await touchDrag(cdp, finger, next, 0, 2);
    finger = next;
    await expect.poll(() => ringPixels(page)).toBeGreaterThan(0);
    expectLensPlaced(frame, await settledLens(page), finger);
    if (i === 1) await shot(page, "loupe-top-edge-left");
  }
  // Past the side limit the spot crosses to the other side of the finger,
  // once, and the lens gets there over frames with the finger still.
  const frames = await stopLensRecorder(page);
  const { largest, slides, flips, longestUnder } = expectNoJump(frame, frames);
  expect(slides).toBeGreaterThanOrEqual(3);
  expect(flips).toBe(1);
  test.info().annotations.push({
    type: "largest-frame-move",
    description: `${largest.toFixed(1)} px, ${slides} frames sliding with the finger still, ${longestUnder} frames passing under`,
  });
  await shot(page, "loupe-top-edge-right");

  // The lens does not flip back on a pixel of jitter here, nor when the
  // finger retraces 80 px of the line: the side it chose holds until an
  // edge forces the other.
  await startLensRecorder(page);
  for (let i = 0; i < 10; i++) {
    const jitter = { x: finger.x + (i % 2 ? 1 : -1), y: finger.y };
    await touchDrag(cdp, finger, jitter, 0, 1);
    finger = jitter;
    await nextFrame(page);
  }
  await settledLens(page);
  for (const next of path.slice(-11, -1).reverse()) {
    await touchDrag(cdp, finger, next, 0, 2);
    finger = next;
    await nextFrame(page);
  }
  await settledLens(page);
  for (const f of await stopLensRecorder(page)) {
    if (f.lens && f.finger) expect(sideOf(f.lens, f.finger)).not.toBe(1);
  }
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
});
