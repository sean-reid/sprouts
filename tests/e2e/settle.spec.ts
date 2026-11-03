import { expect, test, type Page } from "@playwright/test";
import {
  boardNodes,
  clickAway,
  expectDotAt,
  expectNodeDrawnAt,
  fit,
  findDot,
  frameOf,
  nodeAt,
  nodeOnScreen,
  nodeWorld,
  openBoard,
  PLACING_HINT,
  recordNodesOnScreen,
  savedRecord,
  SETTLE_SHIFT_PX,
  shot,
  toClient,
  type Client,
} from "./helpers";

// The settle eases the board over SETTLE_MS in web/main.ts; a frame or two
// of slack on top.
const SETTLE_BOUND_MS = 400;
// With reduced motion the snapshot lands in the first frame after the status.
const INSTANT_BOUND_MS = 100;
// A node moves at most one node spacing per settle (MAX_SHIFT in
// src/settle.rs), and an exchange settles twice.
const SHIFT_UNITS = 2 * 20;

interface SettleProbe {
  // When the worker reported the board with the expected node count.
  replyAt: number | null;
  lastChangeAt: number | null;
  // Painted frames after the reply that differed from the one before them.
  changes: number;
  // Painted frames in the first 300 ms after the reply; a slow runner
  // paints too few of them to see the animation.
  early: number;
}

// Samples every painted frame from now on. Once the worker intercept has
// recorded a board of `nodes` nodes, each later frame is compared with the
// one before it and the time of the last frame that differed is kept.
function armSettleProbe(page: Page, nodes: number): Promise<void> {
  return page.evaluate((nodes) => {
    const probe: SettleProbe = { replyAt: null, lastChangeAt: null, changes: 0, early: 0 };
    const hooks = window as unknown as {
      settleProbe: SettleProbe;
      latestNodes?: unknown[];
    };
    hooks.settleProbe = probe;
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const ctx = canvas.getContext("2d")!;
    let last: Uint8ClampedArray | null = null;
    const sample = (): void => {
      const now = performance.now();
      if (probe.replyAt === null) {
        if ((hooks.latestNodes?.length ?? 0) >= nodes) {
          probe.replyAt = now;
          last = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
        }
        requestAnimationFrame(sample);
        return;
      }
      const { data } = ctx.getImageData(0, 0, canvas.width, canvas.height);
      if (now - probe.replyAt < 300) probe.early++;
      if (last && last.some((v, i) => v !== data[i])) {
        probe.lastChangeAt = now;
        probe.changes++;
      }
      last = data;
      if (now - probe.replyAt < 1200) requestAnimationFrame(sample);
    };
    requestAnimationFrame(sample);
  }, nodes);
}

function readSettleProbe(page: Page): Promise<SettleProbe> {
  return page.evaluate(() => (window as unknown as { settleProbe: SettleProbe }).settleProbe);
}

// A stroke from node 0 to node 1 that passes twelve world units from node
// 3, two units past the clearance the validator asks of a human line, so
// the settle has a shortfall to close on both the node and the line.
async function drawPastNode3(page: Page): Promise<void> {
  const frame = await frameOf(page);
  const t = fit(frame);
  const a = nodeAt(frame, t, 0);
  const b = nodeAt(frame, t, 1);
  const n3 = nodeWorld(3, 4, frame.board);
  const via = toClient(frame, t, { x: n3.x - 12, y: n3.y });
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(via.x, via.y, { steps: 20 });
  await page.mouse.move(b.x, b.y, { steps: 20 });
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
}

function distance(a: Client, b: Client): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

// One exchange past node 3, with the probe armed for the AI's reply.
async function settleExchange(page: Page): Promise<{ drawn: Client; scale: number }> {
  const frame = await frameOf(page);
  const drawn = nodeAt(frame, fit(frame), 3);
  await drawPastNode3(page);
  await expect.poll(() => findDot(page)).not.toBeNull();
  await expectDotAt(page, (await findDot(page))!);
  await shot(page, "settle-before");
  await armSettleProbe(page, 6);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await page.waitForTimeout(1300);
  await shot(page, "settle-after");
  return { drawn, scale: fit(frame).scale };
}

test("a line drawn past a node settles the node away within the stated bound", async ({ page }) => {
  await openBoard(page);
  const { drawn, scale } = await settleExchange(page);
  const settled = await nodeOnScreen(page, 3, 4);
  const moved = distance(settled, drawn);
  test.info().annotations.push({ type: "node-3-shift-px", description: moved.toFixed(2) });
  expect(moved).toBeGreaterThan(1);
  expect(moved).toBeLessThanOrEqual(SHIFT_UNITS * scale + 1);
  await expectNodeDrawnAt(page, settled);

  // Every node sits within the bound of where the record says it was drawn.
  const record = await savedRecord(page);
  const frame = await frameOf(page);
  const drawnNodes = [...(record.nodes ?? []), ...record.moves.map((m) => m.newNode)].map(
    ([x, y]) => toClient(frame, fit(frame), { x, y }),
  );
  const discs = await recordNodesOnScreen(page, record);
  expect(discs).toHaveLength(drawnNodes.length);
  for (const [i, at] of drawnNodes.entries()) {
    expect(distance(discs[i]!, at)).toBeLessThanOrEqual(
      Math.min(SHIFT_UNITS * scale + 1, SETTLE_SHIFT_PX),
    );
  }

  const probe = await readSettleProbe(page);
  expect(probe.replyAt).not.toBeNull();
  const took = probe.lastChangeAt === null ? 0 : probe.lastChangeAt - probe.replyAt!;
  test.info().annotations.push({
    type: "settle-animation-ms",
    description: `${took.toFixed(0)} over ${probe.changes} frames, ${probe.early} early`,
  });
  expect(took).toBeLessThanOrEqual(SETTLE_BOUND_MS);
  // With at least 6 frames in the first 300 ms the animation shows as
  // changed frames; a slower runner cannot tell it from an instant jump.
  if (probe.early >= 6) expect(probe.changes).toBeGreaterThanOrEqual(2);
});

test("with reduced motion the settled board lands in the first frame", async ({ page }) => {
  await page.emulateMedia({ reducedMotion: "reduce" });
  await openBoard(page);
  const { drawn, scale } = await settleExchange(page);
  const settled = await nodeOnScreen(page, 3, 4);
  const moved = distance(settled, drawn);
  expect(moved).toBeGreaterThan(1);
  await expectNodeDrawnAt(page, settled);
  expect(moved).toBeLessThanOrEqual(SHIFT_UNITS * scale + 1);
  const probe = await readSettleProbe(page);
  expect(probe.replyAt).not.toBeNull();
  const took = probe.lastChangeAt === null ? 0 : probe.lastChangeAt - probe.replyAt!;
  test.info().annotations.push({ type: "settle-instant-ms", description: took.toFixed(0) });
  expect(took).toBeLessThanOrEqual(INSTANT_BOUND_MS);
  expect(probe.changes).toBeLessThanOrEqual(1);
});

// The largest offset of a node between two boards, infinite when their
// node sets differ.
function boardOffset(a: Map<number, Client>, b: Map<number, Client>): number {
  if (a.size !== b.size) return Infinity;
  let worst = 0;
  for (const [id, at] of a) {
    const other = b.get(id);
    if (!other) return Infinity;
    worst = Math.max(worst, distance(at, other));
  }
  return worst;
}

test("Back then Forward after a displacing settle lands on the same board", async ({ page }) => {
  await openBoard(page);
  await settleExchange(page);
  const settled = await boardNodes(page);
  expect(settled.size).toBe(6);
  await page.locator("#backBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(page.locator("#moveList [aria-current]")).toHaveCount(0);
  expect((await boardNodes(page)).size).toBe(4);
  await page.locator("#forwardBtn").click();
  await expect(page.locator("#moveList [aria-current]")).toHaveText(/^2\./);
  await expect(page.locator("#status")).toHaveText("Your turn!");
  expect(boardOffset(await boardNodes(page), settled)).toBeLessThanOrEqual(1e-6);
});

test("a reload after Undo and a new stroke rebuilds the same settled board", async ({ page }) => {
  await openBoard(page);
  await settleExchange(page);
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  const a = await nodeOnScreen(page, 0, 4);
  const c = await nodeOnScreen(page, 2, 4);
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(c.x, c.y, { steps: 20 });
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await expect.poll(() => findDot(page)).not.toBeNull();
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  const settled = await boardNodes(page);
  expect(settled.size).toBe(6);
  await page.reload();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 20_000 });
  await expect(page.locator("#moveList li")).toHaveCount(2);
  expect(boardOffset(await boardNodes(page), settled)).toBeLessThanOrEqual(1e-6);
});
