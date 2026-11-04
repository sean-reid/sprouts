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
  shot,
  toClient,
  type Client,
} from "./helpers";

// The settle eases the board over SETTLE_MS in web/main.ts; a frame or two
// of slack on top.
const SETTLE_BOUND_MS = 650;
// With reduced motion the snapshot lands in the first frame after the status.
const INSTANT_BOUND_MS = 100;
// How long the probe samples frames after the worker's reply.
const PROBE_MS = 900;

interface SettleProbe {
  // When the worker reported the board with the expected node count.
  replyAt: number | null;
  // Centre of node 3's hit target at each painted frame after the reply,
  // which follows the geometry the board draws.
  frames: { at: number; x: number; y: number }[];
}

// Samples every painted frame from now on. Once the worker intercept has
// recorded a board of `nodes` nodes, the centre of node 3's hit target is
// kept per frame for the next PROBE_MS.
function armSettleProbe(page: Page, nodes: number): Promise<void> {
  return page.evaluate(
    ([nodes, probeMs]) => {
      const probe: SettleProbe = { replyAt: null, frames: [] };
      const hooks = window as unknown as {
        settleProbe: SettleProbe;
        settleProbeMs: number;
        latestNodes?: unknown[];
      };
      hooks.settleProbe = probe;
      hooks.settleProbeMs = probeMs;
      const sample = (): void => {
        const now = performance.now();
        if (probe.replyAt === null) {
          if ((hooks.latestNodes?.length ?? 0) >= nodes) probe.replyAt = now;
          requestAnimationFrame(sample);
          return;
        }
        // The next board, the AI's reply, starts its own animation.
        if ((hooks.latestNodes?.length ?? 0) > nodes) return;
        const target = document.querySelector('[data-node="3"]');
        if (target) {
          const rect = target.getBoundingClientRect();
          probe.frames.push({
            at: now,
            x: rect.left + rect.width / 2,
            y: rect.top + rect.height / 2,
          });
        }
        if (now - probe.replyAt < hooks.settleProbeMs) requestAnimationFrame(sample);
      };
      requestAnimationFrame(sample);
    },
    [nodes, PROBE_MS] as const,
  );
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
  await armSettleProbe(page, 5);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await page.waitForTimeout(1300);
  await shot(page, "settle-after");
  return { drawn, scale: fit(frame).scale };
}

test("a line drawn past a node settles the node away and keeps every node on the board", async ({
  page,
}) => {
  await openBoard(page);
  const { drawn } = await settleExchange(page);
  const settled = await nodeOnScreen(page, 3, 4);
  const moved = distance(settled, drawn);
  test.info().annotations.push({ type: "node-3-shift-px", description: moved.toFixed(2) });
  expect(moved).toBeGreaterThan(1);
  await expectNodeDrawnAt(page, settled);

  // Every node is drawn where the settled board puts it, inside the board.
  const record = await savedRecord(page);
  const frame = await frameOf(page);
  const drawnNodes = [...(record.nodes ?? []), ...record.moves.map((m) => m.newNode)].map(
    ([x, y]) => toClient(frame, fit(frame), { x, y }),
  );
  const discs = await recordNodesOnScreen(page, record);
  expect(discs).toHaveLength(drawnNodes.length);
  let farthest = 0;
  for (const [i, at] of drawnNodes.entries()) {
    const disc = discs[i]!;
    farthest = Math.max(farthest, distance(disc, at));
    expect(disc.x).toBeGreaterThanOrEqual(frame.left);
    expect(disc.x).toBeLessThanOrEqual(frame.left + frame.width);
    expect(disc.y).toBeGreaterThanOrEqual(frame.top);
    expect(disc.y).toBeLessThanOrEqual(frame.top + frame.height);
    await expectNodeDrawnAt(page, disc);
  }
  test
    .info()
    .annotations.push({ type: "farthest-node-shift-px", description: farthest.toFixed(2) });

  // The human move's settle moves node 3's target from the drawn spot to
  // its settled one through positions off both, and stops within the bound.
  const probe = await readSettleProbe(page);
  expect(probe.replyAt).not.toBeNull();
  const motion = describeMotion(probe, drawn);
  test.info().annotations.push({ type: "settle-animation", description: motion.summary });
  expect(motion.end).toBeGreaterThan(1);
  expect(motion.between).toBeGreaterThan(0);
  expect(motion.took).toBeLessThanOrEqual(SETTLE_BOUND_MS);
});

interface Motion {
  // Frames whose target centre is more than half a pixel from both the
  // drawn spot and the final spot.
  between: number;
  // Distance from the drawn spot to the final spot.
  end: number;
  // Time from the reply to the last frame that moved.
  took: number;
  summary: string;
}

function describeMotion(probe: SettleProbe, drawn: Client): Motion {
  const last = probe.frames.at(-1);
  if (!last) return { between: 0, end: 0, took: 0, summary: "no frames" };
  let between = 0;
  let took = 0;
  let previous: Client | null = null;
  for (const frame of probe.frames) {
    if (distance(frame, drawn) > 0.5 && distance(frame, last) > 0.5) between++;
    if (previous && distance(frame, previous) > 1e-3) took = frame.at - probe.replyAt!;
    previous = frame;
  }
  const end = distance(last, drawn);
  return {
    between,
    end,
    took,
    summary: `${took.toFixed(0)} ms, ${between} frames between, ${end.toFixed(2)} px, ${probe.frames.length} frames`,
  };
}

test("with reduced motion the settled board lands in the first frame", async ({ page }) => {
  await page.emulateMedia({ reducedMotion: "reduce" });
  await openBoard(page);
  const { drawn } = await settleExchange(page);
  const settled = await nodeOnScreen(page, 3, 4);
  const moved = distance(settled, drawn);
  expect(moved).toBeGreaterThan(1);
  await expectNodeDrawnAt(page, settled);
  const probe = await readSettleProbe(page);
  expect(probe.replyAt).not.toBeNull();
  const motion = describeMotion(probe, drawn);
  test.info().annotations.push({ type: "settle-instant", description: motion.summary });
  expect(motion.end).toBeGreaterThan(1);
  expect(motion.between).toBe(0);
  expect(motion.took).toBeLessThanOrEqual(INSTANT_BOUND_MS);
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
