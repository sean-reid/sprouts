import { expect, test, type Page } from "@playwright/test";
import {
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
  statusAt: number | null;
  lastChangeAt: number | null;
  // Painted frames that differed from the one before them.
  changes: number;
}

// From the moment the status reads `message`, compares every painted frame
// with the one before it and keeps the time of the last frame that differed.
function armSettleProbe(page: Page, message: string): Promise<void> {
  return page.evaluate((message) => {
    const probe: SettleProbe = { statusAt: null, lastChangeAt: null, changes: 0 };
    (window as unknown as { settleProbe: SettleProbe }).settleProbe = probe;
    const status = document.getElementById("status")!;
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const ctx = canvas.getContext("2d")!;
    let last: Uint8ClampedArray | null = null;
    const sample = (): void => {
      const { data } = ctx.getImageData(0, 0, canvas.width, canvas.height);
      if (last && last.some((v, i) => v !== data[i])) {
        probe.lastChangeAt = performance.now();
        probe.changes++;
      }
      last = data;
      if (performance.now() - probe.statusAt! < 1200) requestAnimationFrame(sample);
    };
    new MutationObserver(() => {
      if (probe.statusAt !== null || status.textContent !== message) return;
      probe.statusAt = performance.now();
      requestAnimationFrame(sample);
    }).observe(status, { childList: true, characterData: true, subtree: true });
  }, message);
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
  await armSettleProbe(page, "Your turn!");
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
  expect(probe.statusAt).not.toBeNull();
  expect(probe.lastChangeAt).not.toBeNull();
  const took = probe.lastChangeAt! - probe.statusAt!;
  test.info().annotations.push({ type: "settle-animation-ms", description: took.toFixed(0) });
  expect(took).toBeLessThanOrEqual(SETTLE_BOUND_MS);
  expect(probe.changes).toBeGreaterThanOrEqual(2);
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
  expect(probe.statusAt).not.toBeNull();
  const took = probe.lastChangeAt === null ? 0 : probe.lastChangeAt - probe.statusAt!;
  test.info().annotations.push({ type: "settle-instant-ms", description: took.toFixed(0) });
  expect(took).toBeLessThanOrEqual(INSTANT_BOUND_MS);
  expect(probe.changes).toBeLessThanOrEqual(1);
});
