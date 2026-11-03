import { expect, test, type Page } from "@playwright/test";
import {
  armFlashProbe,
  clickAway,
  drawBetween,
  expectDotAt,
  frameOf,
  holdNext,
  interceptWorkerMessages,
  near,
  nodeOnScreen,
  openBoard,
  pixelAt,
  PLACING_HINT,
  playExchange,
  readFlashProbe,
  ringPixel,
  shot,
  type Client,
  type Rgb,
} from "./helpers";

const HUMAN = "#60A5FA";
const AI = "#EF4444";

const lerp = (a: Client, b: Client, t: number): Client => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});

// Commits the locked stroke with Enter and waits for the rejection, checking
// the flash lands within 100 ms of the message.
async function rejectWith(page: Page, message: string, flashAt: Client): Promise<void> {
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await armFlashProbe(page, flashAt, message, AI);
  await page.keyboard.press("Enter");
  await expect(page.locator("#status")).toHaveText(message);
  await expect.poll(async () => (await readFlashProbe(page)).flashAt).not.toBeNull();
  const probe = await readFlashProbe(page);
  expect(probe.flashAt! - probe.statusAt!).toBeLessThan(100);
}

// Three ring samples 200 ms apart while the AI is held mid-think.
async function ringWhileThinking(page: Page): Promise<Rgb[]> {
  await holdNext(page, "aiMove", 1500);
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("AI is thinking");
  const at = ringPixel(await frameOf(page));
  const samples: Rgb[] = [];
  for (let i = 0; i < 3; i++) {
    samples.push(await pixelAt(page, at));
    await page.waitForTimeout(200);
  }
  return samples;
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(interceptWorkerMessages);
});

test("a stroke across an existing line flashes the crossed segment", async ({ page }) => {
  await openBoard(page);
  await playExchange(page);

  // Node 3 to node 2 by way of the corner past the first chord, so the
  // stroke crosses it a third of the way along from node 0.
  const n0 = await nodeOnScreen(page, 0, 4);
  const n1 = await nodeOnScreen(page, 1, 4);
  const n2 = await nodeOnScreen(page, 2, 4);
  const n3 = await nodeOnScreen(page, 3, 4);
  const corner = { x: n0.x, y: n1.y };
  await page.mouse.move(n3.x, n3.y);
  await page.mouse.down();
  await page.mouse.move(corner.x, corner.y, { steps: 20 });
  await page.mouse.move(n2.x, n2.y, { steps: 20 });
  await page.mouse.up();

  const crossing = lerp(n0, n1, 1 / 3);
  await rejectWith(page, "Path crosses an existing line", crossing);
  await shot(page, "flash");
  await page.waitForTimeout(800);
  expect(near(await pixelAt(page, crossing), AI)).toBe(false);
});

test("a stroke brushing a node flashes that node", async ({ page }) => {
  await openBoard(page);
  const n0 = await nodeOnScreen(page, 0, 4);
  const n1 = await nodeOnScreen(page, 1, 4);
  const n2 = await nodeOnScreen(page, 2, 4);
  await page.mouse.move(n0.x, n0.y);
  await page.mouse.down();
  await page.mouse.move(n1.x, n1.y, { steps: 20 });
  await page.mouse.move(n2.x, n2.y, { steps: 20 });
  await page.mouse.up();

  // The flash ring sits 14 px out from the node centre.
  const onRing = { x: n1.x + 14, y: n1.y };
  await rejectWith(page, "Path passes too close to an existing node", onRing);
  await shot(page, "flash-node");
  await page.waitForTimeout(800);
  expect(near(await pixelAt(page, onRing), AI)).toBe(false);
});

test("the turn indicator and ring follow the exchange", async ({ page }) => {
  await openBoard(page);
  const ring = ringPixel(await frameOf(page));
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  expect(near(await pixelAt(page, ring), HUMAN)).toBe(true);
  await shot(page, "ring");

  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#turnText")).toHaveText("AI's turn");
  await expect(page.locator("#status")).toHaveText("AI is thinking");
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  expect(near(await pixelAt(page, ring), HUMAN)).toBe(true);
});

test("AI starts opens with the AI's line and no human input", async ({ page }) => {
  await openBoard(page);
  const toggle = page.locator("button[data-first]");
  for (const button of await toggle.all()) {
    const box = await button.boundingBox();
    expect(box?.width ?? 0).toBeGreaterThanOrEqual(44);
    expect(box?.height ?? 0).toBeGreaterThanOrEqual(44);
  }
  await page.locator('button[data-first="ai"]').click();
  await expect(page.locator('button[data-first="ai"]')).toHaveAttribute("aria-pressed", "true");
  await expect(page.locator('button[data-first="human"]')).toHaveAttribute("aria-pressed", "false");
  await shot(page, "toggle");

  await page.locator('button[data-nodes="4"]').click();
  await expect(page.locator("#status")).toHaveText("AI is thinking");
  await expect(page.locator("#turnText")).toHaveText("AI's turn");
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await shot(page, "ai-starts");

  // Undo takes back the AI's only move, which leaves the AI to play again.
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("AI is thinking");
  await expect(page.locator("#turnText")).toHaveText("AI's turn");
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#undoBtn")).toBeEnabled();
});

test("the ring pulses while the AI thinks", async ({ page }) => {
  await openBoard(page);
  const samples = await ringWhileThinking(page);
  expect(new Set(samples.map(String)).size).toBeGreaterThan(1);
});

test("reduced motion keeps the ring solid while the AI thinks", async ({ page }) => {
  await page.emulateMedia({ reducedMotion: "reduce" });
  await openBoard(page);
  const samples = await ringWhileThinking(page);
  expect(new Set(samples.map(String)).size).toBe(1);
  expect(near(samples[0]!, AI)).toBe(true);
  await shot(page, "ring-thinking-reduced");
});
