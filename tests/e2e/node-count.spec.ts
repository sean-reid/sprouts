import { mkdir, writeFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import {
  clickAway,
  drawBetween,
  expectDotAt,
  expectNodeDrawnAt,
  interceptWorkerMessages,
  nodeOnScreen,
  openBoard,
  shot,
  startGame,
} from "./helpers";

// The AI still draws its candidates through the raster pipeline, which
// takes most of this on the runner's 1000 board; the search itself holds
// to 800 ms.
const REPLY_MS = 30_000;

async function expectNodesDrawn(page: Page, count: number): Promise<void> {
  for (let i = 0; i < count; i++) {
    await expectNodeDrawnAt(page, await nodeOnScreen(page, i, count));
  }
}

// Draws the first chord, places the node, and returns how long the AI's
// reply took to land on the status line.
async function replyTime(page: Page, count: number): Promise<number> {
  const mid = await drawBetween(page, 0, 1, count);
  await expectDotAt(page, mid);
  const started = Date.now();
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: REPLY_MS });
  const took = Date.now() - started;
  // The fixtures directory is what CI uploads, so the runner's time can be read.
  await mkdir("test-results/fixtures", { recursive: true });
  await writeFile(
    `test-results/fixtures/${test.info().project.name}-reply-${count}-nodes.txt`,
    `${took} ms\n`,
  );
  return took;
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(interceptWorkerMessages);
  await openBoard(page);
});

const FRESH = "Draw a line between two nodes";

test("nine nodes start a game and the AI answers", async ({ page }) => {
  await startGame(page, 9);
  await expect(page.locator("#status")).toHaveText(FRESH);
  await expectNodesDrawn(page, 9);
  await shot(page, "nodes-9");
  const took = await replyTime(page, 9);
  expect(took).toBeLessThanOrEqual(REPLY_MS);
  await shot(page, "nodes-9-replied");
});

test("twenty-four nodes fit the small board and the AI still answers", async ({ page }) => {
  test.skip(test.info().project.name !== "mobile", "the 375 wide board is the tight one");
  await startGame(page, 24);
  await expect(page.locator("#status")).toHaveText(FRESH);
  await expectNodesDrawn(page, 24);
  await shot(page, "nodes-24-mobile");
  const took = await replyTime(page, 24);
  expect(took).toBeLessThanOrEqual(REPLY_MS);
  await shot(page, "nodes-24-mobile-replied");
});

test("the count is held to its range and comes back after a reload", async ({ page }) => {
  await startGame(page, 9);
  await expect(page.locator("#status")).toHaveText(FRESH);
  await page.locator("#nodeCount").fill("40");
  await page.locator("#startBtn").click();
  await expect(page.locator("#nodeCount")).toHaveValue("24");
  await expect(page.locator("#status")).toHaveText(FRESH);
  await startGame(page, 5);
  await expect(page.locator("#status")).toHaveText(FRESH);
  await replyTime(page, 5);
  await page.reload();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 20_000 });
  await expect(page.locator("#nodeCount")).toHaveValue("5");
  await expectNodesDrawn(page, 5);
});

test("the node count input is a touch target", async ({ page }) => {
  test.skip(test.info().project.name !== "mobile", "sized for the 375 wide layout");
  const input = await page.locator("#nodeCount").boundingBox();
  const button = await page.locator("#startBtn").boundingBox();
  expect(input?.height).toBeGreaterThanOrEqual(44);
  expect(input?.width).toBeGreaterThanOrEqual(44);
  expect(button?.height).toBeGreaterThanOrEqual(44);
  await shot(page, "node-count-mobile");
});
