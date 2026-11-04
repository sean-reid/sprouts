import { mkdir, readFile, writeFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import {
  boardSettled,
  clickAway,
  drawBetween,
  expectDotAt,
  expectNodeDrawnAt,
  interceptWorkerMessages,
  nodeOnScreen,
  openBoard,
  playExchange,
  share,
  shot,
  startGame,
  strokeCells,
} from "./helpers";

// One 800 ms search plus up to 800 ms of routing. The runner measured
// 1.8 s at 5 and 24 nodes and 2.8 s at 9 nodes; the bound is twice that.
const REPLY_MS = 6_000;

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

// The button's text and its count span, when the span is shown, share one
// line, and nothing spills past the button's box.
async function expectOneLine(page: Page, selector: string): Promise<void> {
  const fit = await page.locator(selector).evaluate((el) => {
    const tops = new Set<number>();
    for (const node of el.childNodes) {
      const range = document.createRange();
      range.selectNodeContents(node);
      const shown =
        !(node instanceof HTMLElement) || getComputedStyle(node).position !== "absolute";
      for (const r of range.getClientRects()) if (shown && r.width > 0) tops.add(Math.round(r.top));
    }
    return { lines: tops.size, spill: el.scrollWidth - el.clientWidth };
  });
  expect(fit.lines).toBe(1);
  expect(fit.spill).toBeLessThanOrEqual(0);
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

test("plus on an untouched board re-lays it at once and arrows step the field", async ({
  page,
}) => {
  const more = page.locator("#moreBtn");
  await more.click();
  await more.click();
  await more.click();
  await expect(page.locator("#nodeCount")).toHaveValue("7");
  await expect(page.locator("#status")).toHaveText(`7 nodes. ${FRESH}`);
  await expect(page.locator("#startBtn")).toHaveText("New game");
  await expectNodesDrawn(page, 7);
  await shot(page, "stepper-7");

  await page.locator("#nodeCount").focus();
  await page.keyboard.press("ArrowDown");
  await expect(page.locator("#nodeCount")).toHaveValue("6");
  await expect(page.locator("#status")).toHaveText(`6 nodes. ${FRESH}`);
  await expectNodesDrawn(page, 6);
  await page.locator("#fewerBtn").click();
  await expect(page.locator("#status")).toHaveText(`5 nodes. ${FRESH}`);
  await expectNodesDrawn(page, 5);
});

test("once a move is played the count waits on the start button", async ({ page }) => {
  await playExchange(page);
  await boardSettled(page);
  const before = await strokeCells(page);
  const canvas = await page.locator("#canvas").boundingBox();
  await page.locator("#moreBtn").click();
  await expect(page.locator("#nodeCount")).toHaveValue("5");
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(page.locator("#countNote")).toHaveText("Next game: 5 nodes");
  await expect(page.locator("#startBtn")).toHaveText("New game, 5 nodes");
  await expect(page.locator("#startBtn")).toHaveAttribute("aria-label", "New game, 5 nodes");
  await expect(page.locator("#moveList li")).toHaveCount(2);
  expect(await strokeCells(page)).toEqual(before);
  expect(await page.locator("#canvas").boundingBox()).toEqual(canvas);
  await expectOneLine(page, "#startBtn");
  await shot(page, "stepper-pending");

  await page.locator("#fewerBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(page.locator("#countNote")).toHaveText("Next game: 4 nodes");
  await expect(page.locator("#startBtn")).toHaveText("New game");
  await expect(page.locator("#moveList li")).toHaveCount(2);

  await page.locator("#moreBtn").click();
  await page.locator("#startBtn").click();
  await expect(page.locator("#status")).toHaveText(FRESH);
  await expect(page.locator("#startBtn")).toHaveText("New game");
  await expect(page.locator("#countNote")).toHaveText("");
  await expect(page.locator("#moveList li")).toHaveCount(0);
  await expectNodesDrawn(page, 5);
  expect(await page.locator("#canvas").boundingBox()).toEqual(canvas);
});

test("the stepper stops at 2 and 24 and hands focus to the field there", async ({ page }) => {
  await page.locator("#nodeCount").fill("2");
  await page.locator("#nodeCount").blur();
  await expect(page.locator("#fewerBtn")).toBeDisabled();
  await expect(page.locator("#moreBtn")).toBeEnabled();
  await expect(page.locator("#status")).toHaveText(`2 nodes. ${FRESH}`);
  await expectNodesDrawn(page, 2);
  await page.locator("#nodeCount").fill("23");
  await page.locator("#moreBtn").focus();
  await expect(page.locator("#status")).toHaveText(`23 nodes. ${FRESH}`);
  await page.keyboard.press("Enter");
  await expect(page.locator("#nodeCount")).toHaveValue("24");
  await expect(page.locator("#moreBtn")).toBeDisabled();
  await expect(page.locator("#nodeCount")).toBeFocused();
  await page.locator("#fewerBtn").click();
  await expect(page.locator("#nodeCount")).toHaveValue("23");
  await expect(page.locator("#moreBtn")).toBeEnabled();
});

test("a typed count waits for Enter or for the field to settle", async ({ page }) => {
  const field = page.locator("#nodeCount");
  await field.selectText();
  await field.pressSequentially("24");
  await expect(field).toHaveValue("24");
  await expect(page.locator("#status")).toHaveText(FRESH);
  await expectNodesDrawn(page, 4);
  await field.press("Enter");
  await expect(page.locator("#status")).toHaveText(FRESH);
  await expectNodesDrawn(page, 24);
  await field.selectText();
  await field.pressSequentially("9");
  await field.blur();
  await expect(page.locator("#status")).toHaveText(`9 nodes. ${FRESH}`);
  await expectNodesDrawn(page, 9);
  await field.fill("");
  await field.blur();
  await expect(field).toHaveValue("4");
  await page.locator("#moreBtn").click();
  await expect(field).toHaveValue("5");
});

test("the board after a failed load keeps the saved game through a step", async ({ page }) => {
  await startGame(page, 5);
  await expect(page.locator("#status")).toHaveText(FRESH);
  await replyTime(page, 5);
  const saved = await page.evaluate(() => localStorage.getItem("sprouts.game"));
  await page.addInitScript(() => {
    (window as unknown as { failNext: (m: string, e: string) => void }).failNext(
      "init",
      "no engine",
    );
  });
  await page.reload();
  await expect(page.locator("#status")).toHaveText(/^Could not load the saved game/, {
    timeout: 20_000,
  });
  expect(saved).not.toBeNull();
  await page.locator("#moreBtn").click();
  await expect(page.locator("#status")).toHaveText(`5 nodes. ${FRESH}`);
  expect(await page.evaluate(() => localStorage.getItem("sprouts.game"))).toBe(saved);

  // A move on that board saves the new game. Undo keeps the moves for
  // Forward, so a step there waits on New game, which clears the save.
  await playExchange(page, 5);
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  await page.locator("#moreBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  await expect(page.locator("#startBtn")).toHaveText("New game, 6 nodes");
  await page.locator("#startBtn").click();
  await expect(page.locator("#status")).toHaveText(FRESH);
  await expectNodesDrawn(page, 6);
  expect(await page.evaluate(() => localStorage.getItem("sprouts.game"))).toBeNull();
});

test("a board that failed to load from a link keeps the link through a step", async ({ page }) => {
  await playExchange(page);
  const { url } = await share(page, "stepper-link-payload-chars");
  const hash = new URL(url).hash;
  await page.addInitScript(() => {
    (window as unknown as { failNext: (m: string, e: string) => void }).failNext(
      "init",
      "no engine",
    );
  });
  // A visitor has no saved game, and a hash-only change would not reload.
  await page.evaluate(() => localStorage.clear());
  await page.goto("about:blank");
  await page.goto(url);
  await expect(page.locator("#status")).toHaveText(/^Could not load the saved game/, {
    timeout: 20_000,
  });
  await page.locator("#moreBtn").click();
  await expect(page.locator("#status")).toHaveText(`5 nodes. ${FRESH}`);
  expect(await page.evaluate(() => location.hash)).toBe(hash);
});

test("a review keeps its status and the count waits on the start button", async ({ page }) => {
  const record = await readFile("fixtures/saved/finished-4n-500.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move 9 of 9\./, {
    timeout: 60_000,
  });
  await page.locator("#moreBtn").click();
  await expect(page.locator("#nodeCount")).toHaveValue("5");
  await expect(page.locator("#startBtn")).toHaveText("New game, 5 nodes");
  await expect(page.locator("#countNote")).toHaveText("Next game: 5 nodes");
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move 9 of 9\./);
  await expect(page.locator("#moveList li")).toHaveCount(9);
});

test("the stepper and the start button are touch targets", async ({ page }) => {
  test.skip(test.info().project.name !== "mobile", "sized for the 375 wide layout");
  for (const id of ["#fewerBtn", "#nodeCount", "#moreBtn", "#startBtn"]) {
    const box = await page.locator(id).boundingBox();
    expect(box?.height, id).toBeGreaterThanOrEqual(44);
    expect(box?.width, id).toBeGreaterThanOrEqual(44);
  }
  await shot(page, "node-count-mobile");
});
