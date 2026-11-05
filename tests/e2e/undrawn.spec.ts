import { mkdir, readFile, writeFile } from "node:fs/promises";
import { expect, test } from "@playwright/test";
import {
  clickAway,
  drawBetween,
  expectDotAt,
  holdNext,
  interceptWorkerMessages,
  openBoard,
} from "./helpers";

// The engine client gives up on a worker call after 30 s. Holding the
// AI's move back past that leaves the board as an undrawn position does:
// the status says so and Undo hands the move back.
test("a reply the worker never sends in time is undrawn, and the human can undo", async ({
  page,
}) => {
  await page.addInitScript(interceptWorkerMessages);
  await openBoard(page);
  const mid = await drawBetween(page, 0, 1, 4);
  await expectDotAt(page, mid);
  await holdNext(page, "aiMove", 31_000);
  await clickAway(page);
  const status = page.locator("#status");
  await expect(status).toHaveText(/ran out of time/, { timeout: 40_000 });
  await expect(page.locator("#gameOverModal")).not.toHaveClass(/visible/);
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await page.locator("#undoBtn").click();
  await expect(status).toHaveText(/Move undone: your turn/, { timeout: 20_000 });
});

// A saved game from the live site where the AI answered Undrawn on a board
// with room to spare, with the AI to move. Loading it has the AI draw, and
// the console carries one line per reply with the engine's answer.
test("a 24-node position with room draws, and the console says so", async ({ page }) => {
  const record = await readFile("fixtures/undrawn/room-24n-1000.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  const lines: string[] = [];
  page.on("console", (message) => {
    if (message.text().startsWith("aiMove:")) lines.push(message.text());
  });
  const started = Date.now();
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  const took = Date.now() - started;
  await mkdir("test-results/fixtures", { recursive: true });
  await writeFile(
    `test-results/fixtures/${test.info().project.name}-reply-room-24-nodes.txt`,
    `${took} ms\n`,
  );
  expect(lines).toHaveLength(1);
  expect(lines[0]).toMatch(/^aiMove: move in \d+ ms$/);
  await expect(page.locator("#moveList li")).toHaveCount(4);
});

// A saved 4-node game from the live site where the AI answered Undrawn on
// its one listed move, a chord out of a corner between two of its own
// spokes. Loading it has the AI draw that chord, the last move of the game.
test("a 4-node position with one chord out of a narrow corner draws", async ({ page }) => {
  const record = await readFile("fixtures/undrawn/pocket-4n-1000.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  const lines: string[] = [];
  page.on("console", (message) => {
    if (message.text().startsWith("aiMove:")) lines.push(message.text());
  });
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText("AI wins", { timeout: 60_000 });
  expect(lines).toHaveLength(1);
  expect(lines[0]).toMatch(/^aiMove: move in \d+ ms$/);
  await expect(page.locator("#moveList li")).toHaveCount(10);
});

// A saved game where the only live node is sealed inside a loop too small
// for any line the AI draws. Loading it hands the AI the move: it must say
// so rather than end the game, and Undo must give the board back to the
// human.
test("an undrawn position is an error state the human can undo, not game over", async ({
  page,
}) => {
  const record = await readFile("fixtures/undrawn/sealed-2n-1000.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  const status = page.locator("#status");
  await expect(status).toHaveText(/could not draw a move/, { timeout: 60_000 });
  await expect(page.locator("#gameOverModal")).not.toHaveClass(/visible/);
  await page.locator("#undoBtn").click();
  await expect(status).toHaveText(/Move undone: your turn/, { timeout: 20_000 });
  await expect(page.locator("#gameOverModal")).not.toHaveClass(/visible/);
});
