import { readFile } from "node:fs/promises";
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
  await expect(status).toHaveText(/could not draw a move/, { timeout: 40_000 });
  await expect(page.locator("#gameOverModal")).not.toHaveClass(/visible/);
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await page.locator("#undoBtn").click();
  await expect(status).toHaveText(/Move undone: your turn/, { timeout: 20_000 });
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
