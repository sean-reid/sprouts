import { readFile } from "node:fs/promises";
import { expect, test } from "@playwright/test";

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
