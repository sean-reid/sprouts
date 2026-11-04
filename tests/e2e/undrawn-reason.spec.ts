import { expect, test } from "@playwright/test";
import type { AiReply } from "../../pkg/sprouts";
import { answerNext, clickAway, drawBetween, expectDotAt, openBoard, shot } from "./helpers";

// The engine answers Undrawn with a reason. Out of time means the position
// may well be drawable, so the status says the AI ran out of time rather
// than that no move can be drawn, and Undo hands the move back.
test("an out-of-time reply says the AI ran out of time, and the human can undo", async ({
  page,
}) => {
  await openBoard(page);
  const mid = await drawBetween(page, 0, 1, 4);
  await expectDotAt(page, mid);
  const reply: AiReply = { kind: "undrawn", reason: "outOfTime" };
  await answerNext(page, "aiMove", reply);
  await clickAway(page);
  const status = page.locator("#status");
  await expect(status).toHaveText(
    "The AI ran out of time drawing its move. Undo or start a new game.",
  );
  await expect(status).toHaveClass("lose");
  await expect(page.locator("#gameOverModal")).not.toHaveClass(/visible/);
  await shot(page, "undrawn-out-of-time");
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await page.locator("#undoBtn").click();
  await expect(status).toHaveText(/Move undone: your turn/, { timeout: 20_000 });
});

test("an exhausted reply says no move can be drawn", async ({ page }) => {
  await openBoard(page);
  const mid = await drawBetween(page, 0, 1, 4);
  await expectDotAt(page, mid);
  const reply: AiReply = { kind: "undrawn", reason: "exhausted" };
  await answerNext(page, "aiMove", reply);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText(
    "The AI could not draw a move here. Undo or start a new game.",
  );
});
