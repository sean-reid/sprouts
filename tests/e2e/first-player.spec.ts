import { expect, test, type Page } from "@playwright/test";
import {
  clickAway,
  drawBetween,
  expectDotAt,
  inkPixels,
  openBoard,
  PLACING_HINT,
  strokeCells,
} from "./helpers";

const pressed = (page: Page, player: string) =>
  expect(page.locator(`button[data-first="${player}"]`)).toHaveAttribute("aria-pressed", "true");

test("the first-player toggle restarts an empty board and waits on one in play", async ({
  page,
}) => {
  test.skip(test.info().project.name !== "desktop", "one project covers the toggle");
  await openBoard(page);
  const empty = await inkPixels(page);

  // The selected button again changes nothing, so the status never moves.
  const quiet = page.evaluate(
    () =>
      new Promise<number>((resolve) => {
        let changes = 0;
        const observer = new MutationObserver(() => changes++);
        observer.observe(document.getElementById("status")!, { childList: true, subtree: true });
        setTimeout(() => {
          observer.disconnect();
          resolve(changes);
        }, 500);
      }),
  );
  await page.locator('button[data-first="human"]').click();
  expect(await quiet).toBe(0);
  await pressed(page, "human");

  await page.locator('button[data-first="ai"]').click();
  await expect(page.locator("#status")).toHaveText("AI is thinking", { timeout: 20_000 });
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  await expect(page.locator("#moveList li")).toHaveText([/^1\. AI \d+ to \d+$/]);
  await expect.poll(() => inkPixels(page)).toBeGreaterThan(empty);

  const opened = await strokeCells(page);
  await page.locator('button[data-first="human"]').click();
  await expect(page.locator("#status")).toHaveText("You start next game");
  await pressed(page, "human");
  expect(await strokeCells(page)).toEqual(opened);
  await expect(page.locator("#moveList li")).toHaveCount(1);

  await page.locator('button[data-nodes="4"]').click();
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  await expect.poll(() => inkPixels(page)).toBe(empty);

  // Mid-stroke the choice is kept but the placing prompt stays on screen.
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await page.locator('button[data-first="ai"]').click();
  await pressed(page, "ai");
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });

  const played = await strokeCells(page);
  await page.locator('button[data-first="human"]').click();
  await expect(page.locator("#status")).toHaveText("You start next game");
  await page.locator('button[data-first="ai"]').click();
  await expect(page.locator("#status")).toHaveText("AI starts next game");
  await pressed(page, "ai");
  expect(await strokeCells(page)).toEqual(played);
  await expect(page.locator("#moveList li")).toHaveCount(2);

  await page.locator('button[data-nodes="4"]').click();
  await expect(page.locator("#status")).toHaveText("AI is thinking", { timeout: 20_000 });
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#moveList li")).toHaveText([/^1\. AI \d+ to \d+$/]);
});
