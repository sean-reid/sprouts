import { expect, test, type Page } from "@playwright/test";
import { clickAway, drawBetween, expectDotAt, openBoard, shot } from "./helpers";

interface Rects {
  toggle: { left: number; top: number; width: number };
  turn: { left: number; width: number };
}

function rectsOf(page: Page): Promise<Rects> {
  return page.evaluate(() => {
    const toggle = document.querySelector(".toggle")!.getBoundingClientRect();
    const turn = document.getElementById("turn")!.getBoundingClientRect();
    return {
      toggle: { left: toggle.left, top: toggle.top, width: toggle.width },
      turn: { left: turn.left, width: turn.width },
    };
  });
}

test("the first player toggle holds still while the turn indicator changes", async ({ page }) => {
  test.skip(
    (test.info().project.use.viewport?.width ?? 0) < 768,
    "phones do not show the indicator",
  );
  await openBoard(page);
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  const before = await rectsOf(page);

  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#turnText")).toHaveText("AI's turn");
  const thinking = await rectsOf(page);
  await shot(page, "turn-indicator-thinking");
  expect(thinking.toggle).toEqual(before.toggle);
  expect(thinking.turn).toEqual(before.turn);

  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  expect(await rectsOf(page)).toEqual(before);
});
