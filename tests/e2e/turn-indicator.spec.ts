import { readFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import {
  clickAway,
  drawBetween,
  expectDotAt,
  holdNext,
  openBoard,
  shot,
  startGame,
} from "./helpers";

interface Rects {
  text: string;
  toggle: { left: number; top: number; width: number };
  turn: { left: number; width: number };
}

function rectsOf(page: Page): Promise<Rects> {
  return page.evaluate(() => {
    const toggle = document.querySelector(".toggle")!.getBoundingClientRect();
    const turn = document.getElementById("turn")!.getBoundingClientRect();
    return {
      text: document.getElementById("turnText")!.textContent ?? "",
      toggle: { left: toggle.left, top: toggle.top, width: toggle.width },
      turn: { left: turn.left, width: turn.width },
    };
  });
}

// The width #turn needs for each text it shows in this engine's font: the
// text plus the swatch, the gap and the padding.
function neededWidths(page: Page): Promise<string> {
  return page.evaluate(() => {
    const text = document.getElementById("turnText")!;
    const was = text.textContent;
    const out: string[] = [];
    for (const label of ["Your turn", "AI's turn", "You won", "AI won"]) {
      text.textContent = label;
      out.push(`${label} ${(text.getBoundingClientRect().width + 12 + 8 + 24).toFixed(1)}`);
    }
    text.textContent = was;
    return out.join(", ");
  });
}

test.beforeEach(() => {
  test.skip(
    (test.info().project.use.viewport?.width ?? 0) < 768,
    "phones do not show the indicator",
  );
});

test("the first player toggle holds still while the turn indicator changes", async ({ page }) => {
  await openBoard(page);
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  const before = await rectsOf(page);
  test.info().annotations.push({ type: "turn-widths", description: await neededWidths(page) });

  // The reply is held so the thinking state stands still while it is read.
  await holdNext(page, "aiMove", 3000);
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#turnText")).toHaveText("AI's turn");
  const thinking = await rectsOf(page);
  expect(thinking.text).toBe("AI's turn");
  await shot(page, "turn-indicator-thinking");
  expect(thinking.toggle).toEqual(before.toggle);
  expect(thinking.turn).toEqual(before.turn);

  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  expect(await rectsOf(page)).toEqual(before);
});

test("the won texts leave the toggle where the idle board has it", async ({ page }) => {
  const record = await readFile("fixtures/saved/finished-4n-500.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(page.locator("#turnText")).toHaveText(/won$/, { timeout: 60_000 });
  const won = await rectsOf(page);
  await shot(page, "turn-indicator-won");

  await startGame(page, 4);
  await expect(page.locator("#turnText")).toHaveText("Your turn");
  const idle = await rectsOf(page);
  expect(won.toggle).toEqual(idle.toggle);
  expect(won.turn).toEqual(idle.turn);
});
