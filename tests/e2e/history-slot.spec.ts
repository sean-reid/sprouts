import { readFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import { awaitReply, openBoard, playExchange, shot, startGame } from "./helpers";

// The panel's floor plus the gap between the board and it.
const PANEL_FLOOR = 160;
const GAP = 8;
const ROW = 44;

interface Slot {
  play: { width: number; height: number };
  canvas: { left: number; top: number; width: number; height: number };
  panel: { height: number };
  list: { height: number; fullRows: number };
}

function slotOf(page: Page): Promise<Slot> {
  return page.evaluate(() => {
    const rect = (el: Element) => el.getBoundingClientRect();
    const play = rect(document.querySelector(".play-area")!);
    const canvas = rect(document.getElementById("canvas")!);
    const panel = rect(document.getElementById("history")!);
    const list = rect(document.getElementById("moveList")!);
    const rows = [...document.querySelectorAll("#moveList button")].map(rect);
    return {
      play: { width: play.width, height: play.height },
      canvas: { left: canvas.left, top: canvas.top, width: canvas.width, height: canvas.height },
      panel: { height: panel.height },
      list: {
        height: list.height,
        fullRows: rows.filter((r) => r.top >= list.top - 0.5 && r.bottom <= list.bottom + 0.5)
          .length,
      },
    };
  });
}

function expectBoardAndPanelToShareTheHeight(slot: Slot): void {
  const side = Math.min(slot.play.width, slot.play.height - PANEL_FLOOR - GAP);
  expect(slot.canvas.width).toBeCloseTo(side, 0);
  expect(slot.canvas.height).toBeCloseTo(side, 0);
  expect(slot.panel.height).toBeGreaterThanOrEqual(PANEL_FLOOR - 0.5);
  expect(slot.panel.height).toBeCloseTo(slot.play.height - side - GAP, 0);
}

test.beforeEach(() => {
  test.skip(
    (test.info().project.use.viewport?.width ?? 0) >= 1024,
    "the panel is a column beside the board at 1024 and up",
  );
});

test("the panel takes the height the square board leaves", async ({ page }) => {
  await openBoard(page);
  expectBoardAndPanelToShareTheHeight(await slotOf(page));
});

test("a tall phone shows more of the move list without the board moving mid game", async ({
  page,
}) => {
  test.skip(test.info().project.name !== "mobile", "a phone height");
  await page.setViewportSize({ width: 390, height: 844 });
  const record = await readFile("fixtures/saved/ten-moves-9n-1000.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await awaitReply(page);
  const slot = await slotOf(page);
  expectBoardAndPanelToShareTheHeight(slot);
  // 844 tall leaves more than the width, so the board is width bound and
  // the list holds three full rows under it.
  expect(slot.canvas.width).toBeCloseTo(slot.play.width, 0);
  expect(slot.list.height).toBeGreaterThanOrEqual(3 * ROW + 2 * GAP);
  expect(slot.list.fullRows).toBeGreaterThanOrEqual(3);
  await expect(page.locator("#moveList [aria-current]")).toBeInViewport();
  await shot(page, "history-slot-tall");

  await startGame(page, 4);
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  const idle = await slotOf(page);
  expect(idle.canvas).toEqual(slot.canvas);
  await playExchange(page);
  expect((await slotOf(page)).canvas).toEqual(slot.canvas);
});
