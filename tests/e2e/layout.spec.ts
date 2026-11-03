import { readFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import type { GameRecord } from "../../web/history";
import type { Player } from "../../web/state";
import { frameOf, near, openBoard, pixelAt, playExchange, ringPixel, shot } from "./helpers";

const COLOURS: Record<Player, string> = { human: "#60A5FA", ai: "#EF4444" };

interface Rect {
  left: number;
  top: number;
  right: number;
  bottom: number;
}

interface Layout {
  canvas: Rect;
  panel: Rect;
  controls: Rect;
  tops: number[];
}

function other(player: Player): Player {
  return player === "human" ? "ai" : "human";
}

function layoutOf(page: Page): Promise<Layout> {
  return page.evaluate(() => {
    const rect = (el: Element): Rect => {
      const { left, top, right, bottom } = el.getBoundingClientRect();
      return { left, top, right, bottom };
    };
    const rows = [...document.querySelectorAll(".controls")].map(rect);
    const items = ["#undoBtn", ".toggle", "#turn"].map((s) => rect(document.querySelector(s)!));
    return {
      canvas: rect(document.getElementById("canvas")!),
      panel: rect(document.getElementById("history")!),
      controls: {
        left: Math.min(...rows.map((r) => r.left)),
        top: Math.min(...rows.map((r) => r.top)),
        right: Math.max(...rows.map((r) => r.right)),
        bottom: Math.max(...rows.map((r) => r.bottom)),
      },
      tops: [...rows, ...items].map((r) => r.top),
    };
  });
}

async function expectSideBySide(page: Page, label: string): Promise<void> {
  const before = await layoutOf(page);
  const gap = before.panel.left - before.canvas.right;
  expect(gap).toBeGreaterThanOrEqual(16);
  expect(gap).toBeLessThanOrEqual(24);
  const groupMid = (before.canvas.left + before.panel.right) / 2;
  const controlsMid = (before.controls.left + before.controls.right) / 2;
  expect(Math.abs(groupMid - controlsMid)).toBeLessThanOrEqual(4);
  for (const top of before.tops) expect(Math.abs(top - before.tops[0]!)).toBeLessThan(1);
  expect(before.canvas.right - before.canvas.left).toBeCloseTo(
    before.canvas.bottom - before.canvas.top,
    0,
  );
  await shot(page, `layout-idle-${label}`);

  await playExchange(page);
  await expect(page.locator("#history")).toBeVisible();
  const after = await layoutOf(page);
  expect(after.canvas).toEqual(before.canvas);
  await shot(page, `layout-played-${label}`);
}

async function expectIndicator(page: Page, text: string, player: Player): Promise<void> {
  await expect(page.locator("#turnText")).toHaveText(text);
  const want = [1, 3, 5].map((i) => parseInt(COLOURS[player].slice(i, i + 2), 16));
  await expect(page.locator("#turn .swatch")).toHaveCSS(
    "background-color",
    `rgb(${want.join(", ")})`,
  );
  await expect
    .poll(async () => near(await pixelAt(page, ringPixel(await frameOf(page))), COLOURS[player]))
    .toBe(true);
}

test("the board and move list sit centred under one row of controls", async ({ page }) => {
  test.skip(test.info().project.name !== "desktop", "the side-by-side layout starts at 1024 wide");
  await openBoard(page);
  await expectSideBySide(page, "1024");

  await page.setViewportSize({ width: 1600, height: 900 });
  await page.locator("button[data-nodes='4']").click();
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  await expectSideBySide(page, "1600");
});

test("a finished game names the winner on the indicator and the ring", async ({ page }) => {
  const record = await readFile("fixtures/saved/finished-4n-500.json", "utf8");
  const { moves, firstPlayer } = JSON.parse(record) as GameRecord;
  const total = moves.length;
  const won = total % 2 === 1 ? firstPlayer : other(firstPlayer);
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText(`Reviewing: move ${total} of ${total}.`, {
    timeout: 60_000,
  });
  await expectIndicator(page, won === "human" ? "You won" : "AI won", won);

  await shot(page, "layout-finished");
  if (test.info().project.name === "desktop") {
    await page.setViewportSize({ width: 1600, height: 900 });
    await expectIndicator(page, won === "human" ? "You won" : "AI won", won);
    await shot(page, "layout-finished-1600");
  }

  // Two moves back puts the loser on move with the game still open.
  await page.locator("#backBtn").click();
  await page.locator("#backBtn").click();
  await expect(page.locator("#status")).toHaveText(
    new RegExp(`^Reviewing: move ${total - 2} of ${total}\\.`),
  );
  const toMove = other(won);
  await expectIndicator(page, toMove === "human" ? "Your turn" : "AI's turn", toMove);
  await page.locator("#forwardBtn").click();
  await page.locator("#forwardBtn").click();
  await expect(page.locator("#status")).toHaveText(`Reviewing: move ${total} of ${total}.`);
  await expectIndicator(page, won === "human" ? "You won" : "AI won", won);

  const frame = await frameOf(page);
  await page.mouse.click(frame.left + frame.width / 2, frame.top + frame.height / 2);
  await expect(page.locator("#gameOverModal")).toBeVisible();
  await page.locator("#newGameBtn").click();
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  await expectIndicator(page, "Your turn", "human");
  await shot(page, "layout-idle");
});
