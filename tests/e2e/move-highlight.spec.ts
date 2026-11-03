import { readFile } from "node:fs/promises";
import { expect, test, type Page } from "@playwright/test";
import { HIGHLIGHT_COLOR, PLAYER_COLORS } from "../../web/render";
import {
  awayFrom,
  colourPixelsNear,
  frameOf,
  nodeOnScreen,
  openBoard,
  playExchange,
  recordNodesOnScreen,
  ringSamplesAround,
  rowIds,
  savedRecord,
  shot,
} from "./helpers";

// The highlight ring sits at NODE_RADIUS + 6 in web/render.ts.
const RING_RADIUS = 14;
const HIGHLIGHT_MS = 2000;

const rows = (page: Page) => page.locator("#moveList li button");

function hasTouch(): boolean {
  return test.info().project.use.hasTouch === true;
}

async function expectRing(page: Page, at: Client): Promise<void> {
  await expect
    .poll(() => ringSamplesAround(page, at, RING_RADIUS, HIGHLIGHT_COLOR))
    .toBeGreaterThanOrEqual(8);
}

async function expectNoRing(page: Page, at: Client): Promise<void> {
  await expect
    .poll(() => ringSamplesAround(page, at, RING_RADIUS, HIGHLIGHT_COLOR))
    .toBeLessThanOrEqual(3);
}

type Client = { x: number; y: number };

test("hovering a row rings its nodes and outlines its line until the pointer leaves", async ({
  page,
}) => {
  test.skip(hasTouch(), "hover needs a mouse");
  await openBoard(page);
  await playExchange(page);
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  const quarter = { x: a.x + (b.x - a.x) / 4, y: a.y + (b.y - a.y) / 4 };
  const blue = PLAYER_COLORS.human;
  await expectNoRing(page, a);
  const plain = await colourPixelsNear(page, quarter, 6, blue);
  expect(plain).toBeGreaterThan(0);
  expect(await colourPixelsNear(page, quarter, 6, HIGHLIGHT_COLOR)).toBeLessThanOrEqual(2);

  await rows(page).nth(0).hover();
  await expectRing(page, a);
  await expectRing(page, b);
  await expect.poll(() => colourPixelsNear(page, quarter, 6, HIGHLIGHT_COLOR)).toBeGreaterThan(8);
  await expect.poll(() => colourPixelsNear(page, quarter, 6, blue)).toBeGreaterThan(0);
  await expect(rows(page).nth(0)).toHaveAttribute("title", "0 to 1, new node 4");
  await shot(page, "move-highlight");

  const away = awayFrom(await frameOf(page));
  await page.mouse.move(away.x, away.y);
  await expectNoRing(page, a);
  await expectNoRing(page, b);
  await expect
    .poll(() => colourPixelsNear(page, quarter, 6, HIGHLIGHT_COLOR))
    .toBeLessThanOrEqual(2);
});

test("a row reached with Tab highlights its move and Enter seeks to it", async ({ page }) => {
  test.skip(hasTouch(), "Tab needs a keyboard");
  await openBoard(page);
  await playExchange(page);
  const a = await nodeOnScreen(page, 0, 4);

  // Safari moves focus between buttons with Option held.
  const tab = test.info().project.name === "webkit" ? "Alt+Tab" : "Tab";
  await page.locator("#shareBtn").focus();
  await page.keyboard.press(tab);
  await expect(rows(page).nth(0)).toBeFocused();
  await expectRing(page, a);

  await page.keyboard.press(`Shift+${tab}`);
  await expect(page.locator("#shareBtn")).toBeFocused();
  await expectNoRing(page, a);

  await page.keyboard.press(tab);
  await expectRing(page, a);
  await page.keyboard.press("Enter");
  await expect(page.locator("#status")).toHaveText("Reviewing: move 1 of 2.");
  await expect(rows(page).nth(0)).toHaveAttribute("aria-current", "step");
  await expectRing(page, a);
});

test("tapping a row points out the move it lands on for two seconds", async ({ page }) => {
  test.skip(!hasTouch(), "the tap path is the touch one");
  await openBoard(page);
  await playExchange(page);
  await page.locator("#backBtn").tap();
  await expect(page.locator("#moveList [aria-current]")).toHaveCount(0);

  await rows(page).nth(1).tap();
  await expect(rows(page).nth(1)).toHaveAttribute("aria-current", "step");
  const [from, to] = await rowIds(page, 1);
  const nodes = await recordNodesOnScreen(page, await savedRecord(page));
  await expectRing(page, nodes[from]!);
  await expectRing(page, nodes[to]!);
  await shot(page, "move-highlight");

  await page.waitForTimeout(HIGHLIGHT_MS + 500);
  await expectNoRing(page, nodes[from]!);
  await expectNoRing(page, nodes[to]!);
});

test("rows describe the shape of each move, name the nodes in the title and wear the mover's colour", async ({
  page,
}) => {
  test.skip(test.info().project.name !== "desktop", "the text does not depend on the viewport");
  const record = await readFile("fixtures/saved/finished-4n-500.json", "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move 9 of 9\./, {
    timeout: 60_000,
  });
  await expect(rows(page).nth(0)).toHaveText("1. You, loop on one node");
  await expect(rows(page).nth(0)).toHaveAttribute("title", "0 to 0, new node 4");
  await expect(rows(page).nth(1)).toHaveText("2. AI, joins two nodes");
  await expect(rows(page).nth(1)).toHaveAttribute("title", "0 to 1, new node 5");
  await expect(rows(page).nth(8)).toHaveText("9. You, joins two nodes");
  await expect(rows(page).nth(8)).toHaveAttribute("title", "4 to 10, new node 12");
  await expect(rows(page).nth(0)).toHaveCSS("border-left-color", "rgb(96, 165, 250)");
  await expect(rows(page).nth(1)).toHaveCSS("border-left-color", "rgb(239, 68, 68)");
  await expect(rows(page).nth(8)).toHaveCSS("border-left-color", "rgb(96, 165, 250)");
});
