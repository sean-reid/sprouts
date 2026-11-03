import { mkdir, readFile, writeFile } from "node:fs/promises";
import {
  expect as baseExpect,
  test,
  type BrowserContextOptions,
  type Page,
} from "@playwright/test";
import type { Fixture } from "../../web/record";

// Every step here replays moves through the engine, which on the 1000
// board takes several seconds per apply.
const expect = baseExpect.configure({ timeout: 20_000 });
import { frameOf, nodeOnScreen, openBoard, playExchange, shot, strokeCells } from "./helpers";

const entries = (page: Page) => page.locator("#moveList li");

// A second visitor with the same screen but no saved game of their own.
function visitorOptions(): BrowserContextOptions {
  const { viewport, hasTouch, isMobile, deviceScaleFactor } = test.info().project.use;
  const options: BrowserContextOptions = {};
  if (viewport) options.viewport = viewport;
  if (hasTouch !== undefined) options.hasTouch = hasTouch;
  if (isMobile !== undefined) options.isMobile = isMobile;
  if (deviceScaleFactor !== undefined) options.deviceScaleFactor = deviceScaleFactor;
  return options;
}
const current = (page: Page) => page.locator("#moveList [aria-current]");

// A stroke that leaves the board on its way between two nodes: the engine
// accepts it, so a saved game with one must load again.
async function playDetourExchange(page: Page): Promise<void> {
  const frame = await frameOf(page);
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  const via = { x: frame.left - 6, y: b.y + (b.y - a.y) / 2 };
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(via.x, via.y, { steps: 15 });
  await page.mouse.move(b.x, b.y, { steps: 15 });
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(/place a new node/);
  await page.waitForTimeout(350);
  await page.mouse.click(a.x + (via.x - a.x) * 0.4, a.y + (via.y - a.y) * 0.4);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
}

test("a reload restores the game and New Game clears it", async ({ page }) => {
  await openBoard(page);
  await expect(page.locator("#history")).toBeHidden();
  await playDetourExchange(page);
  await expect(entries(page)).toHaveText(["1. You 0 to 1", /^2\. AI \d+ to \d+$/]);
  await expect(current(page)).toHaveText(/^2\. AI/);

  const saved = await page.evaluate(() => localStorage.getItem("sprouts.game"));
  const fixture = JSON.parse(saved ?? "null") as Fixture & { cursor: number };
  expect(fixture.version).toBe(1);
  expect(fixture.moves).toHaveLength(2);
  expect(fixture.cursor).toBe(2);
  expect(fixture.moves[0]).toMatchObject({ from: 0, to: 1 });
  await mkdir("test-results/fixtures", { recursive: true });
  await writeFile(`test-results/fixtures/${test.info().project.name}-saved.json`, saved ?? "");

  await page.reload();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 20_000 });
  await expect(entries(page)).toHaveCount(2);
  await expect(current(page)).toHaveText(/^2\. AI/);
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await shot(page, "history-restored");

  await page.locator("button[data-nodes='4']").click();
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  expect(await page.evaluate(() => localStorage.getItem("sprouts.game"))).toBeNull();
  await page.reload();
  await openBoard(page);
  await expect(page.locator("#history")).toBeHidden();
  await expect(page.locator("#undoBtn")).toBeDisabled();
});

test("jumping to a board where the AI is to move enters review and keeps the tail", async ({
  page,
}) => {
  await openBoard(page);
  await playExchange(page);
  await entries(page).nth(0).locator("button").click();
  await expect(page.locator("#status")).toHaveText("Reviewing: move 1 of 2.");
  await expect(entries(page)).toHaveCount(2);
  await expect(current(page)).toHaveText("1. You 0 to 1");

  await page.locator("#forwardBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(current(page)).toHaveText(/^2\. AI/);
  await page.locator("#backBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(current(page)).toHaveCount(0);
  await page.locator("#forwardBtn").click();
  await expect(current(page)).toHaveText(/^2\. AI/);
  await entries(page).nth(0).locator("button").click();
  await expect(page.locator("#status")).toHaveText("Reviewing: move 1 of 2.");

  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  await expect(page.locator("#undoBtn")).toBeDisabled();
  await expect(entries(page)).toHaveCount(2);
  await expect(current(page)).toHaveCount(0);

  await page.locator("#forwardBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(current(page)).toHaveText(/^2\. AI/);
});

test("a share link reopens the game in review and steps through it", async ({ page, browser }) => {
  await openBoard(page);
  await playExchange(page);
  const before = await strokeCells(page);
  await page.locator("#shareBtn").click();
  await expect(page.locator("#status")).toHaveText(/Link copied|Copy the link/);
  const url = page.url();
  expect(url).toContain("#game=");
  test.info().annotations.push({ type: "share-url-bytes", description: String(url.length) });
  await shot(page, "history-shared-source");

  const visitor = await browser.newContext(visitorOptions());
  const other = await visitor.newPage();
  await other.goto(url);
  await expect(other.locator("#status")).toHaveText(/^Reviewing: move 2 of 2/, {
    timeout: 20_000,
  });
  await expect(entries(other)).toHaveText(await entries(page).allTextContents());
  await expect(current(other)).toHaveText(/^2\. AI/);
  await expect(other.locator("#forwardBtn")).toBeDisabled();
  await expect(other.locator("#backBtn")).toBeEnabled();
  await shot(other, "history-shared-visitor");

  const after = await strokeCells(other);
  expect(after.length).toBe(before.length);
  const differing = before.filter((cell, i) => cell !== after[i]).length;
  const drawn = before.filter(Boolean).length;
  expect(drawn).toBeGreaterThan(0);
  expect(differing).toBeLessThanOrEqual(Math.ceil(drawn * 0.1));

  await other.locator("#backBtn").click();
  await expect(current(other)).toHaveText("1. You 0 to 1");
  await expect(other.locator("#forwardBtn")).toBeEnabled();
  await other.locator("#backBtn").click();
  await expect(other.locator("#backBtn")).toBeDisabled();
  await expect(current(other)).toHaveCount(0);
  await expect(other.locator("#status")).toHaveText(/^Reviewing: start/);
  await shot(other, "history-review-start");

  await other.keyboard.press("ArrowRight");
  await expect(current(other)).toHaveText("1. You 0 to 1");
  await entries(other).nth(1).locator("button").click();
  await expect(current(other)).toHaveText(/^2\. AI/);
  await expect(other.locator("#forwardBtn")).toBeDisabled();

  await other.locator("#undoBtn").click();
  await expect(other.locator("#status")).toHaveText("Your turn!");
  await expect(entries(other)).toHaveCount(2);
  expect(other.url()).not.toContain("#");

  await page.locator("#backBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(page.locator("#backBtn")).toBeDisabled();
  expect(page.url()).not.toContain("#");

  await other.locator("button[data-nodes='4']").click();
  await expect(other.locator("#status")).toHaveText("Draw a line between two nodes");
  expect(other.url()).not.toContain("#");
  await visitor.close();
});

test("a finished game reopens in review and steps single moves", async ({ page }) => {
  const record = await readFile("fixtures/saved/finished-4n-500.json", "utf8");
  const total = (JSON.parse(record) as Fixture).moves.length;
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText(`Reviewing: move ${total} of ${total}.`, {
    timeout: 60_000,
  });
  await expect(entries(page)).toHaveCount(total);
  await shot(page, "history-finished");

  const frame = await frameOf(page);
  await page.mouse.click(frame.left + frame.width / 2, frame.top + frame.height / 2);
  await expect(page.locator("#gameOverModal")).toBeVisible();
  await page.locator("#reviewBtn").click();
  await expect(page.locator("#gameOverModal")).toBeHidden();

  await page.locator("#backBtn").click();
  await expect(current(page)).toHaveText(new RegExp(`^${total - 1}\\.`));
  await page.locator("#backBtn").click();
  await expect(current(page)).toHaveText(new RegExp(`^${total - 2}\\.`));
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move \d+ of \d+\./);
  await page.locator("#forwardBtn").click();
  await expect(current(page)).toHaveText(new RegExp(`^${total - 1}\\.`));
  await page.locator("#forwardBtn").click();
  await expect(page.locator("#status")).toHaveText(`Reviewing: move ${total} of ${total}.`);
  await expect(page.locator("#forwardBtn")).toBeDisabled();
});

test("a game the AI started restores with the AI as first player", async ({ page }) => {
  await openBoard(page);
  await page.locator("button[data-first='ai']").click();
  await page.locator("button[data-nodes='4']").click();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(entries(page)).toHaveText([/^1\. AI \d+ to \d+$/]);

  await page.reload();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(entries(page)).toHaveText([/^1\. AI \d+ to \d+$/]);
  await expect(page.locator("button[data-first='ai']")).toHaveAttribute("aria-pressed", "true");
  await expect(page.locator("#turn")).toHaveAttribute("data-player", "human");
  await expect(page.locator("#undoBtn")).toBeEnabled();

  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(entries(page)).toHaveText([/^1\. AI \d+ to \d+$/]);
});

test("the move list buttons meet the touch target size", async ({ page }) => {
  await openBoard(page);
  await playExchange(page);
  for (const id of ["#backBtn", "#forwardBtn", "#shareBtn"]) {
    const box = await page.locator(id).boundingBox();
    expect(box?.height ?? 0).toBeGreaterThanOrEqual(44);
  }
  const entry = await entries(page).first().locator("button").boundingBox();
  expect(entry?.height ?? 0).toBeGreaterThanOrEqual(44);
  await shot(page, "history-panel");
});
