import { mkdir, readFile, writeFile } from "node:fs/promises";
import {
  expect as baseExpect,
  test,
  type BrowserContextOptions,
  type Page,
} from "@playwright/test";
import { decodeRecord } from "../../web/history";
import type { Fixture } from "../../web/record";

// Every step here replays moves through the engine, which on the 1000
// board takes several seconds per apply.
const expect = baseExpect.configure({ timeout: 20_000 });
import {
  clickAway,
  frameOf,
  nodeOnScreen,
  openBoard,
  PLACING_HINT,
  playExchange,
  shot,
  startGame,
  strokeCells,
} from "./helpers";

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

// Link payload lengths in characters, after the hash prefix: one exchange
// on either board, and the nine move game, measured with headroom.
const EXCHANGE_LINK_BUDGET = 60;
const NINE_MOVE_LINK_BUDGET = 500;
const HASH_PREFIX = "#game=";
const NINE_MOVES = "fixtures/saved/nine-moves-4n-1000.json";
// A version 1 link: one exchange on the 500 board.
const VERSION_1_LINK =
  "#game=AQAEAPQBAgACAAAAAQACABwMxAmvCbAJ8ArwCgIAAAAFAGwHxAn4A88BpgpR3gIe5AGEAvsKMwk";

// Two renderings of one board agree when under a tenth of the stroke
// cells differ.
function expectSameStrokes(before: boolean[], after: boolean[]): void {
  expect(after.length).toBe(before.length);
  const differing = before.filter((cell, i) => cell !== after[i]).length;
  const drawn = before.filter(Boolean).length;
  test.info().annotations.push({ type: "stroke-cells", description: `${differing} of ${drawn}` });
  expect(drawn).toBeGreaterThan(0);
  expect(differing).toBeLessThanOrEqual(Math.ceil(drawn * 0.1));
}

// Shares the game and returns the link and its payload length.
async function share(page: Page, label: string): Promise<{ url: string; payload: number }> {
  await page.locator("#shareBtn").click();
  await expect(page.locator("#status")).toHaveText(/Link copied|Copy the link/);
  const url = page.url();
  const at = url.indexOf(HASH_PREFIX);
  expect(at).toBeGreaterThan(0);
  const payload = url.length - at - HASH_PREFIX.length;
  test.info().annotations.push({ type: label, description: String(payload) });
  return { url, payload };
}

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
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await clickAway(page);
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

  await startGame(page, 4);
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
  const { url, payload } = await share(page, "share-payload-chars");
  expect(payload).toBeLessThan(EXCHANGE_LINK_BUDGET);
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

  expectSameStrokes(before, await strokeCells(other));

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

  await startGame(other, 4);
  await expect(other.locator("#status")).toHaveText("Draw a line between two nodes");
  expect(other.url()).not.toContain("#");
  await visitor.close();
});

test("a nine move game shares as a short link that rebuilds the same board", async ({
  page,
  browser,
}) => {
  test.slow();
  const record = await readFile(NINE_MOVES, "utf8");
  await page.addInitScript((saved) => localStorage.setItem("sprouts.game", saved), record);
  await page.goto("/");
  await expect(entries(page)).toHaveCount(9, { timeout: 120_000 });
  await expect(current(page)).toHaveText(/^9\./);
  await expect(page.locator("#status")).not.toHaveText(/Initializing|Replay/);
  const before = await strokeCells(page);
  const { url, payload } = await share(page, "nine-move-payload-chars");
  expect(payload).toBeLessThan(NINE_MOVE_LINK_BUDGET);
  await shot(page, "history-nine-source");

  const visitor = await browser.newContext(visitorOptions());
  const other = await visitor.newPage();
  await other.goto(url);
  await expect(other.locator("#status")).toHaveText(/^Reviewing: move 9 of 9/, {
    timeout: 120_000,
  });
  await expect(entries(other)).toHaveText(await entries(page).allTextContents());
  await shot(other, "history-nine-visitor");
  expectSameStrokes(before, await strokeCells(other));

  await other.locator("#backBtn").click();
  await expect(current(other)).toHaveText(/^8\./);
  await other.locator("#forwardBtn").click();
  await expect(current(other)).toHaveText(/^9\./);
  await expect(other.locator("#forwardBtn")).toBeDisabled();
  await visitor.close();
});

test("a review with moves the engine has not refitted still shares", async ({ page, browser }) => {
  const saved = JSON.parse(await readFile("fixtures/saved/finished-4n-500.json", "utf8")) as {
    cursor: number;
  };
  saved.cursor = 2;
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(saved));
  await page.goto("/");
  await expect(entries(page)).toHaveCount(9, { timeout: 60_000 });
  await expect(current(page)).toHaveText(/^2\./);
  await expect(page.locator("#forwardBtn")).toBeEnabled();
  const { url } = await share(page, "partial-review-payload-chars");

  const visitor = await browser.newContext(visitorOptions());
  const other = await visitor.newPage();
  await other.goto(url);
  await expect(other.locator("#status")).toHaveText(/^Reviewing: move 9 of 9/, {
    timeout: 60_000,
  });
  await expect(entries(other)).toHaveText(await entries(page).allTextContents());
  await visitor.close();
});

// The saved game holds the engine's exact nodes and the link rounds them
// by up to a quarter unit per axis on this 500 board.
test("a version 1 link of the saved game reopens the live game", async ({ page }) => {
  const linked = decodeRecord(VERSION_1_LINK.slice(HASH_PREFIX.length));
  if (!linked) throw new Error("the version 1 link does not decode");
  const saved = {
    ...linked,
    moves: linked.moves.map((m) => ({
      ...m,
      newNode: [m.newNode[0] + 0.24, m.newNode[1] - 0.24],
    })),
  };
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(saved));
  await page.goto("/" + VERSION_1_LINK);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 20_000 });
  expect(page.url()).not.toContain("#");
  await expect(entries(page)).toHaveText(["1. You 0 to 1", "2. AI 2 to 0"]);
  await expect(page.locator("#undoBtn")).toBeEnabled();
});

test("a version 1 link opens in review with its moves", async ({ page }) => {
  await page.goto("/" + VERSION_1_LINK);
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move 2 of 2/, {
    timeout: 20_000,
  });
  await expect(entries(page)).toHaveText(["1. You 0 to 1", "2. AI 2 to 0"]);
  await expect(current(page)).toHaveText("2. AI 2 to 0");
  await expect(page.locator("#forwardBtn")).toBeDisabled();
  await shot(page, "history-version-1-link");
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
  await startGame(page, 4);
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
