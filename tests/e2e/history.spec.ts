import { mkdir, readFile, writeFile } from "node:fs/promises";
import {
  expect as baseExpect,
  test,
  type BrowserContextOptions,
  type Page,
} from "@playwright/test";
import { decodeRecord, hasStroke } from "../../web/history";
import type { Fixture } from "../../web/record";

// Every step here replays moves through the engine, which on the 1000
// board takes several seconds per apply.
const expect = baseExpect.configure({ timeout: 20_000 });
import {
  awaitReply,
  boardNodes,
  boardSettled,
  clickAway,
  expectNodeDrawnAt,
  expectSameStrokes,
  frameOf,
  HASH_PREFIX,
  interceptWorkerMessages,
  nodeOnScreen,
  openBoard,
  PLACING_HINT,
  playExchange,
  recordedNodeOnScreen,
  share,
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
// on either board, and the nine move game with the AI's reply to it,
// measured with headroom. One exchange runs to about 62, since the chord
// on the sunflower spans a third of the board; the nine move game runs
// to 484, its raster-era AI lines being nothing the router redraws.
const EXCHANGE_LINK_BUDGET = 70;
const NINE_MOVE_LINK_BUDGET = 520;
const NINE_MOVES = "fixtures/saved/nine-moves-4n-1000.json";
// A version 1 link: one exchange on the 500 board.
const VERSION_1_LINK =
  "#game=AQAEAPQBAgACAAAAAQACABwMxAmvCbAJ8ArwCgIAAAAFAGwHxAn4A88BpgpR3gIe5AGEAvsKMwk";
// A version 2 link of the saved first exchange, made when links carried no layout.
const VERSION_2_LINK = "#game=AgAEAOgDAgACAAABAALYCegH7wHwAXh3AgAAA_gF6AeaAyVGJkUl";

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
  await expect(entries(page)).toHaveText(["1. You, joins two nodes", /^2\. AI, /]);
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
  await expect(current(page)).toHaveText("1. You, joins two nodes");

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

// A link rounds the drawn geometry to its quantum and the visitor's board
// settles from that, so a settled game comes back within a unit per node
// rather than stroke for stroke.
const SETTLED_REPLAY_UNITS = 1;

test("a share link reopens the game in review and steps through it", async ({ page, browser }) => {
  await openBoard(page);
  await playExchange(page);
  await boardSettled(page);
  const source = await boardNodes(page);
  const { url, payload } = await share(page, "share-payload-chars");
  expect(payload).toBeLessThan(EXCHANGE_LINK_BUDGET);
  await shot(page, "history-shared-source");

  const visitor = await browser.newContext(visitorOptions());
  const other = await visitor.newPage();
  await other.addInitScript(interceptWorkerMessages);
  await other.goto(url);
  await expect(other.locator("#status")).toHaveText(/^Reviewing: move 2 of 2/, {
    timeout: 20_000,
  });
  await expect(entries(other)).toHaveText(await entries(page).allTextContents());
  await expect(current(other)).toHaveText(/^2\. AI/);
  await expect(other.locator("#forwardBtn")).toBeDisabled();
  await expect(other.locator("#backBtn")).toBeEnabled();
  await shot(other, "history-shared-visitor");

  const linked = await boardNodes(other);
  expect(linked.size).toBe(source.size);
  let offset = 0;
  for (const [id, at] of source) {
    const there = linked.get(id)!;
    offset = Math.max(offset, Math.hypot(there.x - at.x, there.y - at.y));
  }
  test
    .info()
    .annotations.push({ type: "shared-node-offset-units", description: offset.toFixed(1) });
  expect(offset).toBeLessThanOrEqual(SETTLED_REPLAY_UNITS);

  await other.locator("#backBtn").click();
  await expect(current(other)).toHaveText("1. You, joins two nodes");
  await expect(other.locator("#forwardBtn")).toBeEnabled();
  await other.locator("#backBtn").click();
  await expect(other.locator("#backBtn")).toBeDisabled();
  await expect(current(other)).toHaveCount(0);
  await expect(other.locator("#status")).toHaveText(/^Reviewing: start/);
  await shot(other, "history-review-start");

  await other.keyboard.press("ArrowRight");
  await expect(current(other)).toHaveText("1. You, joins two nodes");
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
  // The record leaves the AI to move, and it has one move left.
  await awaitReply(page);
  const total = await entries(page).count();
  const before = await strokeCells(page);
  const { url, payload } = await share(page, "nine-move-payload-chars");
  expect(payload).toBeLessThan(NINE_MOVE_LINK_BUDGET);
  await shot(page, "history-nine-source");

  const visitor = await browser.newContext(visitorOptions());
  const other = await visitor.newPage();
  await other.goto(url);
  await expect(other.locator("#status")).toHaveText(
    new RegExp(`^Reviewing: move ${total} of ${total}`),
    { timeout: 120_000 },
  );
  await expect(entries(other)).toHaveText(await entries(page).allTextContents());
  await shot(other, "history-nine-visitor");
  expectSameStrokes(before, await strokeCells(other));

  await other.locator("#backBtn").click();
  await expect(current(other)).toHaveText(new RegExp(`^${total - 1}\\.`));
  await other.locator("#forwardBtn").click();
  await expect(current(other)).toHaveText(new RegExp(`^${total}\\.`));
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
  // The visitor replayed every move and reads each chord's count; the
  // sharer's rows past the cursor read plain until Forward reaches them.
  await expect(entries(other).nth(5)).toHaveText("6. AI, joins two nodes, cuts off 2");
  await expect(entries(page).nth(5)).toHaveText("6. AI, joins two nodes");
  for (let steps = 0; steps < 9 && (await page.locator("#forwardBtn").isEnabled()); steps++) {
    await page.locator("#forwardBtn").click();
    await expect(page.locator("#status")).not.toHaveText("Initializing...", { timeout: 60_000 });
    await expect(page.locator("#backBtn")).toBeEnabled({ timeout: 60_000 });
  }
  await expect(entries(page)).toHaveText(await entries(other).allTextContents());
  await visitor.close();
});

// The saved game holds the engine's exact nodes and the link rounds them
// by up to a quarter unit per axis on this 500 board.
test("a version 1 link of the saved game reopens the live game", async ({ page }) => {
  const linked = decodeRecord(VERSION_1_LINK.slice(HASH_PREFIX.length));
  if (!linked) throw new Error("the version 1 link does not decode");
  const saved = {
    ...linked,
    moves: linked.moves.map((m) => {
      if (!hasStroke(m)) throw new Error("a version 1 link carries strokes");
      return { ...m, newNode: [m.newNode[0] + 0.24, m.newNode[1] - 0.24] };
    }),
  };
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(saved));
  await page.goto("/" + VERSION_1_LINK);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 20_000 });
  expect(page.url()).not.toContain("#");
  await expect(entries(page)).toHaveText(["1. You, joins two nodes", "2. AI, joins two nodes"]);
  await expect(page.locator("#undoBtn")).toBeEnabled();
});

test("a version 2 link opens on the circle with both moves replayed", async ({ page }) => {
  await page.addInitScript(interceptWorkerMessages);
  await page.goto("/" + VERSION_2_LINK);
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move 2 of 2/, {
    timeout: 20_000,
  });
  await expect(entries(page)).toHaveText(["1. You, joins two nodes", "2. AI, joins two nodes"]);
  const record = JSON.parse(await readFile("fixtures/saved/first-exchange.json", "utf8")) as {
    boardSize: number;
    nodes: [number, number][];
  };
  for (let i = 0; i < 4; i++) {
    await expectNodeDrawnAt(page, await recordedNodeOnScreen(page, record, i));
  }
});

test("a saved game without positions restores onto the circle", async ({ page }) => {
  const saved = JSON.parse(await readFile("fixtures/saved/first-exchange.json", "utf8")) as {
    nodes?: unknown;
  };
  delete saved.nodes;
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(saved));
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 20_000 });
  await expect(entries(page)).toHaveText(["1. You, joins two nodes", "2. AI, joins two nodes"]);
  await expect(page.locator("#undoBtn")).toBeEnabled();
});

test("a version 1 link opens in review with its moves", async ({ page }) => {
  await page.goto("/" + VERSION_1_LINK);
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move 2 of 2/, {
    timeout: 20_000,
  });
  await expect(entries(page)).toHaveText(["1. You, joins two nodes", "2. AI, joins two nodes"]);
  await expect(current(page)).toHaveText("2. AI, joins two nodes");
  await expect(current(page)).toHaveAttribute("title", "2 to 0, new node 5");
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
  await expect(entries(page)).toHaveText([/^1\. AI, /]);

  await page.reload();
  await expect(page.locator("#status")).toHaveText("Your turn!");
  await expect(entries(page)).toHaveText([/^1\. AI, /]);
  await expect(page.locator("button[data-first='ai']")).toHaveAttribute("aria-pressed", "true");
  await expect(page.locator("#turn")).toHaveAttribute("data-player", "human");
  await expect(page.locator("#undoBtn")).toBeEnabled();

  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(entries(page)).toHaveText([/^1\. AI, /]);
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
