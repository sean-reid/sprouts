import { readFileSync } from "node:fs";
import { basename } from "node:path";
import { expect, test, type BrowserContextOptions, type Page } from "@playwright/test";
import {
  decodeRecord,
  DRIFT_TOLERANCE,
  encodeRecord,
  hasStroke,
  History,
  isIndexed,
  sameGame,
} from "../../web/history";
import type { Fixture } from "../../web/record";
import { HIGHLIGHT_COLOR } from "../../web/render";
import {
  awaitReply,
  expectSameStrokes,
  HASH_PREFIX,
  interceptWorkerMessages,
  recordNodesOnScreen,
  colourPixelsNear,
  clickAway,
  drawBetween,
  expectDotAt,
  holdNext,
  lastRequestArgs,
  openBoard,
  playExchange,
  requestCount,
  rowIds,
  savedRecord,
  share,
  shot,
  strokeCells,
} from "./helpers";

// A recorded game must survive a share: the engine refits each move from
// the link's thinned geometry, and a move it then rejects stops the
// visitor's replay short. Three games cover a hand-drawn one, a finished
// one and an AI one on the small board; the codec unit tests cover the
// geometry of every fixture.
const fixtures = [
  "fixtures/saved/nine-moves-4n-1000.json",
  "fixtures/saved/finished-4n-500.json",
  "fixtures/games/selfplay-4n-500.json",
];

// Games recorded by the record_shared_games test in src/route/tests.rs,
// the human strokes bulged so the router does not redraw them. Per game:
// the payload budget in characters, the form the link takes, and for a
// shapes link how many AI moves go as topology. The link carries indices
// when the engine regenerates the whole game on a fresh board, and shapes
// when the regenerated board refuses a move, as the nine-node,
// thirty-move and 24-node games do. The budget belongs to the recording:
// the payload measured on router version 5 with the recorder's default
// picks (34, 1486, 2047 and 691) plus 10 percent, rounded up. A payload
// depends on the router version, the pick and the recorder, so a new
// recording brings its own budget.
const routed: [string, number, "indices" | "shapes", number][] = [
  ["fixtures/saved/eight-moves-4n-1000.json", 38, "indices", 4],
  ["fixtures/saved/ten-moves-9n-1000.json", 1635, "shapes", 5],
  ["fixtures/saved/thirty-moves-12n-1000.json", 2252, "shapes", 15],
  ["fixtures/saved/ten-moves-24n-1000.json", 761, "shapes", 5],
];

// A share replays the game up to three times in the engine, routing
// every move; the thirty-move game takes about a minute on the runner.
const SHARE_WAIT = 180_000;

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

// Opens the page on a recorded game and returns its move count.
async function openSaved(page: Page, file: string): Promise<number> {
  const fixture = JSON.parse(readFileSync(file, "utf8")) as Fixture;
  const record = { firstPlayer: "human", cursor: fixture.moves.length, ...fixture };
  await page.addInitScript(interceptWorkerMessages);
  // Seeded once, so a reload keeps what the page saved since.
  await page.addInitScript((s) => {
    if (!localStorage.getItem("sprouts.game")) localStorage.setItem("sprouts.game", s);
  }, JSON.stringify(record));
  await page.goto("/");
  await expect(entries(page)).toHaveCount(fixture.moves.length, { timeout: 240_000 });
  await expect(page.locator("#status")).not.toHaveText(/Initializing|Replay/, {
    timeout: 240_000,
  });
  return fixture.moves.length;
}

for (const file of fixtures) {
  test(`${basename(file, ".json")} replays every move from its link`, async ({ page, browser }) => {
    test.skip(test.info().project.name !== "desktop", "a replay does not depend on the viewport");
    test.setTimeout(600_000);
    await openSaved(page, file);
    await awaitReply(page);
    const record = await savedRecord(page);
    const total = record.moves.length;

    const { url } = await share(page, `${basename(file, ".json")}-payload-chars`);
    const visitor = await browser.newContext();
    const other = await visitor.newPage();
    await other.addInitScript(interceptWorkerMessages);
    await other.goto(url);
    await expect(other.locator("#status")).toHaveText(
      new RegExp(`^Reviewing: move ${total} of ${total}\\.`),
      { timeout: 240_000 },
    );
    // The visitor lands on the last move. Pointing at its row keeps the
    // board pointing the move out, so the highlight is read off the move
    // the page marks current, at the node ids its row names. The lines
    // through an end node cross its ring, so the check counts highlight
    // pixels in a 40 px box around the node: ring and line outline come to
    // about 350, the node's own label to about 20.
    await expect(entries(other)).toHaveCount(total);
    const current = other.locator("#moveList [aria-current]");
    const index = Number(await current.getAttribute("data-move"));
    expect(index).toBe(total - 1);
    await entries(other).nth(index).locator("button").hover();
    const [from, to] = await rowIds(other, index);
    const nodes = await recordNodesOnScreen(other, record);
    for (const id of [from, to]) {
      await expect
        .poll(() => colourPixelsNear(other, nodes[id]!, 20, HIGHLIGHT_COLOR))
        .toBeGreaterThanOrEqual(60);
      test.info().annotations.push({
        type: `highlight-pixels-node-${id}`,
        description: String(await colourPixelsNear(other, nodes[id]!, 20, HIGHLIGHT_COLOR)),
      });
    }
    await visitor.close();
  });
}

for (const [file, budget, form, topologyMoves] of routed) {
  const name = basename(file, ".json");
  test(`${name} shares its moves as ${form} and the visitor rebuilds the game`, async ({
    page,
    browser,
  }) => {
    test.setTimeout(600_000);
    const opened = Date.now();
    const total = await openSaved(page, file);
    await expect(page.locator("#status")).toHaveText("Your turn!");
    test.info().annotations.push({
      type: `${name}-restore-ms`,
      description: String(Date.now() - opened),
    });
    const before = await strokeCells(page);
    // The record predates the cache, so the first share indexes once and
    // the second reads the cache or the refused marker.
    const started = Date.now();
    const { url, payload } = await share(page, `${name}-payload-chars`, SHARE_WAIT);
    test.info().annotations.push({
      type: `${name}-first-share-ms`,
      description: String(Date.now() - started),
    });
    expect(await requestCount(page, "indexMoves")).toBe(1);
    const firstRehearsals = await requestCount(page, "rehearse");
    const again = Date.now();
    const second = await share(page, `${name}-payload-chars-again`, SHARE_WAIT);
    test.info().annotations.push({
      type: `${name}-second-share-ms`,
      description: String(Date.now() - again),
    });
    expect(second.url).toBe(url);
    expect(await requestCount(page, "indexMoves")).toBe(1);
    expect(payload).toBeLessThan(budget);
    // A shapes game's second share reads the forms the first settled.
    if (form === "shapes") {
      expect(await requestCount(page, "classify")).toBe(1);
      expect(await requestCount(page, "rehearse")).toBe(firstRehearsals);
    }
    await expect(page.locator("#status")).not.toHaveText(/replay may stop|may differ/);
    await shot(page, `${name}-source`);

    const linked = decodeRecord(url.slice(url.indexOf(HASH_PREFIX) + HASH_PREFIX.length));
    if (!linked) throw new Error("the link does not decode");
    expect(linked.moves).toHaveLength(total);
    if (form === "indices") {
      expect(linked.moves.every(isIndexed)).toBe(true);
    } else {
      // The human moves first, and strokes stay geometry.
      const humanMoves = linked.moves.filter((_, i) => i % 2 === 0);
      expect(humanMoves.every(hasStroke)).toBe(true);
      const aiMoves = linked.moves.filter((_, i) => i % 2 === 1);
      expect(aiMoves.filter((move) => !hasStroke(move))).toHaveLength(topologyMoves);
    }

    const visitor = await browser.newContext(visitorOptions());
    const other = await visitor.newPage();
    await other.goto(url);
    await expect(other.locator("#status")).toHaveText(
      new RegExp(`^Reviewing: move ${total} of ${total}\\.`),
      { timeout: 240_000 },
    );
    // The same face moves give the same rows, ids and cut-off counts on
    // both ends, whether or not the boards share their geometry.
    await expect(entries(other)).toHaveText(await entries(page).allTextContents());
    for (let i = 0; i < total; i++) expect(await rowIds(other, i)).toEqual(await rowIds(page, i));
    await shot(other, `${name}-visitor`);
    if (form === "shapes") expectSameStrokes(before, await strokeCells(other));

    // Sharing again from the visitor's board gives a link to the same game.
    const visitorShare = await share(other, `${name}-visitor-payload-chars`, SHARE_WAIT);
    const relinked = decodeRecord(
      visitorShare.url.slice(visitorShare.url.indexOf(HASH_PREFIX) + HASH_PREFIX.length),
    );
    if (!relinked) throw new Error("the visitor's link does not decode");
    expect(sameGame(linked, relinked)).toBe(true);
    if (form === "indices") {
      // The regenerated board is a pure function of the move list, so the
      // visitor's link is the sharer's.
      expect(visitorShare.payload).toBe(payload);
      await visitor.close();
      // The share put the link in the address bar; a reload with it lands
      // on the live game, since the saved record indexes to the link's
      // moves.
      expect(page.url()).toBe(url);
      const reloaded = Date.now();
      await page.reload();
      await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 240_000 });
      test.info().annotations.push({
        type: `${name}-reopen-ms`,
        description: String(Date.now() - reloaded),
      });
      expect(page.url()).not.toContain("#");
      await expect(entries(page)).toHaveCount(total);
      expect(await requestCount(page, "indexMoves")).toBe(0);
      return;
    }
    expect(relinked.moves.map(hasStroke)).toEqual(linked.moves.map(hasStroke));
    // The visitor's router placed every topology move's node within the
    // drift the share allowed of the sharer's.
    const drift = linked.moves.map((move, i) => {
      const other = relinked.moves[i];
      if (!other || hasStroke(move) || hasStroke(other) || isIndexed(move) || isIndexed(other)) {
        return 0;
      }
      return Math.hypot(move.node[0] - other.node[0], move.node[1] - other.node[1]);
    });
    const largest = Math.max(...drift);
    test
      .info()
      .annotations.push({ type: `${name}-largest-drift`, description: largest.toFixed(3) });
    expect(largest).toBeLessThanOrEqual((DRIFT_TOLERANCE * linked.boardSize) / 1000);
    await visitor.close();
  });
}

// A new move after an undo discards the moves ahead with their cached
// indices, so the next share indexes the whole game again.
test("a share after an undo and a new move indexes the whole game again", async ({ page }) => {
  test.setTimeout(300_000);
  await openBoard(page);
  await playExchange(page);
  await share(page, "exchange-payload-chars", SHARE_WAIT);
  expect(await requestCount(page, "indexMoves")).toBe(1);
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  await playExchange(page);
  await share(page, "exchange-again-payload-chars", SHARE_WAIT);
  expect(await requestCount(page, "indexMoves")).toBe(2);
  const [plan] = (await lastRequestArgs(page, "indexMoves")) as [{ moves: unknown[] }];
  expect(plan.moves).toHaveLength(2);
});

// A share while the AI is thinking would queue its replays ahead of the
// AI's move in the worker, so the button is disabled and the click does
// nothing; the AI's move lands.
test("Share is refused during the AI's turn and the move still lands", async ({ page }) => {
  await openBoard(page);
  await holdNext(page, "aiMove", 3_000);
  const mid = await drawBetween(page, 0, 1, 4);
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText(/AI is thinking/);
  await expect(page.locator("#shareBtn")).toBeDisabled();
  // A disabled button never dispatches a click, so the attribute comes off
  // and the guard in share() is what refuses the click.
  await page.evaluate(() => document.getElementById("shareBtn")?.removeAttribute("disabled"));
  await page.locator("#shareBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#moveList li")).toHaveCount(2);
  expect(await requestCount(page, "indexMoves")).toBe(0);
  await expect(page.locator("#shareBtn")).toBeEnabled();
});

// A long index link over a saved game with the same header is opened in
// review first and checked against the saved record in a worker of its
// own, so when it turns out to be another game the review still steps.
test("a long index link that is not the saved game opens in review and steps", async ({ page }) => {
  test.setTimeout(600_000);
  const file = "fixtures/saved/thirty-moves-12n-1000.json";
  const fixture = JSON.parse(readFileSync(file, "utf8")) as Fixture;
  const saved = { firstPlayer: "human", cursor: fixture.moves.length, ...fixture };
  // Fifteen moves on the same header, each the first move the engine
  // lists, which is not the saved game.
  const other = new History(fixture.initialNodes, fixture.boardSize, "human", 1);
  for (let i = 0; i < 15; i++) {
    other.push({ fromNode: 0, toNode: 0, polyline: [{ x: 0, y: 0 }], newNodePos: { x: 0, y: 0 } });
    other.moves[i]!.indexed = { index: 0, along: 0 };
  }
  await page.addInitScript(interceptWorkerMessages);
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(saved));
  await page.goto("/" + HASH_PREFIX + encodeRecord(other));
  // The check may finish before the status can be read, so the end state
  // is what is asserted: the link stays open and one indexMoves request
  // went to the checker's worker.
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move \d+ of \d+|^Replay stopped/, {
    timeout: 240_000,
  });
  await expect.poll(() => requestCount(page, "indexMoves"), { timeout: 240_000 }).toBe(1);
  await expect(page.locator("#status")).toHaveText(/^Reviewing: move \d+ of \d+|^Replay stopped/, {
    timeout: 240_000,
  });
  expect(page.url()).toContain("#");
  const rows = await page.locator("#moveList li").count();
  expect(rows).toBeGreaterThan(1);
  await page.locator("#backBtn").click();
  await expect(page.locator("#status")).toHaveText(new RegExp(`^Reviewing: move ${rows - 1} of`), {
    timeout: 60_000,
  });
  // The saved game is untouched.
  const kept = await page.evaluate(() => JSON.parse(localStorage.getItem("sprouts.game") ?? "{}"));
  expect((kept as { moves: unknown[] }).moves).toHaveLength(fixture.moves.length);
});

// A version 4 link of two moves, the second a face move, with its router
// byte set to a version this page does not have.
const OTHER_ROUTER_LINK = "#game=BP8ABADoAwIAAgAAAQAC2AToB6AGAI8DAAICAgQEAgAE2AnoBw";

test("a link from another router opens a fresh game and says so", async ({ page }) => {
  await page.goto("/" + OTHER_ROUTER_LINK);
  await expect(page.locator("#status")).toHaveText(/another version of the game/, {
    timeout: 20_000,
  });
  expect(page.url()).not.toContain("#");
  await expect(entries(page)).toHaveCount(0);
  await expect(page.locator("#canvas")).toBeVisible();
});
