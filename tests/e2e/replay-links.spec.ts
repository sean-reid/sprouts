import { readFileSync } from "node:fs";
import { basename } from "node:path";
import { expect, test, type BrowserContextOptions, type Page } from "@playwright/test";
import { decodeRecord, DRIFT_TOLERANCE, hasStroke, isIndexed, sameGame } from "../../web/history";
import type { Fixture } from "../../web/record";
import { HIGHLIGHT_COLOR } from "../../web/render";
import {
  awaitReply,
  expectSameStrokes,
  HASH_PREFIX,
  interceptWorkerMessages,
  recordNodesOnScreen,
  colourPixelsNear,
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
// when the regenerated board refuses a move, as the nine-node and
// thirty-move games do. The budget belongs to the recording: the payload
// measured when the game was recorded (34, 1186 and 1868) plus 10
// percent, rounded up.
const routed: [string, number, "indices" | "shapes", number][] = [
  ["fixtures/saved/eight-moves-4n-1000.json", 38, "indices", 4],
  ["fixtures/saved/ten-moves-9n-1000.json", 1305, "shapes", 5],
  ["fixtures/saved/thirty-moves-12n-1000.json", 2055, "shapes", 15],
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
  await page.addInitScript((s) => localStorage.setItem("sprouts.game", s), JSON.stringify(record));
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
    const total = await openSaved(page, file);
    await expect(page.locator("#status")).toHaveText("Your turn!");
    const before = await strokeCells(page);
    const { url, payload } = await share(page, `${name}-payload-chars`, SHARE_WAIT);
    expect(payload).toBeLessThan(budget);
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
    const again = await share(other, `${name}-visitor-payload-chars`, SHARE_WAIT);
    const relinked = decodeRecord(
      again.url.slice(again.url.indexOf(HASH_PREFIX) + HASH_PREFIX.length),
    );
    if (!relinked) throw new Error("the visitor's link does not decode");
    expect(sameGame(linked, relinked)).toBe(true);
    if (form === "indices") {
      // The regenerated board is a pure function of the move list, so the
      // visitor's link is the sharer's.
      expect(again.payload).toBe(payload);
      await visitor.close();
      // The sharer opening their own link lands on the live game, since
      // the saved record indexes to the link's moves.
      await page.goto(url);
      await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 240_000 });
      expect(page.url()).not.toContain("#");
      await expect(entries(page)).toHaveCount(total);
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
