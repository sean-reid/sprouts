import { readFileSync } from "node:fs";
import { basename } from "node:path";
import { expect, test, type BrowserContextOptions, type Page } from "@playwright/test";
import { decodeRecord, hasStroke, sameGame } from "../../web/history";
import type { Fixture } from "../../web/record";
import { HIGHLIGHT_COLOR } from "../../web/render";
import {
  awaitReply,
  expectSameStrokes,
  HASH_PREFIX,
  recordNodesOnScreen,
  ringSamplesAround,
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

// Games played on the page against the current AI, the human strokes
// wobbly, so every AI move travels as its face move and every human move
// as geometry. Payload budgets in characters: measured at 231 and 440 on
// the desktop project, with headroom.
const routed: [string, number][] = [
  ["fixtures/saved/eight-moves-4n-1000.json", 250],
  ["fixtures/saved/ten-moves-9n-1000.json", 480],
];

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
    await other.goto(url);
    await expect(other.locator("#status")).toHaveText(
      new RegExp(`^Reviewing: move ${total} of ${total}\\.`),
      { timeout: 240_000 },
    );
    // The visitor lands on the last move, which the board points out for
    // two seconds from the end of the seek.
    const last = record.moves[total - 1]!;
    const nodes = await recordNodesOnScreen(other, record);
    for (const id of [last.from, last.to]) {
      await expect
        .poll(() => ringSamplesAround(other, nodes[id]!, 14, HIGHLIGHT_COLOR))
        .toBeGreaterThanOrEqual(8);
    }
    await expect(entries(other)).toHaveCount(total);
    await visitor.close();
  });
}

for (const [file, budget] of routed) {
  const name = basename(file, ".json");
  test(`${name} shares its AI moves as topology and rebuilds the same board`, async ({
    page,
    browser,
  }) => {
    test.setTimeout(600_000);
    const total = await openSaved(page, file);
    await expect(page.locator("#status")).toHaveText("Your turn!");
    const before = await strokeCells(page);
    const { url, payload } = await share(page, `${name}-payload-chars`);
    expect(payload).toBeLessThan(budget);
    await shot(page, `${name}-source`);

    const linked = decodeRecord(url.slice(url.indexOf(HASH_PREFIX) + HASH_PREFIX.length));
    if (!linked) throw new Error("the link does not decode");
    expect(linked.moves).toHaveLength(total);
    // The human moves first, and strokes stay geometry. AI lines travel as
    // topology unless the share's rehearsal found the visitor's engine
    // would refuse one.
    const humanMoves = linked.moves.filter((_, i) => i % 2 === 0);
    expect(humanMoves.every(hasStroke)).toBe(true);
    const aiMoves = linked.moves.filter((_, i) => i % 2 === 1);
    const asTopology = aiMoves.filter((move) => !hasStroke(move)).length;
    test.info().annotations.push({
      type: `${name}-topology-moves`,
      description: `${asTopology} of ${aiMoves.length}`,
    });
    expect(asTopology).toBeGreaterThan(0);

    const visitor = await browser.newContext(visitorOptions());
    const other = await visitor.newPage();
    await other.goto(url);
    await expect(other.locator("#status")).toHaveText(
      new RegExp(`^Reviewing: move ${total} of ${total}\\.`),
      { timeout: 240_000 },
    );
    await expect(entries(other)).toHaveText(await entries(page).allTextContents());
    await shot(other, `${name}-visitor`);
    expectSameStrokes(before, await strokeCells(other));

    // The visitor's engine drew the topology moves, so sharing again gives
    // a link to the same game with the same moves as topology.
    const again = await share(other, `${name}-visitor-payload-chars`);
    const relinked = decodeRecord(
      again.url.slice(again.url.indexOf(HASH_PREFIX) + HASH_PREFIX.length),
    );
    if (!relinked) throw new Error("the visitor's link does not decode");
    expect(sameGame(linked, relinked)).toBe(true);
    expect(relinked.moves.map(hasStroke)).toEqual(linked.moves.map(hasStroke));
    await visitor.close();
  });
}
