import { readFileSync } from "node:fs";
import { basename } from "node:path";
import { expect, test } from "@playwright/test";
import type { Fixture } from "../../web/record";
import { PLAYER_COLORS } from "../../web/render";
import { recordNodesOnScreen, ringSamplesAround } from "./helpers";

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

const entries = (page: import("@playwright/test").Page) => page.locator("#moveList li");

for (const file of fixtures) {
  test(`${basename(file, ".json")} replays every move from its link`, async ({ page, browser }) => {
    test.skip(test.info().project.name !== "desktop", "a replay does not depend on the viewport");
    test.setTimeout(600_000);
    const fixture = JSON.parse(readFileSync(file, "utf8")) as Fixture;
    const total = fixture.moves.length;
    const record = { firstPlayer: "human", cursor: total, ...fixture };
    await page.addInitScript(
      (s) => localStorage.setItem("sprouts.game", s),
      JSON.stringify(record),
    );
    await page.goto("/");
    await expect(entries(page)).toHaveCount(total, { timeout: 240_000 });
    await expect(page.locator("#status")).not.toHaveText(/Initializing|Replay/, {
      timeout: 240_000,
    });

    await page.locator("#shareBtn").click();
    await expect(page.locator("#status")).toHaveText(/Link copied|Copy the link/);
    const url = page.url();
    const visitor = await browser.newContext();
    const other = await visitor.newPage();
    await other.goto(url);
    await expect(other.locator("#status")).toHaveText(
      new RegExp(`^Reviewing: move ${total} of ${total}\\.`),
      { timeout: 240_000 },
    );
    // The visitor lands on the last move, which the board points out for
    // two seconds from the end of the seek.
    const last = fixture.moves[total - 1]!;
    const nodes = await recordNodesOnScreen(other, fixture);
    const colour = PLAYER_COLORS[(total - 1) % 2 === 0 ? "human" : "ai"];
    for (const id of [last.from, last.to]) {
      await expect
        .poll(() => ringSamplesAround(other, nodes[id]!, 14, colour))
        .toBeGreaterThanOrEqual(8);
    }
    await expect(entries(other)).toHaveCount(total);
    await visitor.close();
  });
}
