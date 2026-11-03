import { readdirSync, readFileSync } from "node:fs";
import { basename, join } from "node:path";
import { expect, test } from "@playwright/test";
import type { Fixture } from "../../web/record";

// Every recorded game must survive a share: the engine refits each move
// from the link's thinned geometry, and a move it then rejects stops the
// visitor's replay short.
const FIXTURE_DIRS = ["fixtures/games", "fixtures/saved"];
const fixtures = FIXTURE_DIRS.flatMap((dir) =>
  readdirSync(dir)
    .filter((f) => f.endsWith(".json"))
    .map((f) => join(dir, f)),
);

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
    await expect(entries(other)).toHaveCount(total);
    await visitor.close();
  });
}
