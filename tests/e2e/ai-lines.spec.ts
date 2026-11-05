import { expect, test, type Page } from "@playwright/test";
import {
  boardSettled,
  clickAway,
  mouseStroke,
  nodeOnScreen,
  openBoard,
  PLACING_HINT,
  shot,
} from "./helpers";

// A human move between two live nodes followed by the AI's reply. Resolves
// false when the stroke or its node is refused, since an earlier line may
// be in the way; the caller tries another pair.
async function exchange(page: Page, from: number, to: number): Promise<boolean> {
  await boardSettled(page);
  const a = await nodeOnScreen(page, from, 4);
  const b = await nodeOnScreen(page, to, 4);
  await mouseStroke(page, a, b);
  const status = page.locator("#status");
  await expect(status).not.toHaveText(/Draw a line|Your turn/);
  if (!PLACING_HINT.test((await status.textContent()) ?? "")) return false;
  await clickAway(page);
  await expect(status)
    .toHaveText(/Your turn!|AI wins|You win!|AI error/, { timeout: 60_000 })
    .catch(() => {});
  const text = (await status.textContent()) ?? "";
  if (/AI error/.test(text)) throw new Error(text);
  return /Your turn!|AI wins|You win!/.test(text);
}

// Ids of the nodes with a line left, from their hit targets.
async function liveNodes(page: Page): Promise<number[]> {
  const names = await page
    .locator("#nodeTargets button")
    .evaluateAll((buttons) =>
      buttons.map((b) => b.getAttribute("aria-label") ?? b.textContent ?? ""),
    );
  return names.map((name) => Number(/Node (\d+)/.exec(name)?.[1])).filter(Number.isFinite);
}

// Three exchanges on the phone layout, one screenshot per AI reply, so the
// routed lines can be looked at for shape and clearance. Each exchange
// tries straight strokes between live nodes until one is accepted, since
// the settled lines bow across the room the board has.
test("the AI's replies on a phone screen are drawn clear of the board", async ({ page }) => {
  test.skip(test.info().project.name !== "mobile", "mobile only");
  test.setTimeout(300_000);
  await openBoard(page);
  let played = 0;
  let attempts = 0;
  while (played < 3 && attempts < 12) {
    const live = await liveNodes(page);
    let done = false;
    for (let i = 0; i < live.length && !done; i++) {
      for (let j = i + 1; j < live.length && !done; j++) {
        attempts += 1;
        done = await exchange(page, live[i]!, live[j]!);
      }
    }
    if (!done) break;
    played += 1;
    await shot(page, `ai-reply-${played}`);
  }
  expect(played).toBe(3);
});
