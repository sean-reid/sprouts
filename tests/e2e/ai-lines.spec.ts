import { expect, test, type Page } from "@playwright/test";
import { clickAway, mouseStroke, nodeOnScreen, openBoard, PLACING_HINT, shot } from "./helpers";

// A human move between two of the opening nodes followed by the AI's reply.
// Resolves false when the stroke or its node is refused, since an earlier
// line may be in the way; the caller tries another pair.
async function exchange(page: Page, from: number, to: number): Promise<boolean> {
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

// Three exchanges on the phone layout, one screenshot per AI reply, so the
// routed lines can be looked at for shape and clearance.
test("the AI's replies on a phone screen are drawn clear of the board", async ({ page }) => {
  test.skip(test.info().project.name !== "mobile", "mobile only");
  await openBoard(page);
  const pairs: [number, number][] = [
    [0, 1],
    [2, 3],
    [1, 2],
    [0, 3],
    [1, 3],
    [0, 2],
  ];
  let played = 0;
  for (const [from, to] of pairs) {
    if (!(await exchange(page, from, to))) continue;
    played += 1;
    await shot(page, `ai-reply-${played}`);
    if (played === 3) break;
  }
  expect(played).toBe(3);
});
