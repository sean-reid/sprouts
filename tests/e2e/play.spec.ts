import { expect, test, type Page } from "@playwright/test";
import {
  dotColourAt,
  drawBetween,
  expectDotAt,
  holdNext,
  holdOneWorkerMessage,
  nodeOnScreen,
  openBoard,
  PLACING_HINT,
  playExchange,
  shot,
  tapAway,
  touchStroke,
  type Client,
} from "./helpers";

async function alongFirstChord(page: Page, t: number): Promise<Client> {
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(holdOneWorkerMessage);
  await openBoard(page);
});

test("opens on a four node board with undo disabled", async ({ page }) => {
  await expect(page.locator("#canvas")).toBeVisible();
  await expect(page.locator("button[data-nodes]")).toHaveCount(3);
  await expect(page.locator("#undoBtn")).toBeDisabled();
  await shot(page, "start");
});

test("escape cancels a locked line", async ({ page }) => {
  await drawBetween(page, 0, 1);
  await shot(page, "placing");
  await page.keyboard.press("Escape");
  await expect(page.locator("#status")).toHaveText("Move cancelled");
});

test("placing the node commits the move and the AI replies", async ({ page }) => {
  await playExchange(page);
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await shot(page, "after-first-exchange");
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  await expect(page.locator("#undoBtn")).toBeDisabled();
});

test("a touch stroke locks the line and a tap elsewhere places the node", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  const cdp = await page.context().newCDPSession(page);
  await touchStroke(cdp, a, b);
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await expectDotAt(page, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });
  await expect(page.locator("#cancelBtn")).toBeVisible();
  await shot(page, "touch-placing");
  await tapAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#cancelBtn")).toBeHidden();
});

test("a late reply to an older drag position cannot overwrite a newer placement dot", async ({
  page,
}) => {
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  const older = await alongFirstChord(page, 0.3);
  const newer = await alongFirstChord(page, 0.7);
  await page.mouse.move(mid.x, mid.y);
  await page.mouse.down();
  await holdNext(page, "placement", 400);
  await page.mouse.move(older.x, older.y);
  await page.mouse.move(newer.x, newer.y);
  await expectDotAt(page, newer);
  await page.mouse.up();
  await page.waitForTimeout(800);
  await shot(page, "stale-placement");
  expect(await dotColourAt(page, newer)).toBe("green");
  expect(await dotColourAt(page, older)).toBeNull();
});
