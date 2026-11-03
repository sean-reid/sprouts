import { expect, test, type Page } from "@playwright/test";
import {
  drawBetween,
  nodeOnScreen,
  openBoard,
  playExchange,
  shot,
  touchStroke,
  type Client,
} from "./helpers";

// Installed before the page loads so a test can hold one request to the
// worker back; the worker then answers a newer request first.
function holdOneWorkerMessage(): void {
  const post = Worker.prototype.postMessage;
  let hold: { method: string; ms: number } | null = null;
  (window as unknown as { holdNext: typeof holdNext }).holdNext = holdNext;
  function holdNext(method: string, ms: number): void {
    hold = { method, ms };
  }
  Worker.prototype.postMessage = function (this: Worker, ...args: unknown[]) {
    const envelope = args[0] as { method?: string } | undefined;
    if (hold && envelope?.method === hold.method) {
      const { ms } = hold;
      hold = null;
      setTimeout(() => Reflect.apply(post, this, args), ms);
      return;
    }
    Reflect.apply(post, this, args);
  };
}

async function alongFirstChord(page: Page, t: number): Promise<Client> {
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

// Reads the canvas pixel under a client point and says whether it is the
// placement dot (green when valid, red when not) rather than stroke or board.
function dotAt(page: Page, at: Client): Promise<boolean> {
  return page.evaluate(({ x, y }) => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const rect = canvas.getBoundingClientRect();
    const px = Math.round((x - rect.left) * (canvas.width / rect.width));
    const py = Math.round((y - rect.top) * (canvas.height / rect.height));
    const [r, g, b] = canvas.getContext("2d")!.getImageData(px, py, 1, 1).data;
    const near = (cr: number, cg: number, cb: number) => Math.hypot(r! - cr, g! - cg, b! - cb) < 60;
    return near(0x10, 0xb9, 0x81) || near(0xef, 0x44, 0x44);
  }, at);
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

test("a touch stroke locks the line and Confirm places the node", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  const cdp = await page.context().newCDPSession(page);
  await touchStroke(cdp, a, b);
  await expect(page.locator("#status")).toHaveText(/Tap Confirm/);
  await expect(page.locator("#mobileOverlay")).toBeVisible();
  await shot(page, "touch-placing");
  await page.locator("#confirmBtn").click();
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#mobileOverlay")).toBeHidden();
});

test("a late reply to an older mousemove cannot overwrite a newer placement dot", async ({
  page,
}) => {
  await drawBetween(page, 0, 1);
  const older = await alongFirstChord(page, 0.3);
  const newer = await alongFirstChord(page, 0.7);
  await page.evaluate(() => {
    (window as unknown as { holdNext: (m: string, ms: number) => void }).holdNext("placement", 400);
  });
  await page.mouse.move(older.x, older.y);
  await page.mouse.move(newer.x, newer.y);
  await expect.poll(() => dotAt(page, newer)).toBe(true);
  await page.waitForTimeout(800);
  await shot(page, "stale-placement");
  expect(await dotAt(page, newer)).toBe(true);
  expect(await dotAt(page, older)).toBe(false);
});
