import { expect, test, type Page } from "@playwright/test";
import {
  nodeSamplesAround,
  clampToViewport,
  clickAway,
  type Client,
  dotColourAt,
  drawBetween,
  expectDotAt,
  expectNodeDrawnAt,
  expectNodeDrawnNear,
  failNext,
  findDot,
  fit,
  frameOf,
  holdNext,
  interceptWorkerMessages,
  mouseStroke,
  nodeAt,
  nodeOnScreen,
  nodeWorld,
  openBoard,
  pinch,
  pinchDrag,
  PLACING_HINT,
  playExchange,
  ringPixels,
  shot,
  startGame,
  tapAway,
  toClient,
  touchDrag,
  touchStroke,
  wheelZoomTo,
} from "./helpers";

const NO_ROOM = "No room for a new node on this line";

async function alongFirstChord(page: Page, t: number): Promise<Client> {
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

// Counts flash-red pixels in a square of the given half-width around a point.
function redPixelsNear(page: Page, at: Client, half: number): Promise<number> {
  return page.evaluate(
    ({ x, y, half }) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d");
      if (!ctx) return 0;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      const left = Math.round((x - rect.left - half) * dpr);
      const top = Math.round((y - rect.top - half) * dpr);
      const span = Math.round(half * 2 * dpr);
      const { data } = ctx.getImageData(left, top, span, span);
      let hits = 0;
      for (let i = 0; i < data.length; i += 4) {
        const [r, g, b] = [data[i] ?? 0, data[i + 1] ?? 0, data[i + 2] ?? 0];
        if (Math.hypot(r - 0xef, g - 0x44, b - 0x44) < 40) hits++;
      }
      return hits;
    },
    { x: at.x, y: at.y, half },
  );
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(interceptWorkerMessages);
  await openBoard(page);
});

test("a stroke locks and the dot appears with no further click", async ({ page }) => {
  const before = await frameOf(page);
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  expect(await frameOf(page)).toEqual(before);
  const cancel = page.locator("#cancelBtn");
  await expect(cancel).toBeVisible();
  const box = await cancel.boundingBox();
  expect(box?.height ?? 0).toBeGreaterThanOrEqual(44);
  expect(box?.width ?? 0).toBeGreaterThanOrEqual(44);
  await expect(page.locator("button:visible")).toContainText(["Cancel"]);
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await shot(page, "placement-dot");
});

test("a click away from the dot commits the move and the AI replies", async ({ page }) => {
  const mid = await alongFirstChord(page, 0.5);
  await playExchange(page);
  await expectNodeDrawnNear(page, mid, 4);
  await expect(page.locator("#cancelBtn")).toBeHidden();
  await expect(page.locator("#undoBtn")).toBeEnabled();
});

test("Enter commits the move", async ({ page }) => {
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await page.keyboard.press("Enter");
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expectNodeDrawnNear(page, mid, 4);
});

test("dragging the dot moves the committed node", async ({ page }) => {
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  const target = await alongFirstChord(page, 0.72);
  await page.mouse.move(mid.x, mid.y);
  await page.mouse.down();
  await page.mouse.move(target.x, target.y, { steps: 6 });
  await expectDotAt(page, target);
  await page.mouse.up();
  await page.waitForTimeout(200);
  expect(await dotColourAt(page, target)).toBe("green");
  expect(await dotColourAt(page, mid)).toBeNull();
  await shot(page, "placement-dragged");

  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expectNodeDrawnNear(page, target, 4);
  expect(await nodeSamplesAround(page, mid)).toBeLessThan(6);
});

test("a curve with no room for a node is rejected when it locks", async ({ page }) => {
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const node = nodeAt(frame, fitted, 0);
  const zoomed = await wheelZoomTo(page, frame, fitted, node, 4);
  // A loop around the node small enough that every point of it is within
  // the node spacing, so no candidate on the curve validates.
  const radius = 7 * (frame.board / 1000) * zoomed.scale;
  const centre = { x: node.x + radius, y: node.y };
  await page.mouse.move(node.x, node.y);
  await page.mouse.down();
  for (let i = 1; i <= 24; i++) {
    const angle = Math.PI + (2 * Math.PI * i) / 24;
    await page.mouse.move(centre.x + radius * Math.cos(angle), centre.y + radius * Math.sin(angle));
  }
  await page.mouse.up();

  const farSide = { x: node.x + 2 * radius, y: node.y };
  await expect(page.locator("#status")).toHaveText(NO_ROOM);
  await expect.poll(() => redPixelsNear(page, farSide, 24)).toBeGreaterThan(0);
  await shot(page, "placement-no-room");
  await expect(page.locator("#cancelBtn")).toBeHidden();
  await expect(page.locator("#undoBtn")).toBeDisabled();
  await page.waitForTimeout(800);
  expect(await redPixelsNear(page, farSide, 24)).toBe(0);
});

test("Escape and Cancel each abort, and Undo and New Game still work", async ({ page }) => {
  await playExchange(page);
  await drawBetween(page, 2, 3);
  await page.locator("#cancelBtn").click();
  await expect(page.locator("#status")).toHaveText("Move cancelled");
  await expect(page.locator("#cancelBtn")).toBeHidden();
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");

  await drawBetween(page, 0, 1);
  await page.keyboard.press("Escape");
  await expect(page.locator("#status")).toHaveText("Move cancelled");
  await expect(page.locator("#cancelBtn")).toBeHidden();
  await startGame(page, 4);
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  await expect(page.locator("#undoBtn")).toBeDisabled();
});

test("a touch drag slides the dot under the loupe and a tap elsewhere places it there", async ({
  page,
}) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
  const target = await alongFirstChord(page, 0.3);
  const cdp = await page.context().newCDPSession(page);
  await touchStroke(cdp, a, b);
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await expectDotAt(page, mid);

  await cdp.send("Input.dispatchTouchEvent", { type: "touchStart", touchPoints: [mid] });
  await touchDrag(cdp, mid, target);
  await expectDotAt(page, target);
  await expect.poll(() => ringPixels(page)).toBeGreaterThan(0);
  await shot(page, "placement-touch-drag");
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
  await expect.poll(() => ringPixels(page)).toBe(0);
  expect(await dotColourAt(page, target)).toBe("green");
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);

  await tapAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expectNodeDrawnNear(page, target, 4);
});

test("a pinch during the drag cancels it and leaves the dot where it was", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const a = nodeAt(frame, fitted, 0);
  const b = nodeAt(frame, fitted, 1);
  const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
  const target = await alongFirstChord(page, 0.7);
  const cdp = await page.context().newCDPSession(page);
  await touchStroke(cdp, a, b);
  await expectDotAt(page, mid);

  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...mid, id: 1 }],
  });
  await touchDrag(cdp, mid, target, 1);
  await expectDotAt(page, target);

  const second = { x: target.x, y: target.y + 50 };
  const secondTo = { x: target.x, y: target.y + 100 };
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [
      { ...target, id: 1 },
      { ...second, id: 2 },
    ],
  });
  await pinchDrag(cdp, [target, second], [target, secondTo]);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });

  const t = clampToViewport(frame, pinch(frame, fitted, [target, second], [target, secondTo]));
  const n0 = nodeWorld(0, 4, frame.board);
  const n1 = nodeWorld(1, 4, frame.board);
  const world = { x: n0.x + (n1.x - n0.x) * 0.7, y: n0.y + (n1.y - n0.y) * 0.7 };
  const moved = toClient(frame, t, world);
  await expectNodeDrawnAt(page, nodeAt(frame, t, 1));
  await expectDotAt(page, moved);
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await expect(page.locator("#cancelBtn")).toBeVisible();
  await shot(page, "placement-pinch-kept");

  await tapAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expectNodeDrawnNear(page, moved, 4);
});

test("Escape while the fit is pending returns to idle", async ({ page }) => {
  await holdNext(page, "fit", 600);
  await drawBetween(page, 0, 1);
  await page.keyboard.press("Escape");
  await expect(page.locator("#status")).toHaveText("Move cancelled");
  await expect(page.locator("#cancelBtn")).toBeHidden();
  await page.waitForTimeout(900);
  await expect(page.locator("#status")).toHaveText("Move cancelled");
  expect(await findDot(page)).toBeNull();
});

test("Enter while the fit is pending still places the node", async ({ page }) => {
  await holdNext(page, "fit", 600);
  await drawBetween(page, 0, 1);
  await page.keyboard.press("Enter");
  await expect(page.locator("#status")).toHaveText(/AI is thinking|Your turn/, { timeout: 60_000 });
  await expect(page.locator("#undoBtn")).toBeEnabled({ timeout: 60_000 });
});

test("a stroke the engine cannot fit returns to idle with Cancel hidden", async ({ page }) => {
  await failNext(page, "fit", "fit failed");
  // The error lands right after the lock, so the placing hint is not awaited.
  await mouseStroke(page, await nodeOnScreen(page, 0, 4), await nodeOnScreen(page, 1, 4));
  await expect(page.locator("#status")).toHaveText("Could not fit the stroke: fit failed");
  await expect(page.locator("#cancelBtn")).toBeHidden();
  expect(await findDot(page)).toBeNull();
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
});

test("a stroke that leaves the board gets its dot on the board", async ({ page }) => {
  const frame = await frameOf(page);
  const a = await nodeOnScreen(page, 0, 4);
  const b = await nodeOnScreen(page, 1, 4);
  // Out through the top edge, which has room above the canvas on every
  // project, and far enough that the inset candidates are clear of it.
  const via = { x: a.x + 30, y: frame.top - 40 };
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(via.x, via.y, { steps: 15 });
  await page.mouse.move(b.x, b.y, { steps: 15 });
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await expect.poll(() => findDot(page)).not.toBeNull();
  const dot = (await findDot(page))!;
  // MIN_NODE_SPACING in src/types.rs, scaled to the board and the screen.
  const margin = 20 * (frame.board / 1000) * fit(frame).scale - 1;
  expect(dot.x).toBeGreaterThanOrEqual(frame.left + margin);
  expect(dot.x).toBeLessThanOrEqual(frame.left + frame.width - margin);
  expect(dot.y).toBeGreaterThanOrEqual(frame.top + margin);
  expect(dot.y).toBeLessThanOrEqual(frame.top + frame.height - margin);
  expect(Math.hypot(dot.x - via.x, dot.y - via.y)).toBeGreaterThan(20);
  await shot(page, "placement-detour");
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
});

test("Cancel and Escape do nothing while the move is being applied", async ({ page }) => {
  await holdNext(page, "applyMove", 1000);
  const mid = await drawBetween(page, 0, 1);
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#cancelBtn")).toBeHidden();
  await page.keyboard.press("Escape");
  await page.waitForTimeout(300);
  await expect(page.locator("#status")).not.toHaveText("Move cancelled");
  await expect(page.locator("#status")).toHaveText("AI is thinking", { timeout: 10_000 });
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
});

test("a pinch that starts away from the dot zooms without placing the node", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const a = nodeAt(frame, fitted, 0);
  const b = nodeAt(frame, fitted, 1);
  const cdp = await page.context().newCDPSession(page);
  await touchStroke(cdp, a, b);
  await expectDotAt(page, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });

  // The fingers straddle the dot 30 px out, past its grab radius, so the
  // zoom is about the dot and it stays put; node 0 is the nearer end of
  // the chord and stays on the canvas at one and a half times.
  const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
  const first = { x: mid.x - 30, y: mid.y };
  const second = { x: mid.x + 30, y: mid.y };
  const secondTo = { x: mid.x + 60, y: mid.y };
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...first, id: 1 }],
  });
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [
      { ...first, id: 1 },
      { ...second, id: 2 },
    ],
  });
  await pinchDrag(cdp, [first, second], [first, secondTo]);
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchEnd",
    touchPoints: [{ ...first, id: 1 }],
  });
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });

  const t = clampToViewport(frame, pinch(frame, fitted, [first, second], [first, secondTo]));
  const n0 = nodeWorld(0, 4, frame.board);
  const n1 = nodeWorld(1, 4, frame.board);
  const midWorld = { x: (n0.x + n1.x) / 2, y: (n0.y + n1.y) / 2 };
  await expectNodeDrawnAt(page, nodeAt(frame, t, 0));
  await expectDotAt(page, toClient(frame, t, midWorld));
  await page.waitForTimeout(500);
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  await expect(page.locator("#cancelBtn")).toBeVisible();
  await shot(page, "placement-pinch-away");
});
