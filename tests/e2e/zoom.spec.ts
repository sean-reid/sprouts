import { expect, test } from "@playwright/test";
import {
  ctrlWheel,
  expectNodeDrawnAt,
  fit,
  frameOf,
  mouseStroke,
  nodeAt,
  openBoard,
  pinch,
  plainWheel,
  shot,
  touchDrag,
  touchStroke,
  zoomAt,
  type Client,
} from "./helpers";

const status = (page: Parameters<typeof openBoard>[0]) => page.locator("#status");

test.beforeEach(async ({ page }) => {
  await openBoard(page);
});

test("ctrl+wheel zooms at a node and a stroke from it still locks a line", async ({ page }) => {
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const a = nodeAt(frame, fitted, 0);
  const delta = await ctrlWheel(page, a, -100);
  const zoomed = zoomAt(frame, fitted, a, 2 ** (-delta / 100));
  const b = nodeAt(frame, zoomed, 1);
  await expectNodeDrawnAt(page, a);
  await expectNodeDrawnAt(page, b);
  await shot(page, "zoom-wheel");

  await mouseStroke(page, a, b);
  await expect(status(page)).toHaveText(/place a new node/);
  await shot(page, "zoom-wheel-locked");

  await page.keyboard.press("Escape");
  await page.keyboard.press("0");
  await expectNodeDrawnAt(page, nodeAt(frame, fitted, 1));
});

test("plain wheel and space drag pan the zoomed board", async ({ page }) => {
  const frame = await frameOf(page);
  const centre = { x: frame.left + frame.width / 2, y: frame.top + frame.height / 2 };
  const delta = await ctrlWheel(page, centre, -100);
  let t = zoomAt(frame, fit(frame), centre, 2 ** (-delta / 100));
  await expectNodeDrawnAt(page, nodeAt(frame, t, 2));

  const panned = await plainWheel(page, 40);
  t = { ...t, ty: t.ty - panned };
  await expectNodeDrawnAt(page, nodeAt(frame, t, 2));

  await page.keyboard.down("Space");
  await page.mouse.move(centre.x, centre.y);
  await page.mouse.down();
  await page.mouse.move(centre.x + 30, centre.y + 20, { steps: 5 });
  await page.mouse.up();
  await page.keyboard.up("Space");
  t = { ...t, tx: t.tx + 30, ty: t.ty + 20 };
  await expectNodeDrawnAt(page, nodeAt(frame, t, 2));
  await expect(status(page)).toHaveText("Draw a line between two nodes");
  await shot(page, "zoom-panned");
});

test("pinch zooms and a touch stroke still locks a line", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const centre = { x: frame.left + frame.width / 2, y: frame.top + frame.height / 2 };
  const from: [Client, Client] = [
    { x: centre.x - 30, y: centre.y },
    { x: centre.x + 30, y: centre.y },
  ];
  const to: [Client, Client] = [
    { x: centre.x - 75, y: centre.y + 10 },
    { x: centre.x + 75, y: centre.y + 10 },
  ];
  const cdp = await page.context().newCDPSession(page);
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [
      { ...from[0], id: 1 },
      { ...from[1], id: 2 },
    ],
  });
  for (let i = 1; i <= 10; i++) {
    const s = i / 10;
    await cdp.send("Input.dispatchTouchEvent", {
      type: "touchMove",
      touchPoints: [
        {
          x: from[0].x + (to[0].x - from[0].x) * s,
          y: from[0].y + (to[0].y - from[0].y) * s,
          id: 1,
        },
        {
          x: from[1].x + (to[1].x - from[1].x) * s,
          y: from[1].y + (to[1].y - from[1].y) * s,
          id: 2,
        },
      ],
    });
  }
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });

  const t = pinch(frame, fit(frame), from, to);
  const a = nodeAt(frame, t, 0);
  const b = nodeAt(frame, t, 1);
  await expectNodeDrawnAt(page, a);
  await expectNodeDrawnAt(page, b);
  await expect(status(page)).toHaveText("Draw a line between two nodes");
  await shot(page, "zoom-pinch");

  await touchStroke(cdp, a, b);
  await expect(status(page)).toHaveText(/Tap Confirm/);
  await expect(page.locator("#mobileOverlay")).toBeVisible();
  await shot(page, "zoom-pinch-locked");
});

test("double tap zooms in at a node and double tap again fits", async ({ page }) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const a = nodeAt(frame, fitted, 0);
  await page.touchscreen.tap(a.x, a.y);
  await page.touchscreen.tap(a.x, a.y);
  const zoomed = zoomAt(frame, fitted, a, 2.5);
  await expectNodeDrawnAt(page, a);
  await expectNodeDrawnAt(page, nodeAt(frame, zoomed, 1));
  await expect(status(page)).toHaveText("Draw a line between two nodes");
  await shot(page, "zoom-double-tap");

  await page.waitForTimeout(400);
  await page.touchscreen.tap(a.x, a.y);
  await page.touchscreen.tap(a.x, a.y);
  await expectNodeDrawnAt(page, nodeAt(frame, fitted, 1));
  await expect(page.locator("#mobileOverlay")).toBeHidden();
});

test("the backing store follows devicePixelRatio", async ({ page }) => {
  const backing = await page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const rect = canvas.getBoundingClientRect();
    return { width: canvas.width, css: rect.width, dpr: window.devicePixelRatio };
  });
  expect(backing.width).toBe(Math.round(backing.css * backing.dpr));
  if (test.info().project.name === "mobile") expect(backing.dpr).toBe(2);
});

test("a click on a node without movement changes nothing", async ({ page }) => {
  const frame = await frameOf(page);
  const a = nodeAt(frame, fit(frame), 0);
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.up();
  await page.waitForTimeout(300);
  await expect(status(page)).toHaveText("Draw a line between two nodes");
  await expect(page.locator("#mobileOverlay")).toBeHidden();
  await shot(page, "click-no-move");

  await mouseStroke(page, a, nodeAt(frame, fit(frame), 1));
  await expect(status(page)).toHaveText(/place a new node/);
});

test("a pinch mid-stroke pauses the stroke and it finishes after one finger lifts", async ({
  page,
}) => {
  test.skip(!test.info().project.use.hasTouch, "touch only");
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const a = nodeAt(frame, fitted, 0);
  const b = nodeAt(frame, fitted, 1);
  const rest = { x: a.x + (b.x - a.x) * 0.4, y: a.y + (b.y - a.y) * 0.4 };
  const cdp = await page.context().newCDPSession(page);
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [{ ...a, id: 1 }],
  });
  await touchDrag(cdp, a, rest, 1, 8);

  const second = { x: rest.x, y: rest.y + 50 };
  const secondTo = { x: rest.x, y: rest.y + 100 };
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchStart",
    touchPoints: [
      { ...rest, id: 1 },
      { ...second, id: 2 },
    ],
  });
  for (let i = 1; i <= 10; i++) {
    const s = i / 10;
    await cdp.send("Input.dispatchTouchEvent", {
      type: "touchMove",
      touchPoints: [
        { ...rest, id: 1 },
        { x: second.x, y: second.y + (secondTo.y - second.y) * s, id: 2 },
      ],
    });
  }
  await cdp.send("Input.dispatchTouchEvent", {
    type: "touchEnd",
    touchPoints: [{ ...secondTo, id: 2 }],
  });

  const t = pinch(frame, fitted, [rest, second], [rest, secondTo]);
  const bZoomed = nodeAt(frame, t, 1);
  await expectNodeDrawnAt(page, bZoomed);
  await shot(page, "zoom-mid-stroke");

  await touchDrag(cdp, rest, bZoomed, 1);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
  await expect(status(page)).toHaveText(/Tap Confirm/);
  await shot(page, "zoom-mid-stroke-locked");
});
