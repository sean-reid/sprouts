import { expect, test, type Page } from "@playwright/test";
import { openBoard, playExchange, shot } from "./helpers";

interface Rect {
  left: number;
  top: number;
  right: number;
  bottom: number;
}

interface Layout {
  canvas: Rect;
  panel: Rect;
  controls: Rect;
  tops: number[];
}

function layoutOf(page: Page): Promise<Layout> {
  return page.evaluate(() => {
    const rect = (el: Element): Rect => {
      const { left, top, right, bottom } = el.getBoundingClientRect();
      return { left, top, right, bottom };
    };
    const rows = [...document.querySelectorAll(".controls")].map(rect);
    const items = ["#undoBtn", ".toggle", "#turn"].map((s) => rect(document.querySelector(s)!));
    return {
      canvas: rect(document.getElementById("canvas")!),
      panel: rect(document.getElementById("history")!),
      controls: {
        left: Math.min(...rows.map((r) => r.left)),
        top: Math.min(...rows.map((r) => r.top)),
        right: Math.max(...rows.map((r) => r.right)),
        bottom: Math.max(...rows.map((r) => r.bottom)),
      },
      tops: [...rows, ...items].map((r) => r.top),
    };
  });
}

async function expectSideBySide(page: Page, label: string): Promise<void> {
  const before = await layoutOf(page);
  const gap = before.panel.left - before.canvas.right;
  expect(gap).toBeGreaterThanOrEqual(0);
  expect(gap).toBeLessThanOrEqual(24);
  const groupMid = (before.canvas.left + before.panel.right) / 2;
  const controlsMid = (before.controls.left + before.controls.right) / 2;
  expect(Math.abs(groupMid - controlsMid)).toBeLessThanOrEqual(4);
  for (const top of before.tops) expect(Math.abs(top - before.tops[0]!)).toBeLessThan(1);
  expect(before.canvas.right - before.canvas.left).toBeCloseTo(
    before.canvas.bottom - before.canvas.top,
    0,
  );
  await shot(page, `layout-idle-${label}`);

  await playExchange(page);
  await expect(page.locator("#history")).toBeVisible();
  const after = await layoutOf(page);
  expect(after.canvas).toEqual(before.canvas);
  await shot(page, `layout-played-${label}`);
}

test("the board and move list sit centred under one row of controls", async ({ page }) => {
  test.skip(test.info().project.name !== "desktop", "the side-by-side layout starts at 1024 wide");
  await openBoard(page);
  await expectSideBySide(page, "1024");

  await page.setViewportSize({ width: 1600, height: 900 });
  await page.locator("button[data-nodes='4']").click();
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
  await expectSideBySide(page, "1600");
});
