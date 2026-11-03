import { expect, test, type Page } from "@playwright/test";

interface Client {
  x: number;
  y: number;
}

// Mirrors the opening layout in src/types.rs: nodes on a circle around the
// board centre, radius scaled to the board size.
function nodeOnScreen(page: Page, index: number, count: number): Promise<Client> {
  return page.evaluate(
    ([i, n]) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const rect = canvas.getBoundingClientRect();
      const board = canvas.width;
      const radius = (n <= 4 ? 120 : 160) * (board / 1000);
      const angle = (2 * Math.PI * i) / n;
      const x = board / 2 + radius * Math.cos(angle);
      const y = board / 2 + radius * Math.sin(angle);
      return {
        x: rect.left + (x / board) * rect.width,
        y: rect.top + (y / board) * rect.height,
      };
    },
    [index, count] as const,
  );
}

async function drawBetween(page: Page, from: number, to: number): Promise<Client> {
  const a = await nodeOnScreen(page, from, 4);
  const b = await nodeOnScreen(page, to, 4);
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(b.x, b.y, { steps: 20 });
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(/place a new node/);
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

function shot(page: Page, name: string) {
  return page.screenshot({
    path: `test-results/shots/${test.info().project.name}-${name}.png`,
    fullPage: true,
  });
}

test.beforeEach(async ({ page }) => {
  await page.goto("/");
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes");
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
  const mid = await drawBetween(page, 0, 1);
  await page.waitForTimeout(350);
  await page.mouse.click(mid.x, mid.y);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await expect(page.locator("#undoBtn")).toBeEnabled();
  await shot(page, "after-first-exchange");
  await page.locator("#undoBtn").click();
  await expect(page.locator("#status")).toHaveText("Move undone: your turn");
  await expect(page.locator("#undoBtn")).toBeDisabled();
});
