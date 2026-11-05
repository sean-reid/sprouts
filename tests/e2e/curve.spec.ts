import { expect, test, type Page } from "@playwright/test";
import { hasStroke, LINK_QUANTUM, type RecordMove } from "../../web/history";
import {
  clickAway,
  findDot,
  fit,
  frameOf,
  holdNext,
  interceptWorkerMessages,
  latestCurve,
  nodeWorld,
  openBoard,
  PLACING_HINT,
  shot,
  toClient,
  wheelZoomTo,
  type Client,
  type Frame,
  type Transform,
} from "./helpers";

interface Exclusion extends Client {
  r: number;
}

// The turn ring hugs the canvas edge and changes colour with the turn.
const RING_MARGIN_CSS = 6;

// Marks every blue pixel of the canvas (the locked stroke, a committed human
// line and the node discs all read as blue; the board and the AI's red do
// not) outside the exclusions and the ring, keeps the mask on the window
// under `key`, and returns how many pixels it marked.
function captureBlue(page: Page, key: string, exclude: Exclusion[]): Promise<number> {
  return page.evaluate(
    async ({ key, exclude, margin }) => {
      await new Promise<void>((resolve) => {
        requestAnimationFrame(() => requestAnimationFrame(() => resolve()));
      });
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d");
      if (!ctx) return 0;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      const { data, width, height } = ctx.getImageData(0, 0, canvas.width, canvas.height);
      const mask = new Uint8Array(width * height);
      let count = 0;
      const band = Math.ceil(margin * dpr);
      for (let i = 0; i < width * height; i++) {
        const col = i % width;
        const row = Math.floor(i / width);
        if (col < band || row < band || col >= width - band || row >= height - band) continue;
        const r = data[i * 4] ?? 0;
        const g = data[i * 4 + 1] ?? 0;
        const b = data[i * 4 + 2] ?? 0;
        if (!(b > 200 && r < 130 && g > 90 && g < 200)) continue;
        const x = (i % width) / dpr + rect.left;
        const y = Math.floor(i / width) / dpr + rect.top;
        if (exclude.some((e) => Math.hypot(x - e.x, y - e.y) < e.r)) continue;
        mask[i] = 1;
        count++;
      }
      const store = window as unknown as Record<string, { mask: Uint8Array; width: number }>;
      store[key] = { mask, width };
      return count;
    },
    { key, exclude, margin: RING_MARGIN_CSS },
  );
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(interceptWorkerMessages);
  await openBoard(page);
});

interface Arc {
  frame: Frame;
  fitted: Transform;
  along: (t: number) => Client;
}

// Draws an arc from node 0 through a bend to node 1 and leaves it locked.
async function lockArc(page: Page): Promise<Arc> {
  const frame = await frameOf(page);
  const fitted = fit(frame);
  const a = nodeWorld(0, 4, frame.board);
  const b = nodeWorld(1, 4, frame.board);
  const bend = { x: 0.61 * frame.board, y: 0.61 * frame.board };
  const along = (t: number): Client => ({
    x: (1 - t) ** 2 * a.x + 2 * t * (1 - t) * bend.x + t * t * b.x,
    y: (1 - t) ** 2 * a.y + 2 * t * (1 - t) * bend.y + t * t * b.y,
  });

  const start = toClient(frame, fitted, a);
  await page.mouse.move(start.x, start.y);
  await page.mouse.down();
  for (let i = 1; i <= 24; i++) {
    const at = toClient(frame, fitted, along(i / 24));
    await page.mouse.move(at.x, at.y);
  }
  await page.mouse.up();
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  return { frame, fitted, along };
}

test("a click that lands before the fit returns still places the node", async ({ page }) => {
  await holdNext(page, "fit", 600);
  const { frame, fitted, along } = await lockArc(page);
  const mid = toClient(frame, fitted, along(0.5));
  await page.mouse.click(mid.x, mid.y);
  await expect(page.locator("#status")).toHaveText(/AI is thinking|Your turn/, { timeout: 60_000 });
  await expect(page.locator("#undoBtn")).toBeEnabled({ timeout: 60_000 });
});

// The board settles once the move is in, so the committed line moves; the
// curve the page showed locked is the one the game records.
test("the locked curve is the curve the move is recorded with, at fit zoom and at 4x", async ({
  page,
}) => {
  const { frame, fitted, along } = await lockArc(page);
  await page.waitForTimeout(400);
  await shot(page, "curve-locked-fit");
  const shown = await latestCurve(page);
  expect(shown.length).toBeGreaterThan(0);

  const mid = along(0.5);
  const zoomed = await wheelZoomTo(page, frame, fitted, toClient(frame, fitted, mid), 4);
  expect(zoomed.scale / fitted.scale).toBeGreaterThanOrEqual(4);
  await page.waitForTimeout(200);
  await shot(page, "curve-locked-4x");

  // The dot, and the node it becomes, are the one thing that differs.
  const dot = await findDot(page);
  expect(dot).not.toBeNull();
  const exclude = [{ ...dot!, r: 16 }];
  const locked = await captureBlue(page, "locked", exclude);
  expect(locked).toBeGreaterThan(500);

  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
  await shot(page, "curve-committed-4x");

  // The record keeps the committed spline, which is the locked curve put
  // on the unit grid.
  const saved = await page.evaluate(
    () => JSON.parse(localStorage.getItem("sprouts.game") ?? "null") as unknown,
  );
  const first = (saved as { moves: RecordMove[] } | null)?.moves[0];
  if (!first || !hasStroke(first) || !first.segments) {
    throw new Error("the first move was not saved with its spline");
  }
  expect(first.segments).toHaveLength(shown.length);
  const half = LINK_QUANTUM / 2 + 1e-9;
  first.segments.forEach((c, i) => {
    for (const k of ["p0", "p1", "p2", "p3"] as const) {
      expect(Math.abs(c[k].x - shown[i]![k].x)).toBeLessThanOrEqual(half);
      expect(Math.abs(c[k].y - shown[i]![k].y)).toBeLessThanOrEqual(half);
    }
  });
});
