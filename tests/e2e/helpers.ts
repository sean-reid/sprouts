import { expect, test, type CDPSession, type Page } from "@playwright/test";

export interface Client {
  x: number;
  y: number;
}

export interface Frame {
  left: number;
  top: number;
  width: number;
  height: number;
  board: number;
}

// Canvas-local CSS pixels per world unit plus the offset of world origin.
export interface Transform {
  scale: number;
  tx: number;
  ty: number;
}

export function frameOf(page: Page): Promise<Frame> {
  return page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const rect = canvas.getBoundingClientRect();
    const board = window.innerWidth <= 768 || window.innerHeight <= 768 ? 500 : 1000;
    return { left: rect.left, top: rect.top, width: rect.width, height: rect.height, board };
  });
}

export function fit(frame: Frame): Transform {
  const scale = Math.min(frame.width, frame.height) / frame.board;
  return {
    scale,
    tx: (frame.width - frame.board * scale) / 2,
    ty: (frame.height - frame.board * scale) / 2,
  };
}

// Scales about a client point so the world under it stays put.
export function zoomAt(frame: Frame, t: Transform, at: Client, factor: number): Transform {
  const local = { x: at.x - frame.left, y: at.y - frame.top };
  const world = { x: (local.x - t.tx) / t.scale, y: (local.y - t.ty) / t.scale };
  const scale = t.scale * factor;
  return { scale, tx: local.x - world.x * scale, ty: local.y - world.y * scale };
}

// Two fingers moving from one pair of client points to another.
export function pinch(
  frame: Frame,
  t: Transform,
  from: [Client, Client],
  to: [Client, Client],
): Transform {
  const mid = (pair: [Client, Client]): Client => ({
    x: (pair[0].x + pair[1].x) / 2,
    y: (pair[0].y + pair[1].y) / 2,
  });
  const span = (pair: [Client, Client]): number =>
    Math.hypot(pair[0].x - pair[1].x, pair[0].y - pair[1].y);
  const m0 = mid(from);
  const m1 = mid(to);
  const zoomed = zoomAt(frame, t, m0, span(to) / span(from));
  return { ...zoomed, tx: zoomed.tx + (m1.x - m0.x), ty: zoomed.ty + (m1.y - m0.y) };
}

// Mirrors the opening layout in src/types.rs: nodes on a circle around the
// board centre, radius scaled to the board size.
export function nodeWorld(index: number, count: number, board: number): Client {
  const radius = (count <= 4 ? 120 : 160) * (board / 1000);
  const angle = (2 * Math.PI * index) / count;
  return { x: board / 2 + radius * Math.cos(angle), y: board / 2 + radius * Math.sin(angle) };
}

export function toClient(frame: Frame, t: Transform, world: Client): Client {
  return { x: frame.left + t.tx + world.x * t.scale, y: frame.top + t.ty + world.y * t.scale };
}

export function nodeAt(frame: Frame, t: Transform, index: number, count = 4): Client {
  return toClient(frame, t, nodeWorld(index, count, frame.board));
}

export async function nodeOnScreen(page: Page, index: number, count: number): Promise<Client> {
  const frame = await frameOf(page);
  return nodeAt(frame, fit(frame), index, count);
}

// Counts node-blue samples on a ring 5 px out from a client point, which
// clears the white label and sits inside the 8 px disc.
export function blueSamplesAround(page: Page, at: Client): Promise<number> {
  return page.evaluate(
    ([x, y]) => {
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d");
      if (!ctx) return 0;
      const rect = canvas.getBoundingClientRect();
      const dpr = canvas.width / rect.width;
      let blue = 0;
      for (let k = 0; k < 8; k++) {
        const angle = (Math.PI * 2 * k) / 8;
        const px = Math.round((x - rect.left + 5 * Math.cos(angle)) * dpr);
        const py = Math.round((y - rect.top + 5 * Math.sin(angle)) * dpr);
        const [r, , b] = ctx.getImageData(px, py, 1, 1).data;
        if (r !== undefined && b !== undefined && r < 100 && b > 200) blue++;
      }
      return blue;
    },
    [at.x, at.y] as const,
  );
}

export async function expectNodeDrawnAt(page: Page, at: Client): Promise<void> {
  await expect.poll(() => blueSamplesAround(page, at)).toBeGreaterThanOrEqual(6);
}

export async function drawBetween(page: Page, from: number, to: number): Promise<Client> {
  const a = await nodeOnScreen(page, from, 4);
  const b = await nodeOnScreen(page, to, 4);
  await mouseStroke(page, a, b);
  await expect(page.locator("#status")).toHaveText(/place a new node/);
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

export async function mouseStroke(page: Page, a: Client, b: Client): Promise<void> {
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(b.x, b.y, { steps: 20 });
  await page.mouse.up();
}

export async function touchStroke(cdp: CDPSession, a: Client, b: Client): Promise<void> {
  await cdp.send("Input.dispatchTouchEvent", { type: "touchStart", touchPoints: [a] });
  await touchDrag(cdp, a, b);
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
}

export async function touchDrag(
  cdp: CDPSession,
  a: Client,
  b: Client,
  id = 0,
  steps = 20,
): Promise<void> {
  for (let i = 1; i <= steps; i++) {
    const t = i / steps;
    await cdp.send("Input.dispatchTouchEvent", {
      type: "touchMove",
      touchPoints: [{ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, id }],
    });
  }
}

// Two fingers, ids 1 and 2, moving from one pair of client points to another.
export async function pinchDrag(
  cdp: CDPSession,
  from: [Client, Client],
  to: [Client, Client],
  steps = 10,
): Promise<void> {
  for (let i = 1; i <= steps; i++) {
    const s = i / steps;
    await cdp.send("Input.dispatchTouchEvent", {
      type: "touchMove",
      touchPoints: from.map((f, k) => ({
        x: f.x + (to[k]!.x - f.x) * s,
        y: f.y + (to[k]!.y - f.y) * s,
        id: k + 1,
      })),
    });
  }
}

// Mobile emulation rescales wheel deltas, so the test reads what the page got.
export async function ctrlWheel(page: Page, at: Client, deltaY: number): Promise<number> {
  await page.mouse.move(at.x, at.y);
  const delivered = page.evaluate(
    () =>
      new Promise<number>((resolve) => {
        const canvas = document.getElementById("canvas") as HTMLCanvasElement;
        canvas.addEventListener("wheel", (e) => resolve(e.deltaY), { once: true });
      }),
  );
  await page.keyboard.down("Control");
  await page.mouse.wheel(0, deltaY);
  await page.keyboard.up("Control");
  return delivered;
}

export async function plainWheel(page: Page, deltaY: number): Promise<number> {
  const delivered = page.evaluate(
    () =>
      new Promise<number>((resolve) => {
        const canvas = document.getElementById("canvas") as HTMLCanvasElement;
        canvas.addEventListener("wheel", (e) => resolve(e.deltaY), { once: true });
      }),
  );
  await page.mouse.wheel(0, deltaY);
  return delivered;
}

// Installed before the page loads so a test can hold one request to the
// worker back; the worker then answers a newer request first.
export function holdOneWorkerMessage(): void {
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

export function holdNext(page: Page, method: string, ms: number): Promise<void> {
  return page.evaluate(
    ([m, delay]) => {
      (window as unknown as { holdNext: (m: string, ms: number) => void }).holdNext(m, delay);
    },
    [method, ms] as const,
  );
}

export type Rgb = [number, number, number];

export function pixelAt(page: Page, at: Client): Promise<Rgb> {
  return page.evaluate(({ x, y }) => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const rect = canvas.getBoundingClientRect();
    const px = Math.round((x - rect.left) * (canvas.width / rect.width));
    const py = Math.round((y - rect.top) * (canvas.height / rect.height));
    const [r, g, b] = canvas.getContext("2d")!.getImageData(px, py, 1, 1).data;
    return [r!, g!, b!];
  }, at);
}

export function near(pixel: Rgb, hex: string, tolerance = 40): boolean {
  const want = [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
  return Math.hypot(pixel[0] - want[0]!, pixel[1] - want[1]!, pixel[2] - want[2]!) < tolerance;
}

// The 4 px turn ring starts at the canvas edge; 2 px in is its centre line.
export function ringPixel(frame: Frame): Client {
  return { x: frame.left + 2, y: frame.top + frame.height / 2 };
}

export function shot(page: Page, name: string) {
  return page.screenshot({
    path: `test-results/shots/${test.info().project.name}-${name}.png`,
    fullPage: true,
  });
}

export async function openBoard(page: Page, path = "/"): Promise<void> {
  await page.goto(path);
  // Fetching the wasm and building the 1000 grid takes a few seconds on a shared runner.
  await expect(page.locator("#status")).toHaveText("Draw a line between two nodes", {
    timeout: 20_000,
  });
}

// One human move and the AI's reply, with the mouse.
export async function playExchange(page: Page): Promise<void> {
  const mid = await drawBetween(page, 0, 1);
  await page.waitForTimeout(350);
  await page.mouse.click(mid.x, mid.y);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
}
