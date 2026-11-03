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

export const PLACING_HINT = /Drag the dot/;

// Locks a straight stroke between two nodes and returns its midpoint, where
// the dot lands on an empty board.
export async function drawBetween(page: Page, from: number, to: number): Promise<Client> {
  const a = await nodeOnScreen(page, from, 4);
  const b = await nodeOnScreen(page, to, 4);
  await mouseStroke(page, a, b);
  await expect(page.locator("#status")).toHaveText(PLACING_HINT);
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

export type DotColour = "green" | "red";

// Reads the canvas pixel under a client point and says which placement dot
// colour it is, or null for anything else.
export function dotColourAt(page: Page, at: Client): Promise<DotColour | null> {
  return page.evaluate(({ x, y }) => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const rect = canvas.getBoundingClientRect();
    const px = Math.round((x - rect.left) * (canvas.width / rect.width));
    const py = Math.round((y - rect.top) * (canvas.height / rect.height));
    const [r, g, b] = canvas.getContext("2d")!.getImageData(px, py, 1, 1).data;
    const near = (cr: number, cg: number, cb: number) => Math.hypot(r! - cr, g! - cg, b! - cb) < 60;
    if (near(0x10, 0xb9, 0x81)) return "green";
    if (near(0xef, 0x44, 0x44)) return "red";
    return null;
  }, at);
}

export async function expectDotAt(page: Page, at: Client, colour: DotColour = "green") {
  await expect.poll(() => dotColourAt(page, at)).toBe(colour);
}

// A spot in the canvas corner, clear of the nodes and of any dot on the
// first chord, where a press commits the placement.
export function awayFrom(frame: Frame): Client {
  return { x: frame.left + 24, y: frame.top + 24 };
}

export async function clickAway(page: Page): Promise<void> {
  const at = awayFrom(await frameOf(page));
  await page.mouse.click(at.x, at.y);
}

export async function tapAway(page: Page): Promise<void> {
  const at = awayFrom(await frameOf(page));
  await page.touchscreen.tap(at.x, at.y);
}

// Counts loupe-ring amber pixels anywhere on the canvas.
export function ringPixels(page: Page): Promise<number> {
  return page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const ctx = canvas.getContext("2d");
    if (!ctx) return 0;
    const { data } = ctx.getImageData(0, 0, canvas.width, canvas.height);
    let hits = 0;
    for (let i = 0; i < data.length; i += 4) {
      const r = data[i] ?? 0;
      const g = data[i + 1] ?? 0;
      const b = data[i + 2] ?? 0;
      if (r > 200 && g > 150 && g < 220 && b < 100) hits++;
    }
    return hits;
  });
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
// worker back, so the worker answers a newer request first, or fail one
// with an error reply the worker never saw.
export function interceptWorkerMessages(): void {
  const post = Worker.prototype.postMessage;
  let hold: { method: string; ms: number } | null = null;
  let fail: { method: string; error: string } | null = null;
  const hooks = window as unknown as { holdNext: typeof holdNext; failNext: typeof failNext };
  hooks.holdNext = holdNext;
  hooks.failNext = failNext;
  function holdNext(method: string, ms: number): void {
    hold = { method, ms };
  }
  function failNext(method: string, error: string): void {
    fail = { method, error };
  }
  Worker.prototype.postMessage = function (this: Worker, ...args: unknown[]) {
    const envelope = args[0] as { id?: number; method?: string } | undefined;
    if (fail && envelope?.method === fail.method) {
      const data = { id: envelope.id, ok: false, error: fail.error };
      fail = null;
      setTimeout(() => this.dispatchEvent(new MessageEvent("message", { data })));
      return;
    }
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

export function failNext(page: Page, method: string, error: string): Promise<void> {
  return page.evaluate(
    ([m, e]) => {
      (window as unknown as { failNext: (m: string, e: string) => void }).failNext(m, e);
    },
    [method, error] as const,
  );
}

// Client position of the placement dot, from the centroid of its green
// pixels, or null when none is drawn.
export function findDot(page: Page): Promise<Client | null> {
  return page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const ctx = canvas.getContext("2d");
    if (!ctx) return null;
    const rect = canvas.getBoundingClientRect();
    const dpr = canvas.width / rect.width;
    const { data, width } = ctx.getImageData(0, 0, canvas.width, canvas.height);
    let count = 0;
    let sx = 0;
    let sy = 0;
    for (let i = 0; i < data.length; i += 4) {
      const [r, g, b] = [data[i] ?? 0, data[i + 1] ?? 0, data[i + 2] ?? 0];
      if (Math.hypot(r - 0x10, g - 0xb9, b - 0x81) >= 40) continue;
      count++;
      sx += (i / 4) % width;
      sy += Math.floor(i / 4 / width);
    }
    if (count === 0) return null;
    return { x: rect.left + sx / count / dpr, y: rect.top + sy / count / dpr };
  });
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

export interface FlashProbe {
  statusAt: number | null;
  flashAt: number | null;
}

// Watches for the status to change to the message, then samples the pixel
// every frame until it turns the colour, timing both in the page's clock.
export function armFlashProbe(page: Page, at: Client, message: string, hex: string): Promise<void> {
  return page.evaluate(
    ({ x, y, message, hex }) => {
      const probe: FlashProbe = { statusAt: null, flashAt: null };
      (window as unknown as { flashProbe: FlashProbe }).flashProbe = probe;
      const status = document.getElementById("status")!;
      const canvas = document.getElementById("canvas") as HTMLCanvasElement;
      const ctx = canvas.getContext("2d")!;
      const want = [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
      const sample = (): void => {
        const rect = canvas.getBoundingClientRect();
        const px = Math.round((x - rect.left) * (canvas.width / rect.width));
        const py = Math.round((y - rect.top) * (canvas.height / rect.height));
        const [r, g, b] = ctx.getImageData(px, py, 1, 1).data;
        if (Math.hypot(r! - want[0]!, g! - want[1]!, b! - want[2]!) < 40) {
          probe.flashAt = performance.now();
        } else if (performance.now() - probe.statusAt! < 1000) {
          requestAnimationFrame(sample);
        }
      };
      new MutationObserver(() => {
        if (probe.statusAt !== null || status.textContent !== message) return;
        probe.statusAt = performance.now();
        sample();
      }).observe(status, { childList: true, characterData: true, subtree: true });
    },
    { x: at.x, y: at.y, message, hex },
  );
}

export function readFlashProbe(page: Page): Promise<FlashProbe> {
  return page.evaluate(() => (window as unknown as { flashProbe: FlashProbe }).flashProbe);
}

// Resolves after the page has painted a frame, so a pixel read sees the
// state the status line already reports.
export function nextFrame(page: Page): Promise<void> {
  return page.evaluate(
    () =>
      new Promise<void>((resolve) =>
        requestAnimationFrame(() => requestAnimationFrame(() => resolve())),
      ),
  );
}

// The 4 px turn ring starts at the canvas edge; 2 px in is its centre line.
export function ringPixel(frame: Frame): Client {
  return { x: frame.left + 2, y: frame.top + frame.height / 2 };
}

// Ctrl+wheel notches at a client point until the view has zoomed in by the
// factor, mirroring the page's clamp of the board to the viewport.
export async function wheelZoomTo(
  page: Page,
  frame: Frame,
  from: Transform,
  at: Client,
  factor: number,
): Promise<Transform> {
  let t = from;
  let zoom = 1;
  for (let notch = 0; notch < 10 && zoom < factor - 1e-6; notch++) {
    const delta = await ctrlWheel(page, at, -100);
    const step = Math.min(2, Math.max(0.5, 2 ** (-delta / 100)));
    zoom *= step;
    t = clampToViewport(frame, zoomAt(frame, t, at, step));
  }
  return t;
}

export function clampToViewport(frame: Frame, t: Transform): Transform {
  const board = frame.board * t.scale;
  const axis = (offset: number, viewport: number): number =>
    board <= viewport ? (viewport - board) / 2 : Math.min(0, Math.max(viewport - board, offset));
  return { ...t, tx: axis(t.tx, frame.width), ty: axis(t.ty, frame.height) };
}

export async function shot(page: Page, name: string): Promise<Buffer> {
  await nextFrame(page);
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
  await expectDotAt(page, mid);
  await clickAway(page);
  await expect(page.locator("#status")).toHaveText("Your turn!", { timeout: 60_000 });
}

// Coarse map of which 10 px cells hold a stroke colour, for comparing two
// renderings of the same board without pixel-exact matching.
export function strokeCells(page: Page): Promise<boolean[]> {
  return page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const ctx = canvas.getContext("2d")!;
    const { data } = ctx.getImageData(0, 0, canvas.width, canvas.height);
    const cell = Math.round(10 * (canvas.width / canvas.getBoundingClientRect().width));
    const cols = Math.ceil(canvas.width / cell);
    const rows = Math.ceil(canvas.height / cell);
    const cells = new Array<boolean>(cols * rows).fill(false);
    for (let y = 0; y < canvas.height; y++) {
      for (let x = 0; x < canvas.width; x++) {
        const i = (y * canvas.width + x) * 4;
        const r = data[i]!;
        const g = data[i + 1]!;
        const b = data[i + 2]!;
        const stroke = (b > 180 && r < 120) || (r > 180 && g < 100);
        if (stroke) cells[Math.floor(y / cell) * cols + Math.floor(x / cell)] = true;
      }
    }
    return cells;
  });
}

// Pixels in either player's colour: nodes, strokes and the ring. A move
// adds a line and a node, so the count rises whatever the AI draws.
export function inkPixels(page: Page): Promise<number> {
  return page.evaluate(() => {
    const canvas = document.getElementById("canvas") as HTMLCanvasElement;
    const { data } = canvas.getContext("2d")!.getImageData(0, 0, canvas.width, canvas.height);
    let ink = 0;
    for (let i = 0; i < data.length; i += 4) {
      const r = data[i]!;
      const g = data[i + 1]!;
      const b = data[i + 2]!;
      if ((b > 180 && r < 120) || (r > 180 && g < 100)) ink++;
    }
    return ink;
  });
}
