import type { Point } from "./state";

export const MAX_ZOOM = 8;
export const DOUBLE_TAP_ZOOM = 2.5;
export const WHEEL_ZOOM_NOTCH = 100;

export function wheelZoomFactor(deltaY: number): number {
  return Math.min(2, Math.max(0.5, 2 ** (-deltaY / WHEEL_ZOOM_NOTCH)));
}

// Maps world (engine board units) to screen (CSS pixels of the canvas).
export class View {
  width = 0;
  height = 0;
  scale = 1;
  tx = 0;
  ty = 0;
  private readonly listeners: (() => void)[] = [];

  constructor(readonly boardSize: number) {}

  onChange(listener: () => void): void {
    this.listeners.push(listener);
  }

  get fitScale(): number {
    return Math.min(this.width, this.height) / this.boardSize;
  }

  get zoom(): number {
    return this.scale / this.fitScale;
  }

  toScreen(world: Point): Point {
    return { x: world.x * this.scale + this.tx, y: world.y * this.scale + this.ty };
  }

  toWorld(screen: Point): Point {
    return { x: (screen.x - this.tx) / this.scale, y: (screen.y - this.ty) / this.scale };
  }

  resize(width: number, height: number): void {
    const zoom = this.width && this.height ? this.zoom : 1;
    const centre = this.toWorld({ x: this.width / 2, y: this.height / 2 });
    this.width = width;
    this.height = height;
    this.anchor(centre, { x: width / 2, y: height / 2 }, this.fitScale * zoom);
  }

  fit(): void {
    this.scale = this.fitScale;
    this.clamp();
    this.emit();
  }

  zoomAt(screen: Point, factor: number): void {
    this.anchor(this.toWorld(screen), screen, this.scale * factor);
  }

  // Sets the scale and places the given world point under the given screen point.
  anchor(world: Point, screen: Point, scale: number): void {
    const fit = this.fitScale;
    this.scale = Math.min(fit * MAX_ZOOM, Math.max(fit, scale));
    this.tx = screen.x - world.x * this.scale;
    this.ty = screen.y - world.y * this.scale;
    this.clamp();
    this.emit();
  }

  panBy(dx: number, dy: number): void {
    this.tx += dx;
    this.ty += dy;
    this.clamp();
    this.emit();
  }

  private clamp(): void {
    const board = this.boardSize * this.scale;
    this.tx = clampAxis(this.tx, board, this.width);
    this.ty = clampAxis(this.ty, board, this.height);
  }

  private emit(): void {
    for (const listener of this.listeners) listener();
  }
}

// A board smaller than the viewport centres; a larger one never shows a gap.
function clampAxis(offset: number, board: number, viewport: number): number {
  if (board <= viewport) return (viewport - board) / 2;
  return Math.min(0, Math.max(viewport - board, offset));
}

// Keeps the backing store at device pixels for the canvas's CSS size.
export function trackCanvasSize(canvas: HTMLCanvasElement, view: View, onResize: () => void): void {
  const apply = (): void => {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    view.resize(rect.width, rect.height);
    onResize();
  };
  apply();
  new ResizeObserver(apply).observe(canvas);
  const watchDpr = (): void => {
    const query = window.matchMedia(`(resolution: ${window.devicePixelRatio}dppx)`);
    query.addEventListener(
      "change",
      () => {
        apply();
        watchDpr();
      },
      { once: true },
    );
  };
  watchDpr();
}
