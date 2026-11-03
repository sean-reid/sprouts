import type { Flash, GameState, Player, Point } from "./state";
import type { View } from "./view";

export const PLAYER_COLORS: Record<Player, string> = { human: "#60A5FA", ai: "#EF4444" };
const LINE_WIDTH = 3;
const NODE_RADIUS = 8;
const RING_WIDTH = 4;
const PULSE_PERIOD_MS = 1200;
const FLASH_COLOR = "#EF4444";
const FLASH_WIDTH = 6;
const FLASH_MIN_LENGTH = 24;
const FLASH_RING_RADIUS = NODE_RADIUS + 6;

export type Project = (world: Point) => Point;

function strokePolyline(ctx: CanvasRenderingContext2D, points: Point[]): void {
  const first = points[0];
  if (!first) return;
  ctx.beginPath();
  ctx.moveTo(first.x, first.y);
  for (const point of points.slice(1)) ctx.lineTo(point.x, point.y);
  ctx.stroke();
}

function length(points: Point[]): number {
  let total = 0;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1];
    const b = points[i];
    if (a && b) total += Math.hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

// Widens the flashed range along the path until it is long enough to see.
function flashedSpan(path: Point[], first: number, last: number): Point[] {
  let lo = Math.min(first, path.length - 2);
  let hi = Math.min(last + 1, path.length - 1);
  while (length(path.slice(lo, hi + 1)) < FLASH_MIN_LENGTH && (lo > 0 || hi < path.length - 1)) {
    if (lo > 0) lo--;
    if (hi < path.length - 1) hi++;
  }
  return path.slice(lo, hi + 1);
}

function drawFlash(
  ctx: CanvasRenderingContext2D,
  state: GameState,
  flash: Flash,
  project: Project,
): void {
  ctx.save();
  ctx.strokeStyle = FLASH_COLOR;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  const { at } = flash;
  if (at.kind === "segments") {
    const path = flash.path.map(project);
    ctx.lineWidth = FLASH_WIDTH;
    strokePolyline(ctx, flashedSpan(path, at.first, at.last));
  } else {
    const world =
      at.kind === "node" ? state.nodes.find((node) => node.id === at.id)?.position : at.at;
    if (world) {
      const centre = project(world);
      ctx.lineWidth = LINE_WIDTH;
      ctx.beginPath();
      ctx.arc(centre.x, centre.y, FLASH_RING_RADIUS, 0, Math.PI * 2);
      ctx.stroke();
    }
  }
  ctx.restore();
}

// The ring sits just inside the canvas edge in screen pixels, whatever the zoom.
function drawRing(ctx: CanvasRenderingContext2D, state: GameState, view: View): void {
  ctx.strokeStyle = PLAYER_COLORS[state.currentPlayer];
  ctx.lineWidth = RING_WIDTH;
  if (state.thinking && !state.reducedMotion) {
    const phase = (performance.now() / PULSE_PERIOD_MS) * Math.PI * 2;
    ctx.globalAlpha = 0.65 + 0.35 * Math.sin(phase);
  }
  const inset = RING_WIDTH / 2;
  ctx.strokeRect(inset, inset, view.width - RING_WIDTH, view.height - RING_WIDTH);
  ctx.globalAlpha = 1;
}

// Catmull-Rom through every point; tension 6 is the standard value.
function drawSmoothPath(ctx: CanvasRenderingContext2D, points: Point[]): void {
  const first = points[0];
  const second = points[1];
  if (!first || !second) return;
  ctx.beginPath();
  ctx.moveTo(first.x, first.y);
  if (points.length === 2) {
    ctx.lineTo(second.x, second.y);
  } else {
    for (let i = 0; i < points.length - 1; i++) {
      const p0 = points[Math.max(i - 1, 0)];
      const p1 = points[i];
      const p2 = points[i + 1];
      const p3 = points[Math.min(i + 2, points.length - 1)];
      if (!p0 || !p1 || !p2 || !p3) break;
      const t = 6;
      const cp1x = p1.x + (p2.x - p0.x) / t;
      const cp1y = p1.y + (p2.y - p0.y) / t;
      const cp2x = p2.x - (p3.x - p1.x) / t;
      const cp2y = p2.y - (p3.y - p1.y) / t;
      ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, p2.x, p2.y);
    }
  }
  ctx.stroke();
}

export const BOARD_BACKGROUND = "#1A1A1A";

// Points are mapped to screen before stroking so widths stay constant under zoom.
export function drawBoard(ctx: CanvasRenderingContext2D, state: GameState, project: Project): void {
  const screen = (points: Point[]): Point[] => points.map(project);

  ctx.lineWidth = LINE_WIDTH;
  for (const line of state.lines) {
    ctx.strokeStyle = PLAYER_COLORS[line.player];
    drawSmoothPath(ctx, screen(line.polyline));
  }

  const activePath =
    state.mode === "dragging" ? state.dragPath : state.mode === "placing" ? state.lockedPath : null;
  if (activePath && activePath.length > 1) {
    ctx.strokeStyle = "#3B82F6";
    drawSmoothPath(ctx, screen(activePath));
  }

  if (state.mode === "placing" && state.placementDot) {
    const { position, valid } = state.placementDot;
    const at = project(position);
    ctx.fillStyle = valid ? "#10B981" : "#EF4444";
    ctx.beginPath();
    ctx.arc(at.x, at.y, NODE_RADIUS, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.font = "bold 10px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const node of state.nodes) {
    const remaining = 3 - node.connectionCount;
    const at = project(node.position);
    ctx.fillStyle = state.activeNodes.has(node.id) ? "#3B82F6" : "#555";
    ctx.strokeStyle = "#FFF";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(at.x, at.y, NODE_RADIUS, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#FFF";
    ctx.fillText(String(remaining), at.x, at.y + 0.5);
  }

  if (state.mode === "animating_ai" && state.animatingLine) {
    const { path } = state.animatingLine;
    const shown = Math.floor(state.animationProgress * path.length);
    if (shown > 1) {
      ctx.strokeStyle = PLAYER_COLORS.ai;
      ctx.lineWidth = LINE_WIDTH;
      drawSmoothPath(ctx, screen(path.slice(0, shown)));
    }
  }

  if (state.flash) drawFlash(ctx, state, state.flash, project);
}

// Draws in CSS pixels on a backing store kept at device pixels.
export function setCssPixelTransform(ctx: CanvasRenderingContext2D, view: View): void {
  const dpr = view.width > 0 ? ctx.canvas.width / view.width : 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

export function draw(ctx: CanvasRenderingContext2D, state: GameState, view: View): void {
  setCssPixelTransform(ctx, view);
  ctx.fillStyle = BOARD_BACKGROUND;
  ctx.fillRect(0, 0, view.width, view.height);
  drawBoard(ctx, state, (p) => view.toScreen(p));
  drawRing(ctx, state, view);
}
