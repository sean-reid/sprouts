import type { GameState, Player, Point } from "./state";
import type { View } from "./view";

const PLAYER_COLORS: Record<Player, string> = { human: "#60A5FA", ai: "#EF4444" };
const LINE_WIDTH = 3;
const NODE_RADIUS = 8;

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

export type Project = (world: Point) => Point;

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
}
