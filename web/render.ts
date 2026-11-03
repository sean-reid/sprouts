import type { GameState, Point } from "./state";

const PLAYER_COLORS = ["#60A5FA", "#EF4444"] as const;

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

export function draw(ctx: CanvasRenderingContext2D, state: GameState): void {
  const { width, height } = ctx.canvas;
  ctx.fillStyle = "#1A1A1A";
  ctx.fillRect(0, 0, width, height);

  for (const line of state.lines) {
    ctx.strokeStyle = PLAYER_COLORS[line.player === 0 ? 0 : 1];
    ctx.lineWidth = 3;
    drawSmoothPath(ctx, line.polyline);
  }

  const activePath =
    state.mode === "dragging" ? state.dragPath : state.mode === "placing" ? state.lockedPath : null;
  if (activePath && activePath.length > 1) {
    ctx.strokeStyle = "#3B82F6";
    ctx.lineWidth = 3;
    drawSmoothPath(ctx, activePath);
  }

  if (state.mode === "placing" && state.placementDot) {
    const { position, valid } = state.placementDot;
    ctx.fillStyle = valid ? "#10B981" : "#EF4444";
    ctx.beginPath();
    ctx.arc(position.x, position.y, 8, 0, Math.PI * 2);
    ctx.fill();
  }

  for (const node of state.nodes) {
    const remaining = 3 - node.connectionCount;
    ctx.fillStyle = state.activeNodes.has(node.id) ? "#3B82F6" : "#555";
    ctx.strokeStyle = "#FFF";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(node.position.x, node.position.y, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#FFF";
    ctx.font = "bold 10px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(String(remaining), node.position.x, node.position.y + 0.5);
  }

  if (state.mode === "animating_ai" && state.animatingLine) {
    const { path } = state.animatingLine;
    const shown = Math.floor(state.animationProgress * path.length);
    if (shown > 1) {
      ctx.strokeStyle = PLAYER_COLORS[1];
      ctx.lineWidth = 3;
      drawSmoothPath(ctx, path.slice(0, shown));
    }
  }
}
