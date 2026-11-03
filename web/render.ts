import {
  winner,
  type Cubic,
  type Flash,
  type GameState,
  type Geometry,
  type Player,
  type Point,
} from "./state";
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
const ID_OFFSET = NODE_RADIUS + 13;
export const HIGHLIGHT_COLOR = "#FFFFFF";
const HIGHLIGHT_OUTLINE = 2;
export const CANDIDATE_ALPHA = 0.5;

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
  board: Geometry,
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
      at.kind === "node" ? board.nodes.find((node) => node.id === at.id)?.position : at.at;
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
  ctx.strokeStyle = PLAYER_COLORS[winner(state) ?? state.currentPlayer];
  ctx.lineWidth = RING_WIDTH;
  if (state.thinking && !state.reducedMotion) {
    const phase = (performance.now() / PULSE_PERIOD_MS) * Math.PI * 2;
    ctx.globalAlpha = 0.65 + 0.35 * Math.sin(phase);
  }
  const inset = RING_WIDTH / 2;
  ctx.strokeRect(inset, inset, view.width - RING_WIDTH, view.height - RING_WIDTH);
  ctx.globalAlpha = 1;
}

function strokeRing(ctx: CanvasRenderingContext2D, at: Point): void {
  ctx.beginPath();
  ctx.arc(at.x, at.y, FLASH_RING_RADIUS, 0, Math.PI * 2);
  ctx.stroke();
}

function lerp(a: Point, b: Point, t: number): Point {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

// The part of a cubic before parameter t, by de Casteljau.
function head(c: Cubic, t: number): Cubic {
  const p01 = lerp(c.p0, c.p1, t);
  const p12 = lerp(c.p1, c.p2, t);
  const p23 = lerp(c.p2, c.p3, t);
  const p012 = lerp(p01, p12, t);
  const p123 = lerp(p12, p23, t);
  return { p0: c.p0, p1: p01, p2: p012, p3: lerp(p012, p123, t) };
}

// Control points go through the projection like any point, so the screen
// curve is the affine image of the world curve. `upTo` is the fraction of
// segments drawn.
function strokeSegments(
  ctx: CanvasRenderingContext2D,
  segments: Cubic[],
  project: Project,
  upTo = 1,
): void {
  const first = segments[0];
  if (!first) return;
  const total = segments.length * Math.min(1, Math.max(0, upTo));
  const whole = Math.floor(total);
  const partial = total - whole;
  ctx.beginPath();
  const start = project(first.p0);
  ctx.moveTo(start.x, start.y);
  for (let i = 0; i < segments.length; i++) {
    let c = segments[i];
    if (!c) break;
    if (i >= whole) {
      if (partial <= 0) break;
      c = head(c, partial);
    }
    const p1 = project(c.p1);
    const p2 = project(c.p2);
    const p3 = project(c.p3);
    ctx.bezierCurveTo(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
    if (i >= whole) break;
  }
  ctx.stroke();
}

// Strokes the move's line again, wider, rings its two end nodes and labels
// the three nodes it touched with their ids. Ids sit on the side of the
// disc facing the board centre so they stay inside the canvas edge.
function drawHighlight(
  ctx: CanvasRenderingContext2D,
  board: Geometry,
  index: number,
  project: Project,
  centre: Point,
): void {
  const line = board.lines[index];
  if (!line) return;
  const created = board.nodes.length - board.lines.length + index;
  const nodeAt = (id: number): Point | undefined =>
    board.nodes.find((node) => node.id === id)?.position;
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = HIGHLIGHT_COLOR;
  ctx.lineWidth = LINE_WIDTH + 2 * HIGHLIGHT_OUTLINE;
  strokeSegments(ctx, line.segments, project);
  ctx.strokeStyle = PLAYER_COLORS[line.player];
  ctx.lineWidth = LINE_WIDTH;
  strokeSegments(ctx, line.segments, project);
  ctx.strokeStyle = HIGHLIGHT_COLOR;
  ctx.fillStyle = HIGHLIGHT_COLOR;
  for (const id of new Set([line.fromNode, line.toNode])) {
    const world = nodeAt(id);
    if (world) strokeRing(ctx, project(world));
  }
  const origin = project(centre);
  for (const id of new Set([line.fromNode, line.toNode, created])) {
    const world = nodeAt(id);
    if (!world) continue;
    const at = project(world);
    const dx = origin.x - at.x;
    const dy = origin.y - at.y;
    const span = Math.hypot(dx, dy) || 1;
    const x = at.x + (dx / span) * ID_OFFSET;
    const y = at.y + (dy / span) * ID_OFFSET;
    ctx.fillText(String(id), x, y + 0.5);
  }
  ctx.restore();
}

export const BOARD_BACKGROUND = "#1A1A1A";

// Points are mapped to screen before stroking so widths stay constant under
// zoom. `centre` is the board centre in world units. While a settle
// animates, the board drawn is the one part way to the snapshot.
export function drawBoard(
  ctx: CanvasRenderingContext2D,
  state: GameState,
  project: Project,
  centre: Point,
): void {
  const screen = (points: Point[]): Point[] => points.map(project);
  const board: Geometry = state.settling ?? state;

  ctx.lineWidth = LINE_WIDTH;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  for (const line of board.lines) {
    ctx.strokeStyle = PLAYER_COLORS[line.player];
    strokeSegments(ctx, line.segments, project);
  }

  ctx.strokeStyle = "#3B82F6";
  if (state.mode === "dragging") {
    strokePolyline(ctx, screen(state.dragPath));
  } else if (state.mode === "placing") {
    ctx.save();
    if (state.routed) {
      ctx.strokeStyle = PLAYER_COLORS[state.currentPlayer];
      ctx.globalAlpha = CANDIDATE_ALPHA;
    }
    if (state.lockedSegments) strokeSegments(ctx, state.lockedSegments, project);
    else strokePolyline(ctx, screen(state.lockedPath));
    ctx.restore();
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
  for (const node of board.nodes) {
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

  ctx.strokeStyle = HIGHLIGHT_COLOR;
  ctx.lineWidth = LINE_WIDTH;
  for (const id of new Set([state.pickedNode, state.focusedNode])) {
    const node = id === null ? undefined : board.nodes.find((n) => n.id === id);
    if (node) strokeRing(ctx, project(node.position));
  }

  if (state.highlightedMove !== null) {
    drawHighlight(ctx, board, state.highlightedMove, project, centre);
  }

  if (state.mode === "animating_ai" && state.animatingLine) {
    ctx.strokeStyle = PLAYER_COLORS.ai;
    ctx.lineWidth = LINE_WIDTH;
    strokeSegments(ctx, state.animatingLine.segments, project, state.animationProgress);
  }

  if (state.flash) drawFlash(ctx, board, state.flash, project);
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
  drawBoard(ctx, state, (p) => view.toScreen(p), boardCentre(view));
  drawRing(ctx, state, view);
}

export function boardCentre(view: View): Point {
  return { x: view.boardSize / 2, y: view.boardSize / 2 };
}
