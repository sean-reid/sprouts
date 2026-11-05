import { BOARD_BACKGROUND, boardCentre, drawBoard, setCssPixelTransform } from "./render";
import type { GameState, Point } from "./state";
import type { View } from "./view";

export const LOUPE_DIAMETER = 120;
export const LOUPE_LIFT = 90;
export const LOUPE_MAGNIFICATION = 2.5;
export const LOUPE_SHOW_WITHIN = 24;
export const LOUPE_HIDE_BEYOND = 36;
export const LOUPE_RING = "#FBBF24";

const RADIUS = LOUPE_DIAMETER / 2;
const RING_WIDTH = 2;
const CROSSHAIR_ARM = 6;

function clamp(value: number, low: number, high: number): number {
  return Math.max(low, Math.min(high, value));
}

function segmentDistance(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSq = dx * dx + dy * dy;
  const t = lengthSq === 0 ? 0 : clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSq, 0, 1);
  return Math.hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
}

function polylineDistance(p: Point, points: Point[]): number {
  let best = Infinity;
  for (let i = 0; i + 1 < points.length; i++) {
    const a = points[i];
    const b = points[i + 1];
    if (a && b) best = Math.min(best, segmentDistance(p, a, b));
  }
  return best;
}

// Screen pixels from a world point to the nearest line or node.
export function nearestFeature(world: Point, state: GameState, view: View): number {
  let best = Infinity;
  for (const line of state.lines) best = Math.min(best, polylineDistance(world, line.polyline));
  for (const node of state.nodes) {
    best = Math.min(best, Math.hypot(world.x - node.position.x, world.y - node.position.y));
  }
  return best * view.scale;
}

export type LoupeSide = 1 | -1;
// When the target moves to another spot the lens swings round the finger
// toward it: LOUPE_FOLLOW px of arc per px the finger moves, and at least
// LOUPE_SETTLE px a frame with the finger still.
export const LOUPE_FOLLOW = 1.5;
export const LOUPE_SETTLE = 6;

const DEGREE = Math.PI / 180;

export function lensAt(finger: Point, angle: number): Point {
  return { x: finger.x + LOUPE_LIFT * Math.sin(angle), y: finger.y - LOUPE_LIFT * Math.cos(angle) };
}

// Degrees the stroke's side is preferred by, so a symmetric choice keeps the
// side and an edge that forces the lens lower than that swings it across.
const SIDE_PREFERENCE = 10;

// The angle round the finger, zero straight above, of the spot LOUPE_LIFT
// away that keeps the whole disc on the canvas: the highest one, on the
// stroke's side when the sides are close, so the lens drops below the finger
// only when neither side has a higher spot, which is the top corners.
export function loupeAngle(finger: Point, width: number, height: number, side: LoupeSide): number {
  let best = 0;
  let bestCost = Infinity;
  for (let degrees = -180; degrees <= 180; degrees++) {
    const p = lensAt(finger, degrees * DEGREE);
    if (p.x < RADIUS || p.x > width - RADIUS || p.y < RADIUS || p.y > height - RADIUS) continue;
    const wrongSide = degrees !== 0 && Math.sign(degrees) !== side;
    const cost = Math.abs(degrees) + (wrongSide ? SIDE_PREFERENCE : 0);
    if (cost < bestCost) {
      bestCost = cost;
      best = degrees * DEGREE;
    }
  }
  return best;
}

// Turns the lens from one angle toward another the short way round, by at
// most the arc the finger's travel allows.
export function loupeStep(from: number, target: number, travel: number): number {
  let delta = target - from;
  while (delta > Math.PI) delta -= Math.PI * 2;
  while (delta < -Math.PI) delta += Math.PI * 2;
  const limit = Math.max(LOUPE_SETTLE, travel * LOUPE_FOLLOW) / LOUPE_LIFT;
  if (Math.abs(delta) <= limit) return target;
  return from + Math.sign(delta) * limit;
}

export function loupeSwings(finger: Point): boolean {
  return finger.y - LOUPE_LIFT < RADIUS;
}

export class Loupe {
  private world: Point | null = null;
  private armed = false;
  private side: LoupeSide | null = null;
  private angle: number | null = null;
  private finger: Point | null = null;

  // Arms once the finger has cleared the node the stroke started on, then
  // follows it while it stays near a feature; returns whether the frame needs
  // a redraw.
  update(world: Point, start: Point, state: GameState, view: View): boolean {
    if (!this.armed) {
      const fromStart = Math.hypot(world.x - start.x, world.y - start.y) * view.scale;
      if (fromStart <= LOUPE_HIDE_BEYOND) return false;
      this.armed = true;
    }
    const distance = nearestFeature(world, state, view);
    const show = this.world ? distance <= LOUPE_HIDE_BEYOND : distance <= LOUPE_SHOW_WITHIN;
    if (!show) return this.hide();
    return this.follow(world);
  }

  // Keeps the lens on the finger wherever it is; returns whether it moved.
  follow(world: Point): boolean {
    const moved = !this.world || this.world.x !== world.x || this.world.y !== world.y;
    this.world = { x: world.x, y: world.y };
    return moved;
  }

  hide(): boolean {
    const shown = this.world !== null;
    this.world = null;
    this.angle = null;
    this.finger = null;
    return shown;
  }

  // The stroke is over; the next one has to clear its start node again.
  release(): boolean {
    this.armed = false;
    this.side = null;
    return this.hide();
  }

  // Returns whether the lens is still sliding toward its spot and needs
  // another frame.
  draw(ctx: CanvasRenderingContext2D, state: GameState, view: View): boolean {
    if (!this.world) return false;
    setCssPixelTransform(ctx, view);
    const finger = view.toScreen(this.world);
    // One side per stroke, so a finger crossing the middle does not flip the lens.
    if (this.side === null && loupeSwings(finger)) {
      this.side = finger.x < view.width / 2 ? 1 : -1;
    }
    const target = loupeAngle(finger, view.width, view.height, this.side ?? 1);
    const travel = this.finger ? Math.hypot(finger.x - this.finger.x, finger.y - this.finger.y) : 0;
    const angle = this.angle === null ? target : loupeStep(this.angle, target, travel);
    this.angle = angle;
    this.finger = finger;
    const centre = lensAt(finger, angle);
    const settling = angle !== target;
    const project = (p: Point): Point => {
      const s = view.toScreen(p);
      return {
        x: centre.x + (s.x - finger.x) * LOUPE_MAGNIFICATION,
        y: centre.y + (s.y - finger.y) * LOUPE_MAGNIFICATION,
      };
    };

    ctx.save();
    ctx.beginPath();
    ctx.arc(centre.x, centre.y, RADIUS, 0, Math.PI * 2);
    ctx.clip();
    ctx.fillStyle = BOARD_BACKGROUND;
    ctx.fillRect(centre.x - RADIUS, centre.y - RADIUS, LOUPE_DIAMETER, LOUPE_DIAMETER);
    drawBoard(ctx, state, project, boardCentre(view));
    ctx.strokeStyle = LOUPE_RING;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centre.x - CROSSHAIR_ARM, centre.y);
    ctx.lineTo(centre.x + CROSSHAIR_ARM, centre.y);
    ctx.moveTo(centre.x, centre.y - CROSSHAIR_ARM);
    ctx.lineTo(centre.x, centre.y + CROSSHAIR_ARM);
    ctx.stroke();
    ctx.restore();

    ctx.strokeStyle = LOUPE_RING;
    ctx.lineWidth = RING_WIDTH;
    ctx.beginPath();
    ctx.arc(centre.x, centre.y, RADIUS, 0, Math.PI * 2);
    ctx.stroke();
    return settling;
  }
}
