import type { Point } from "./state";

export function distance(a: Point, b: Point): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function ccw(a: Point, b: Point, c: Point): boolean {
  return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
}

function segmentsIntersect(a1: Point, a2: Point, b1: Point, b2: Point): boolean {
  return ccw(a1, b1, b2) !== ccw(a2, b1, b2) && ccw(a1, a2, b1) !== ccw(a1, a2, b2);
}

// Cuts a self-crossing out of a freehand stroke by dropping the loop between
// the two crossing segments.
export function healPath(points: Point[]): Point[] {
  const first = points[0];
  if (points.length < 4 || !first) return points;
  const healed: Point[] = [first];
  for (let i = 1; i < points.length; i++) {
    const current = points[i];
    const tail = healed[healed.length - 1];
    if (!current || !tail) break;
    for (let j = 0; j < healed.length - 2; j++) {
      const a = healed[j];
      const b = healed[j + 1];
      if (a && b && segmentsIntersect(tail, current, a, b)) {
        return healPath([...healed.slice(0, j + 2), current]);
      }
    }
    healed.push(current);
  }
  return healed;
}
