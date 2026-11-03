import type { Point } from "../pkg/sprouts";

// Even-odd ray cast along +x; a point on an edge counts as inside.
export function pointInPolygon(point: Point, polygon: Point[]): boolean {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const a = polygon[i];
    const b = polygon[j];
    if (!a || !b) continue;
    const crosses = a.y > point.y !== b.y > point.y;
    if (crosses && point.x < ((b.x - a.x) * (point.y - a.y)) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}

// How many of the points a closed stroke encloses; a stroke of fewer than
// three points bounds nothing.
export function enclosedCount(polyline: Point[], points: Point[]): number {
  if (polyline.length < 3) return 0;
  return points.filter((p) => pointInPolygon(p, polyline)).length;
}

// What a move did to the board, for a reader who cannot see it.
export function shapeText(loop: boolean, enclosed: number): string {
  if (!loop) return "joins two nodes";
  return enclosed > 0 ? `loop on one node, encloses ${enclosed}` : "loop on one node";
}
