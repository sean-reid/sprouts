export interface Point {
  x: number;
  y: number;
}

export interface Node {
  id: number;
  position: Point;
  connectionCount: number;
}

export interface Line {
  id: number;
  from: number;
  to: number;
  polyline: Point[];
  newNodePos: Point;
  player: number;
}

export type Mode = "idle" | "dragging" | "placing" | "animating_ai";

export interface PlacementDot {
  position: Point;
  valid: boolean;
}

export interface AnimatingLine {
  path: Point[];
  newPos: Point;
}

export interface Board {
  mode: Mode;
  nodes: Node[];
  lines: Line[];
  activeNodes: Set<number>;
  currentPlayer: number;
  dragPath: Point[];
  lockedPath: Point[];
  placementDot: PlacementDot | null;
  dragStartNode: number | null;
  dragEndNode: number | null;
  hoveredNode: number | null;
  animatingLine: AnimatingLine | null;
  animationProgress: number;
  gameOver: boolean;
  reviewing: boolean;
}

export interface GameState extends Board {
  // Tracked per gesture rather than as a device flag so hybrids (Surface,
  // iPad with keyboard) can use touch and mouse in the same session.
  lastInputWasTouch: boolean;
}

export function freshBoard(): Board {
  return {
    mode: "idle",
    nodes: [],
    lines: [],
    activeNodes: new Set(),
    currentPlayer: 0,
    dragPath: [],
    lockedPath: [],
    placementDot: null,
    dragStartNode: null,
    dragEndNode: null,
    hoveredNode: null,
    animatingLine: null,
    animationProgress: 0,
    gameOver: false,
    reviewing: false,
  };
}

export function toPathData(points: Point[]): number[] {
  return points.flatMap((p) => [p.x, p.y]);
}
