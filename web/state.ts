import type { Line, Node, Player, Point } from "../pkg/sprouts";

export type { Line, Node, Player, Point };

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
  currentPlayer: Player;
  dragPath: Point[];
  lockedPath: Point[];
  placementDot: PlacementDot | null;
  dragStartNode: number | null;
  dragEndNode: number | null;
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
    currentPlayer: "human",
    dragPath: [],
    lockedPath: [],
    placementDot: null,
    dragStartNode: null,
    dragEndNode: null,
    animatingLine: null,
    animationProgress: 0,
    gameOver: false,
    reviewing: false,
  };
}

// Any assignment through the returned proxy reports a change, so the renderer
// can redraw only when something moved.
export function watched<T extends object>(target: T, onChange: () => void): T {
  return new Proxy(target, {
    set(object, key, value) {
      Reflect.set(object, key, value);
      onChange();
      return true;
    },
  });
}
