import type { BoardLocation, Cubic, Line, Node, Player, Point } from "../pkg/sprouts";

export type { BoardLocation, Cubic, Line, Node, Player, Point };

export type Mode = "idle" | "dragging" | "placing" | "animating_ai";

export interface PlacementDot {
  position: Point;
  valid: boolean;
}

export interface AnimatingLine {
  segments: Cubic[];
  newPos: Point;
}

// The path is the one the rejection's segment indices refer to.
export interface Flash {
  at: BoardLocation;
  path: Point[];
}

export interface Geometry {
  nodes: Node[];
  lines: Line[];
}

export interface Board {
  mode: Mode;
  nodes: Node[];
  lines: Line[];
  activeNodes: Set<number>;
  currentPlayer: Player;
  dragPath: Point[];
  lockedPath: Point[];
  // The fit of lockedPath once the worker returns it; placement waits for it.
  lockedSegments: Cubic[] | null;
  placementDot: PlacementDot | null;
  dragStartNode: number | null;
  dragEndNode: number | null;
  animatingLine: AnimatingLine | null;
  animationProgress: number;
  thinking: boolean;
  flash: Flash | null;
  gameOver: boolean;
  reviewing: boolean;
  // Index of the move whose line and nodes the board points out.
  highlightedMove: number | null;
  // The node whose target has keyboard focus, and the start of a keyboard move.
  focusedNode: number | null;
  pickedNode: number | null;
  // The locked curve came from the router, not a stroke.
  routed: boolean;
  // The board part way from the last snapshot to this one while the settle
  // animates; nodes and lines hold where the engine put them.
  settling: Geometry | null;
}

export interface GameState extends Board {
  reducedMotion: boolean;
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
    lockedSegments: null,
    placementDot: null,
    dragStartNode: null,
    dragEndNode: null,
    animatingLine: null,
    animationProgress: 0,
    thinking: false,
    flash: null,
    gameOver: false,
    reviewing: false,
    highlightedMove: null,
    focusedNode: null,
    pickedNode: null,
    routed: false,
    settling: null,
  };
}

// The player to move at game over is the one who lost.
export function winner(board: Board): Player | null {
  if (!board.gameOver) return null;
  return board.currentPlayer === "human" ? "ai" : "human";
}

// Any assignment through the returned proxy reports a change, so the renderer
// can redraw only when something moved.
export function watched<T extends object>(target: T, onChange: () => void): T {
  return new Proxy(target, {
    set(object, key, value) {
      const changed = !Object.is(Reflect.get(object, key), value);
      Reflect.set(object, key, value);
      if (changed) onChange();
      return true;
    },
  });
}
