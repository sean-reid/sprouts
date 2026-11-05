import type {
  Classification,
  Classified,
  Cubic,
  FaceMove,
  Indexed,
  IndexedMove,
  Line,
  Move,
  Outcome,
  Player,
  Point,
  Rehearsal,
  Rehearsed,
} from "../pkg/sprouts";
import { circleLayout, openingLayout, sameLayout, type Position } from "./layout";
import { distance } from "./path";
import type { Fixture, FixtureMove, MoveCache } from "./record";
import { shapeText } from "./shape";

// A move a link carried as its face move and node; the engine draws the
// curve on load.
export interface TopologyMove {
  from: number;
  to: number;
  face: FaceMove;
  node: [number, number];
}

// A move a link carried as its index in the engine's move list for the
// position; its ends are unknown until the engine has drawn it.
export interface IndexedRecordMove {
  index: number;
  along: number;
}

export type RecordMove = FixtureMove | TopologyMove | IndexedRecordMove;

// A replay fixture plus the fields a resumed session needs, so a saved
// game loads in the bench unchanged; a record decoded from a link may hold
// moves the engine has yet to draw.
export interface GameRecord extends Omit<Fixture, "moves"> {
  version: 1;
  firstPlayer: Player;
  cursor: number;
  // The settle the game was played under; 0 replays with none.
  settleVersion: number;
  moves: RecordMove[];
  // The router, move list and settle the moves' caches were worked out
  // under; a record from other versions loads with no cache.
  cacheVersions?: CacheVersions;
  shapes?: ShapesCache;
}

// What a share settled for the shapes link of this exact move list: the
// move count, whether the board on screen was checked against the
// rehearsed one, and the warnings it ended with; each move's form sits
// in its cache. Any change to the move list drops it.
export interface ShapesCache {
  moves: number;
  checked?: true;
  mismatch?: boolean;
  refusedAt?: number;
}

export interface CacheVersions {
  router: number;
  list: number;
  settle: number;
}

function cacheVersions(): CacheVersions {
  return { router: ROUTER_VERSION, list: MOVE_LIST_VERSION, settle: SETTLE_VERSION };
}

function currentVersions(versions: CacheVersions | undefined): boolean {
  const now = cacheVersions();
  return (
    versions !== undefined &&
    versions.router === now.router &&
    versions.list === now.list &&
    versions.settle === now.settle
  );
}

export function hasStroke(move: RecordMove): move is FixtureMove {
  return "polyline" in move;
}

// A move carried by index alone; a saved stroke keeps its cached index
// beside its polyline and is not one.
export function isIndexed(move: RecordMove): move is IndexedRecordMove {
  return "index" in move && !("polyline" in move);
}

// A move with what the engine reported about it: the spline it fitted, the
// face move the router draws this very line from, when it does, the
// nodes it cut off, unknown until the engine has applied it, and its
// index in the move list when a link carries it that way. The saved
// record keeps the stroke and the count. A move that arrived as topology
// or by index has an empty polyline until the engine has drawn it.
export interface RecordedMove extends Move {
  segments?: Cubic[];
  topology?: FaceMove;
  cutOff?: number;
  indexed?: IndexedMove;
  // The regenerated board would not take this move, so the game cannot
  // travel as indices while the move stands.
  refused?: true;
  // How a share settled the move travels in a shapes link.
  shape?: "topology" | "geometry";
}

export const UNKNOWN_NODE = -1;

export function drawn(move: RecordedMove): boolean {
  return move.polyline.length > 0;
}

export type Applied = Extract<Outcome, { kind: "applied" }>;
// A line as the engine drew it, before the board settled around it.
type DrawnLine = Pick<Line, "fromNode" | "toNode" | "polyline" | "newNodePos" | "segments">;

export const STORAGE_KEY = "sprouts.game";
const HASH_PREFIX = "#game=";
const RECORD_VERSION = 1;
// Link version 1 carries thinned strokes, version 2 the fitted cubics,
// version 3 the starting layout when it is not the default, version 4 a
// move as its face move where the router redraws it, and version 5 the
// settle version; a version 1 or 2 link was drawn on the circle layout
// and a link before version 5 was played with no settling.
const LINK_VERSION = 6;
// Match ROUTER_VERSION, SETTLE_VERSION and MOVE_LIST_VERSION in
// src/protocol.rs; a link
// carries them after the version byte from versions 4 and 5 on.
export const ROUTER_VERSION = 7;
export const SETTLE_VERSION = 1;
export const MOVE_LIST_VERSION = 1;
const SHAPES_LINK_VERSION = 5;
const FACE_LINK_VERSION = 4;
const LAYOUT_LINK_VERSION = 3;
const CUBIC_LINK_VERSION = 2;
const STROKE_LINK_VERSION = 1;
const LINK_VERSIONS = [
  LINK_VERSION,
  SHAPES_LINK_VERSION,
  FACE_LINK_VERSION,
  LAYOUT_LINK_VERSION,
  CUBIC_LINK_VERSION,
  STROKE_LINK_VERSION,
];
const FLAG_AI_FIRST = 1;
const FLAG_LAYOUT = 2;
// Coordinates travel as tenths of a world unit in a version 1 link.
const TENTHS = 10;
// Geometry below is in units of a 1000 board and scales with the board.
// The fit tolerance of src/curve.rs: the engine keeps a curve within it of
// the stroke, so a curve flattened at it refits inside the error the
// engine already accepts. A stroke point closer than STROKE_TOLERANCE to
// the chord of its neighbours is left out of the link.
export const CURVE_TOLERANCE = 1.5;
export const STROKE_TOLERANCE = 1.5;
// Per move shape in the link.
const SHAPE_STROKE = 0;
const SHAPE_CUBICS = 1;
const SHAPE_FACE = 2;
// Bit 0 of an indexed move's varint says a place along the curve follows.
const HAS_PLACE = 1;
// Each move adds four half-edges to the face model.
const HALF_EDGES_PER_MOVE = 4;
// Points travel in whole units, which moves one by at most 0.71.
export const LINK_QUANTUM = 1;
// A routed move whose rehearsed node lands farther than this from the
// recorded one, in units at board size 1000, goes as geometry instead.
export const DRIFT_TOLERANCE = 4;
// A link can claim any control point; the subdivision is capped so a
// hostile one costs at most this many points per cubic.
const MAX_SUBDIVISIONS = 256;
// Bounds for a record that arrived by link; a Sprouts game lasts at most
// 3n - 1 moves from n nodes. Endpoints must name nodes that exist before
// the move; points are unbounded since strokes may leave the board.
const MAX_NODES = 500;
const MIN_BOARD = 100;
const MAX_BOARD = 6000;

export class History {
  moves: RecordedMove[] = [];
  cursor = 0;
  // Starting positions once the engine has laid the board out.
  nodes: Position[] | null = null;
  // The shapes link a share settled for this move list, until it changes.
  shapes: ShapesCache | undefined;

  constructor(
    public initialNodes: number,
    public boardSize: number,
    public firstPlayer: Player = "human",
    public settleVersion: number = SETTLE_VERSION,
  ) {}

  static from(record: GameRecord): History {
    const history = new History(
      record.initialNodes,
      record.boardSize,
      record.firstPlayer,
      record.settleVersion,
    );
    history.nodes = record.nodes ?? null;
    history.moves = record.moves.map(fromRecordMove);
    history.shapes = record.shapes;
    // A cache from other versions is dropped; a move a link carries by
    // index alone keeps its index, which is the move itself.
    if (!currentVersions(record.cacheVersions)) {
      for (const move of history.moves.filter(drawn)) {
        delete move.indexed;
        delete move.refused;
        if (move.shape) delete move.topology;
        delete move.shape;
      }
      history.shapes = undefined;
    }
    history.cursor = Math.min(Math.max(record.cursor, 0), history.moves.length);
    return history;
  }

  get atStart(): boolean {
    return this.cursor === 0;
  }

  get atEnd(): boolean {
    return this.cursor === this.moves.length;
  }

  // Player who makes move number index + 1.
  moverOf(index: number): Player {
    const firstMoves = index % 2 === 0;
    return firstMoves === (this.firstPlayer === "human") ? "human" : "ai";
  }

  toMove(): Player {
    return this.moverOf(this.cursor);
  }

  // A move made from the cursor discards whatever was ahead of it; the
  // AI's move comes with the face move it was drawn from.
  push(move: Move, topology?: FaceMove, cutOff?: number): void {
    this.moves.length = this.cursor;
    const kept = cloneMove(move);
    if (cutOff !== undefined) kept.cutOff = cutOff;
    if (topology) kept.topology = topology;
    this.moves.push(kept);
    this.cursor = this.moves.length;
    this.shapes = undefined;
  }

  // Whether every move carries the index the link would send. A move's
  // index depends only on the moves before it, so the moves a new move or
  // a truncation discards take their caches with them and the rest hold.
  get indexed(): boolean {
    return this.moves.length > 0 && this.moves.every((move) => move.indexed);
  }

  // A copy to work on while the game goes on.
  clone(): History {
    const copy = new History(
      this.initialNodes,
      this.boardSize,
      this.firstPlayer,
      this.settleVersion,
    );
    copy.nodes = this.nodes ? this.nodes.map(([x, y]) => [x, y]) : null;
    copy.moves = this.moves.map(cloneMove);
    copy.cursor = this.cursor;
    copy.shapes = this.shapes ? { ...this.shapes } : undefined;
    return copy;
  }

  truncate(): void {
    this.moves.length = this.cursor;
    this.shapes = undefined;
  }

  // Whether the shapes a share settled still describe this move list; a
  // share with the board on screen needs a cache that checked it too.
  shaped(withBoard: boolean): boolean {
    return (
      this.shapes !== undefined &&
      this.shapes.moves === this.moves.length &&
      (!withBoard || this.shapes.checked === true) &&
      this.moves.every((move) => move.shape && (move.shape === "geometry" || move.topology))
    );
  }

  // Takes the spline the engine holds for each move up to the cursor. Under
  // a settle the engine's lines are not the moves as drawn, so only an
  // unsettled game takes its splines from them.
  adopt(lines: Line[]): void {
    if (this.settleVersion !== 0) return;
    lines.forEach((line, i) => {
      const move = this.moves[i];
      if (!move) return;
      if (line.fromNode === move.fromNode && line.toNode === move.toNode) {
        move.segments = line.segments;
      }
    });
  }

  // Takes the line the engine drew for move number index + 1 when the move
  // arrived as topology.
  settle(index: number, line: DrawnLine | undefined): void {
    const move = this.moves[index];
    if (!move || !line || drawn(move)) return;
    if (move.fromNode === UNKNOWN_NODE) {
      move.fromNode = line.fromNode;
      move.toNode = line.toNode;
    }
    if (line.fromNode === move.fromNode && line.toNode === move.toNode) {
      move.polyline = line.polyline.map((p) => ({ x: p.x, y: p.y }));
      move.newNodePos = { x: line.newNodePos.x, y: line.newNodePos.y };
      move.segments = line.segments;
    }
  }

  // Takes the outcome of applying move number index + 1.
  applied(index: number, outcome: Applied): void {
    const { segments } = outcome.mov;
    if (segments) this.settle(index, { ...outcome.mov, segments });
    const move = this.moves[index];
    if (move) move.cutOff = outcome.cutOff;
  }

  // The id of the node move number index + 1 created.
  newNodeOf(index: number): number {
    return this.initialNodes + index;
  }

  // Where a node sits: a starting one from the layout, a later one where
  // its move put it. Unknown until the engine has laid the board out.
  positionOf(id: number): Point | null {
    if (id < this.initialNodes) {
      const start = this.nodes?.[id];
      return start ? { x: start[0], y: start[1] } : null;
    }
    const move = this.moves[id - this.initialNodes];
    return move?.newNodePos ?? null;
  }

  label(index: number): string {
    const move = this.moves[index];
    if (!move) return "";
    const who = this.moverOf(index) === "human" ? "You" : "AI";
    if (move.fromNode === UNKNOWN_NODE) return `${index + 1}. ${who}`;
    const loop = move.fromNode === move.toNode;
    return `${index + 1}. ${who}, ${shapeText(loop, move.cutOff ?? 0)}`;
  }

  // The node ids of a move, for the row's title.
  ids(index: number): string {
    const move = this.moves[index];
    if (!move) return "";
    if (move.fromNode === UNKNOWN_NODE) return `new node ${this.newNodeOf(index)}`;
    return `${move.fromNode} to ${move.toNode}, new node ${this.newNodeOf(index)}`;
  }

  record(): GameRecord {
    return {
      version: 1,
      initialNodes: this.initialNodes,
      boardSize: this.boardSize,
      ...(this.nodes ? { nodes: this.nodes } : {}),
      firstPlayer: this.firstPlayer,
      settleVersion: this.settleVersion,
      moves: this.moves.map(toRecordMove),
      cursor: this.cursor,
      ...(this.moves.some((move) => move.indexed || move.refused || move.shape)
        ? { cacheVersions: cacheVersions() }
        : {}),
      ...(this.shapes ? { shapes: { ...this.shapes } } : {}),
    };
  }
}

// Records what the engine's indexing said about a move: its index, or
// `false` when the regenerated board refused it.
function setIndexed(move: RecordedMove, indexed: IndexedMove | false): void {
  if (indexed) {
    move.indexed = indexed;
    delete move.refused;
  } else {
    delete move.indexed;
    move.refused = true;
  }
}

function cloneMove(move: RecordedMove): RecordedMove {
  const clone: RecordedMove = toMove(move);
  if (move.segments) clone.segments = move.segments;
  if (move.topology) clone.topology = move.topology;
  if (move.cutOff !== undefined) clone.cutOff = move.cutOff;
  if (move.indexed) clone.indexed = move.indexed;
  if (move.refused) clone.refused = true;
  if (move.shape) clone.shape = move.shape;
  return clone;
}

// The face move to send for a move the engine has yet to draw.
export function pendingFace(move: RecordedMove): FaceMove | undefined {
  return drawn(move) ? undefined : move.topology;
}

// The indexed move to send for a move the engine has yet to list and draw.
export function pendingIndex(move: RecordedMove): IndexedMove | undefined {
  return drawn(move) ? undefined : move.indexed;
}

// The move as the engine takes it, without the segments the page keeps.
export function toMove(move: RecordedMove): Move {
  return {
    fromNode: move.fromNode,
    toNode: move.toNode,
    polyline: move.polyline.map((p) => ({ x: p.x, y: p.y })),
    newNodePos: { x: move.newNodePos.x, y: move.newNodePos.y },
    ...(move.segments ? { segments: move.segments } : {}),
  };
}

// The saved record holds strokes only; a move the engine has not drawn
// yet has none to give.
function toRecordMove(move: RecordedMove): FixtureMove {
  if (!drawn(move)) throw new RangeError("a move the engine has not drawn cannot be saved");
  const cache = cacheOf(move);
  return {
    from: move.fromNode,
    to: move.toNode,
    polyline: move.polyline.map((p): [number, number] => [p.x, p.y]),
    newNode: [move.newNodePos.x, move.newNodePos.y],
    ...(move.cutOff !== undefined ? { cutOff: move.cutOff } : {}),
    ...(move.segments ? { segments: move.segments } : {}),
    ...(cache ? { cache } : {}),
  };
}

// The saved form of what shares worked out for a move, or undefined when
// nothing is cached.
function cacheOf(move: RecordedMove): MoveCache | undefined {
  const cache: MoveCache = {
    ...(move.indexed ? { index: move.indexed.index, along: move.indexed.along } : {}),
    ...(move.refused ? { refused: true as const } : {}),
    ...(move.shape ? { shape: move.shape } : {}),
    ...(move.shape === "topology" && move.topology ? { topology: move.topology } : {}),
  };
  return Object.keys(cache).length > 0 ? cache : undefined;
}

function fromRecordMove(move: RecordMove): RecordedMove {
  if (isIndexed(move)) {
    return {
      fromNode: UNKNOWN_NODE,
      toNode: UNKNOWN_NODE,
      polyline: [],
      newNodePos: { x: 0, y: 0 },
      indexed: { index: move.index, along: move.along },
    };
  }
  if (hasStroke(move)) {
    return {
      fromNode: move.from,
      toNode: move.to,
      polyline: move.polyline.map(([x, y]) => ({ x, y })),
      newNodePos: { x: move.newNode[0], y: move.newNode[1] },
      ...(move.cutOff !== undefined ? { cutOff: move.cutOff } : {}),
      ...(move.segments ? { segments: move.segments } : {}),
      ...(move.cache?.index !== undefined && move.cache.along !== undefined
        ? { indexed: { index: move.cache.index, along: move.cache.along } }
        : {}),
      ...(move.cache?.refused ? { refused: true as const } : {}),
      ...(move.cache?.shape ? { shape: move.cache.shape } : {}),
      ...(move.cache?.shape === "topology" && move.cache.topology
        ? { topology: move.cache.topology }
        : {}),
    };
  }
  return {
    fromNode: move.from,
    toNode: move.to,
    polyline: [],
    newNodePos: { x: move.node[0], y: move.node[1] },
    topology: move.face,
  };
}

export function save(history: History): void {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(history.record()));
  } catch {
    // Storage can be full or blocked; the game still plays.
  }
}

export function clearSaved(): void {
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch {
    // Nothing to clear when storage is blocked.
  }
}

export function loadSaved(): GameRecord | null {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return null;
    return asRecord(JSON.parse(raw));
  } catch {
    return null;
  }
}

function asRecord(value: unknown): GameRecord | null {
  if (typeof value !== "object" || value === null) return null;
  const v = value as Record<string, unknown>;
  if (v.version !== RECORD_VERSION) return null;
  if (!isCount(v.initialNodes) || !isCount(v.boardSize) || !isCount(v.cursor)) return null;
  if (v.initialNodes < 1 || v.initialNodes > MAX_NODES) return null;
  if (!isBoard(v.boardSize)) return null;
  if (v.firstPlayer !== "human" && v.firstPlayer !== "ai") return null;
  if (!Array.isArray(v.moves) || v.moves.length > 3 * v.initialNodes) return null;
  const nodes = v.initialNodes;
  if (!v.moves.every((m, i) => isRecordMove(m, nodes + i, HALF_EDGES_PER_MOVE * i))) {
    return null;
  }
  if (v.cursor > v.moves.length) return null;
  if (v.nodes !== undefined && !isLayout(v.nodes, nodes)) return null;
  // A record from a newer settle would replay through this one onto
  // another board.
  const settleVersion = v.settleVersion ?? 0;
  if (!isCount(settleVersion) || settleVersion > SETTLE_VERSION) return null;
  const cacheVersions = v.cacheVersions === undefined ? undefined : asVersions(v.cacheVersions);
  if (v.cacheVersions !== undefined && !cacheVersions) return null;
  const shapes = v.shapes === undefined ? undefined : asShapes(v.shapes, v.moves.length);
  if (v.shapes !== undefined && !shapes) return null;
  return {
    version: 1,
    initialNodes: v.initialNodes,
    boardSize: v.boardSize,
    // A record saved without positions was played on the circle layout.
    nodes: v.nodes ?? circleLayout(nodes, v.boardSize),
    firstPlayer: v.firstPlayer,
    settleVersion,
    moves: v.moves,
    cursor: v.cursor,
    ...(cacheVersions ? { cacheVersions } : {}),
    ...(shapes ? { shapes } : {}),
  };
}

function asShapes(value: unknown, count: number): ShapesCache | null {
  if (typeof value !== "object" || value === null) return null;
  const c = value as Record<string, unknown>;
  if (!isCount(c.moves) || c.moves > count) return null;
  if (c.checked !== undefined && c.checked !== true) return null;
  if (c.mismatch !== undefined && typeof c.mismatch !== "boolean") return null;
  if (c.refusedAt !== undefined && !(isCount(c.refusedAt) && c.refusedAt < count)) return null;
  return {
    moves: c.moves,
    ...(c.checked ? { checked: true as const } : {}),
    ...(c.mismatch !== undefined ? { mismatch: c.mismatch } : {}),
    ...(c.refusedAt !== undefined ? { refusedAt: c.refusedAt } : {}),
  };
}

function asVersions(value: unknown): CacheVersions | null {
  if (typeof value !== "object" || value === null) return null;
  const c = value as Record<string, unknown>;
  if (!isCount(c.router) || !isCount(c.list) || !isCount(c.settle)) return null;
  return { router: c.router, list: c.list, settle: c.settle };
}

function isCount(value: unknown): value is number {
  return typeof value === "number" && Number.isInteger(value) && value >= 0;
}

function isBoard(value: number): boolean {
  return value >= MIN_BOARD && value <= MAX_BOARD;
}

function isPair(value: unknown): value is [number, number] {
  return (
    Array.isArray(value) &&
    value.length === 2 &&
    Number.isFinite(value[0]) &&
    Number.isFinite(value[1])
  );
}

function isLayout(value: unknown, count: number): value is [number, number][] {
  return Array.isArray(value) && value.length === count && value.every(isPair);
}

// A move before which `nodes` nodes and `halfEdges` half-edges exist.
function isRecordMove(value: unknown, nodes: number, halfEdges: number): value is RecordMove {
  if (typeof value !== "object" || value === null) return false;
  const m = value as Record<string, unknown>;
  if ("index" in m && !("polyline" in m)) {
    return isCount(m.index) && isCount(m.along) && m.along <= 255;
  }
  if (!isCount(m.from) || !isCount(m.to) || m.from >= nodes || m.to >= nodes) return false;
  if ("face" in m) return isPair(m.node) && isFace(m.face, m.from, m.to, nodes, halfEdges);
  if (m.cutOff !== undefined && !isCount(m.cutOff)) return false;
  if (m.cache !== undefined && !isMoveCache(m.cache, m.from, m.to, nodes, halfEdges)) {
    return false;
  }
  return (
    Array.isArray(m.polyline) &&
    m.polyline.length >= 2 &&
    m.polyline.every(isPair) &&
    isPair(m.newNode) &&
    (m.segments === undefined || isSegments(m.segments))
  );
}

function isMoveCache(
  value: unknown,
  from: number,
  to: number,
  nodes: number,
  halfEdges: number,
): boolean {
  if (typeof value !== "object" || value === null) return false;
  const c = value as Record<string, unknown>;
  if (c.index !== undefined && !isCount(c.index)) return false;
  if (c.along !== undefined && !(isCount(c.along) && c.along <= 255)) return false;
  if (c.refused !== undefined && c.refused !== true) return false;
  if (c.shape !== undefined && c.shape !== "topology" && c.shape !== "geometry") return false;
  if (c.topology !== undefined && !isFace(c.topology, from, to, nodes, halfEdges)) return false;
  if (c.shape === "topology" && c.topology === undefined) return false;
  return true;
}

function isSegments(value: unknown): value is Cubic[] {
  const isPoint = (p: unknown): boolean =>
    typeof p === "object" &&
    p !== null &&
    Number.isFinite((p as { x: unknown }).x) &&
    Number.isFinite((p as { y: unknown }).y);
  return (
    Array.isArray(value) &&
    value.length > 0 &&
    value.every(
      (c: unknown) =>
        typeof c === "object" &&
        c !== null &&
        ["p0", "p1", "p2", "p3"].every((k) => isPoint((c as Record<string, unknown>)[k])),
    )
  );
}

function isFace(
  value: unknown,
  from: number,
  to: number,
  nodes: number,
  halfEdges: number,
): value is FaceMove {
  if (typeof value !== "object" || value === null) return false;
  const f = value as Record<string, unknown>;
  const isSpoke = (at: unknown): boolean => at == null || (isCount(at) && at < halfEdges);
  return (
    f.a === from &&
    f.b === to &&
    isSpoke(f.atA) &&
    isSpoke(f.atB) &&
    Array.isArray(f.enclosed) &&
    f.enclosed.every((id) => isCount(id) && id < nodes) &&
    new Set(f.enclosed).size === f.enclosed.length
  );
}

function scaleOf(boardSize: number): number {
  return boardSize / 1000;
}

// Share link layout, little endian: version u8, the router version u8
// from version 4 on, the settle version u8 from version 5 on, the move
// list version u8 in version 6, flags u8 (bit 0 says the AI opens, bit 1
// says the starting positions follow the header), initial nodes u16,
// board size u16, move count u16, cursor u16, the starting positions as
// zigzag varint deltas in link quanta when they are not the default
// layout, then the moves. A version 6 link carries every move as a
// varint, twice its index in the engine's move list for the position
// plus HAS_PLACE when a u8 follows naming the node's place along the
// routed curve. A version 5 link carries per move from and to as varints,
// a shape u8 and the shape. Shapes 0 and 1 carry a varint count, the
// shape's points and the new node, every point as zigzag varint deltas
// from the previous point in link quanta, starting at the origin; a
// stroke counts its points, cubics count segments and carry the first
// start then the other three control points of each. Shape 2 carries the
// face move: the spoke at each end as a varint one above the half-edge id
// with 0 for none, the enclosed node ids as a varint count and varints,
// then the new node as a point. A move travels as whichever encodes
// shorter. A version 1 link has from, to and point count as u16, the
// first point as two u16 tenths, later points as zigzag varint deltas in
// tenths, and the new node as two u16 tenths.
export function encodeRecord(history: History): string {
  const out = new ByteWriter();
  const scale = scaleOf(history.boardSize);
  const quantum = LINK_QUANTUM * scale;
  const layout =
    history.nodes &&
    !sameLayout(history.nodes, undefined, history.initialNodes, history.boardSize, quantum / 2)
      ? history.nodes
      : null;
  const indices = history.moves.length > 0 && history.moves.every((move) => move.indexed);
  out.u8(indices ? LINK_VERSION : SHAPES_LINK_VERSION);
  out.u8(ROUTER_VERSION);
  out.u8(history.settleVersion);
  if (indices) out.u8(MOVE_LIST_VERSION);
  out.u8((history.firstPlayer === "ai" ? FLAG_AI_FIRST : 0) | (layout ? FLAG_LAYOUT : 0));
  out.u16(history.initialNodes);
  out.u16(history.boardSize);
  out.u16(history.moves.length);
  out.u16(history.cursor);
  if (layout) {
    const point = deltaWriter(out, quantum);
    for (const [x, y] of layout) point({ x, y });
  }
  if (indices) {
    for (const move of history.moves) {
      const { index, along } = move.indexed!;
      out.varint(index * 2 + (along > 0 ? HAS_PLACE : 0));
      if (along > 0) out.u8(along);
    }
    return base64url(out.bytes());
  }
  // A settled game stores its splines on the link grid, so the cubics are
  // exact and the thinned stroke, which is not, stays out of its links
  // unless a move has nothing else.
  const exact = history.settleVersion !== 0;
  for (const move of history.moves) {
    out.varint(move.fromNode);
    out.varint(move.toNode);
    const face = encodeFace(move, scale);
    const cubics = encodeCubics(move, scale);
    const shapes = [face, cubics, exact && (face || cubics) ? null : encodeStroke(move, scale)];
    const shortest = shapes.reduce((best, shape) =>
      shape && (!best || shape.length < best.length) ? shape : best,
    );
    if (!shortest) throw new RangeError("a move with no stroke and no topology");
    out.append(shortest);
  }
  return base64url(out.bytes());
}

function encodeFace(move: RecordedMove, scale: number): Uint8Array | null {
  const face = move.topology;
  if (!face || face.a !== move.fromNode || face.b !== move.toNode) return null;
  const out = new ByteWriter();
  out.u8(SHAPE_FACE);
  out.varint(face.atA === undefined ? 0 : face.atA + 1);
  out.varint(face.atB === undefined ? 0 : face.atB + 1);
  out.varint(face.enclosed.length);
  for (const id of face.enclosed) out.varint(id);
  deltaWriter(out, LINK_QUANTUM * scale)(move.newNodePos);
  return out.bytes();
}

function encodeStroke(move: RecordedMove, scale: number): Uint8Array | null {
  if (!drawn(move)) return null;
  const out = new ByteWriter();
  const point = deltaWriter(out, LINK_QUANTUM * scale);
  const stroke = simplify(move.polyline, STROKE_TOLERANCE * scale);
  out.u8(SHAPE_STROKE);
  out.varint(stroke.length);
  for (const p of stroke) point(p);
  point(move.newNodePos);
  return out.bytes();
}

function encodeCubics(move: RecordedMove, scale: number): Uint8Array | null {
  const first = move.segments?.[0];
  if (!move.segments || !first) return null;
  const out = new ByteWriter();
  const point = deltaWriter(out, LINK_QUANTUM * scale);
  out.u8(SHAPE_CUBICS);
  out.varint(move.segments.length);
  point(first.p0);
  for (const cubic of move.segments) {
    point(cubic.p1);
    point(cubic.p2);
    point(cubic.p3);
  }
  point(move.newNodePos);
  return out.bytes();
}

// A link whose topology moves were drawn by another router, which this
// page would draw differently.
export class StaleLink extends Error {
  constructor() {
    super("This link comes from another version of the game and cannot be replayed here");
    this.name = "StaleLink";
  }
}

// Null for a link that does not parse; throws StaleLink for one made for
// another router.
export function decodeRecord(payload: string): GameRecord | null {
  let bytes: Uint8Array;
  try {
    bytes = fromBase64url(payload);
  } catch {
    return null;
  }
  const reader = new ByteReader(bytes);
  try {
    const version = reader.u8();
    if (!LINK_VERSIONS.includes(version)) return null;
    if (version >= FACE_LINK_VERSION && reader.u8() !== ROUTER_VERSION) throw new StaleLink();
    const settleVersion = version >= SHAPES_LINK_VERSION ? reader.u8() : 0;
    if (settleVersion > SETTLE_VERSION) throw new StaleLink();
    const indices = version >= LINK_VERSION;
    if (indices && reader.u8() !== MOVE_LIST_VERSION) throw new StaleLink();
    const flags = reader.u8();
    const initialNodes = reader.u16();
    const boardSize = reader.u16();
    if (!isBoard(boardSize)) return null;
    const count = reader.u16();
    const cursor = reader.u16();
    if (initialNodes > MAX_NODES) return null;
    let nodes: Position[];
    if (version < LAYOUT_LINK_VERSION) {
      nodes = circleLayout(initialNodes, boardSize);
    } else if (flags & FLAG_LAYOUT) {
      const point = deltaReader(reader, LINK_QUANTUM * scaleOf(boardSize));
      nodes = Array.from({ length: initialNodes }, () => {
        const p = point();
        return [p.x, p.y];
      });
    } else {
      nodes = openingLayout(initialNodes, boardSize);
    }
    const moves: RecordMove[] = [];
    for (let m = 0; m < count; m++) {
      const move = indices
        ? readIndexedMove(reader)
        : version === STROKE_LINK_VERSION
          ? readStrokeMove(reader)
          : readCurveMove(reader, boardSize, version >= FACE_LINK_VERSION);
      if (!move) return null;
      moves.push(move);
    }
    if (!reader.done) return null;
    return asRecord({
      version: 1,
      initialNodes,
      boardSize,
      nodes,
      firstPlayer: flags & FLAG_AI_FIRST ? "ai" : "human",
      settleVersion,
      moves,
      cursor,
    });
  } catch (error) {
    if (error instanceof StaleLink) throw error;
    return null;
  }
}

function readIndexedMove(reader: ByteReader): RecordMove {
  const raw = reader.varint();
  const along = raw & HAS_PLACE ? reader.u8() : 0;
  return { index: Math.floor(raw / 2), along };
}

// A move of a version 2 to 5 link; the face shape only from version 4.
function readCurveMove(reader: ByteReader, boardSize: number, faces: boolean): RecordMove | null {
  const from = reader.varint();
  const to = reader.varint();
  const shape = reader.u8();
  if (shape === SHAPE_FACE) {
    if (!faces) return null;
    const spoke = (): number | undefined => {
      const raw = reader.varint();
      return raw === 0 ? undefined : raw - 1;
    };
    const atA = spoke();
    const atB = spoke();
    const enclosed = Array.from({ length: reader.varint() }, () => reader.varint());
    const node = deltaReader(reader, LINK_QUANTUM * scaleOf(boardSize))();
    return { from, to, face: { a: from, atA, b: to, atB, enclosed }, node: [node.x, node.y] };
  }
  const count = reader.varint();
  const scale = scaleOf(boardSize);
  const point = deltaReader(reader, LINK_QUANTUM * scale);
  let polyline: Point[];
  let segments: Cubic[] | undefined;
  if (shape === SHAPE_CUBICS) {
    segments = [];
    let p0 = point();
    for (let i = 0; i < count; i++) {
      const cubic = { p0, p1: point(), p2: point(), p3: point() };
      segments.push(cubic);
      p0 = cubic.p3;
    }
    polyline = flatten(segments, CURVE_TOLERANCE * scale);
  } else if (shape === SHAPE_STROKE) {
    if (count < 2) return null;
    polyline = [];
    for (let i = 0; i < count; i++) polyline.push(point());
  } else {
    return null;
  }
  const newNodePos = point();
  return toRecordMove(
    segments
      ? { fromNode: from, toNode: to, polyline, newNodePos, segments }
      : { fromNode: from, toNode: to, polyline, newNodePos },
  );
}

function readStrokeMove(reader: ByteReader): RecordMove | null {
  const fromNode = reader.u16();
  const toNode = reader.u16();
  const points = reader.u16();
  if (points < 2) return null;
  const polyline: Point[] = [];
  let last = { x: 0, y: 0 };
  for (let i = 0; i < points; i++) {
    const q =
      i === 0
        ? { x: reader.u16(), y: reader.u16() }
        : { x: last.x + unzigzag(reader.varint()), y: last.y + unzigzag(reader.varint()) };
    polyline.push({ x: q.x / TENTHS, y: q.y / TENTHS });
    last = q;
  }
  const newNodePos = { x: reader.u16() / TENTHS, y: reader.u16() / TENTHS };
  return toRecordMove({ fromNode, toNode, polyline, newNodePos });
}

// The point as the link carries it, in whole quanta.
function quantized(p: Point, quantum: number): Point {
  return { x: Math.round(p.x / quantum), y: Math.round(p.y / quantum) };
}

function deltaWriter(out: ByteWriter, quantum: number): (p: Point) => void {
  let last = { x: 0, y: 0 };
  return (p) => {
    const q = quantized(p, quantum);
    out.varint(zigzag(q.x - last.x));
    out.varint(zigzag(q.y - last.y));
    last = q;
  };
}

function deltaReader(reader: ByteReader, quantum: number): () => Point {
  let last = { x: 0, y: 0 };
  return () => {
    last = { x: last.x + unzigzag(reader.varint()), y: last.y + unzigzag(reader.varint()) };
    return { x: last.x * quantum, y: last.y * quantum };
  };
}

// Points on the spline within tolerance of it: each cubic is cut into the
// uniform pieces Wang's bound asks for, so a straight one stays two points.
function flatten(segments: Cubic[], tolerance: number): Point[] {
  const out: Point[] = [];
  segments.forEach((c, i) => {
    if (i === 0) out.push(c.p0);
    const bend = Math.max(secondDifference(c.p0, c.p1, c.p2), secondDifference(c.p1, c.p2, c.p3));
    const pieces = Math.ceil(Math.sqrt((0.75 * bend) / tolerance));
    const n = Math.min(MAX_SUBDIVISIONS, Math.max(1, pieces));
    for (let k = 1; k < n; k++) out.push(pointAt(c, k / n));
    out.push(c.p3);
  });
  return out;
}

function secondDifference(a: Point, b: Point, c: Point): number {
  return Math.hypot(a.x - 2 * b.x + c.x, a.y - 2 * b.y + c.y);
}

function pointAt(c: Cubic, t: number): Point {
  const u = 1 - t;
  const w0 = u * u * u;
  const w1 = 3 * u * u * t;
  const w2 = 3 * u * t * t;
  const w3 = t * t * t;
  return {
    x: w0 * c.p0.x + w1 * c.p1.x + w2 * c.p2.x + w3 * c.p3.x,
    y: w0 * c.p0.y + w1 * c.p1.y + w2 * c.p2.y + w3 * c.p3.y,
  };
}

// The real history, for the engine to say which moves the router redraws;
// null while a move the engine has not drawn is in the list.
export function classification(history: History): Classification | null {
  if (!history.moves.every(drawn)) return null;
  return {
    initialNodes: history.initialNodes,
    boardSize: history.boardSize,
    firstPlayer: history.firstPlayer,
    nodes: history.nodes?.map(([x, y]) => ({ x, y })),
    settleVersion: history.settleVersion,
    moves: history.moves.map(toMove),
    known: history.moves.map((move) => move.topology),
  };
}

// The link's own reading of the game, for replaying on a fresh board the
// way a visitor's engine will.
export function rehearsal(history: History): Rehearsal | null {
  const record = decodeRecord(encodeRecord(history));
  if (!record) return null;
  return {
    routerVersion: ROUTER_VERSION,
    settleVersion: record.settleVersion,
    listVersion: MOVE_LIST_VERSION,
    initialNodes: record.initialNodes,
    boardSize: record.boardSize,
    firstPlayer: record.firstPlayer,
    nodes: record.nodes?.map(([x, y]) => ({ x, y })),
    moves: record.moves.map((move) =>
      isIndexed(move)
        ? { kind: "indexed", index: move.index, along: move.along }
        : hasStroke(move)
          ? { kind: "drawn", mov: fromRecordMove(move) }
          : { kind: "routed", face: move.face, at: { x: move.node[0], y: move.node[1] } },
    ),
  };
}

// Sends a move the visitor's engine refused as geometry from now on.
export function demote(history: History, index: number): boolean {
  const move = history.moves[index];
  if (!move?.topology || !drawn(move)) return false;
  delete move.topology;
  return true;
}

// The engine requests a share needs.
export interface LinkEngine {
  classify(plan: Classification): Promise<Classified>;
  index(plan: Classification): Promise<Indexed>;
  rehearse(plan: Rehearsal): Promise<Rehearsed>;
}

// The game as the link will carry it, worked out on a copy. An index
// link comes first: a game whose every move carries an index from an
// earlier share goes out at once, a game with a move the regenerated
// board refused goes as shapes at once (from the forms an earlier share
// settled for this move list when it has them, else worked out again),
// and otherwise the engine indexes the whole game while replaying it on
// a fresh board, so the visitor's board is that board rather than this
// one; only that call's failure is caught. For shapes the
// engine says which drawn moves the router redraws, then the link
// is replayed as decoded and any topology move the replay refuses,
// or lands its node more than DRIFT_TOLERANCE from the recorded
// one, both nodes rounded as a link carries them, goes as geometry
// until the whole link applies and every routed node holds. The
// replay settles as the visitor's will, so when `settled`, the
// board on screen, is given and a node of the rehearsed board sits
// more than DRIFT_TOLERANCE from it, the latest topology move goes
// as geometry too, and when none is left `mismatch` says the
// visitor's board may differ. A share from a review position gives
// no board, since the rehearsal replays the whole record, and runs
// the record check only. A refused drawn move is blamed on the latest
// topology move before it, whose redraw shifted the board under
// it; when none precedes it the index comes back so the sharer can
// be told. A history the engine refuses to classify, as one left
// with moves past a stopped restore, keeps the face moves the AI
// reported.
export async function prepareLink(
  source: History,
  engine: LinkEngine,
  settled?: Point[],
): Promise<{ history: History; refusedAt?: number; mismatch?: boolean }> {
  const history = source.clone();
  if (history.indexed) return { history };
  const refused = history.moves.some((move) => move.refused);
  if (refused && history.shaped(settled !== undefined)) {
    for (const move of history.moves) if (move.shape === "geometry") delete move.topology;
    const { mismatch, refusedAt } = history.shapes!;
    return {
      history,
      ...(mismatch ? { mismatch } : {}),
      ...(refusedAt !== undefined ? { refusedAt } : {}),
    };
  }
  const settle = (outcome: { refusedAt?: number; mismatch?: boolean }) => {
    for (const move of history.moves) move.shape = move.topology ? "topology" : "geometry";
    history.shapes = {
      moves: history.moves.length,
      ...(settled !== undefined ? { checked: true as const } : {}),
      ...outcome,
    };
    return { history, ...outcome };
  };
  const plan = classification(history);
  if (plan && !refused) {
    const indexed = await engine.index(plan).catch(() => undefined);
    if (indexed) {
      history.moves.forEach((move, i) => setIndexed(move, indexed.moves[i] ?? false));
      if (indexed.refused === undefined) return { history };
    }
  }
  if (plan) {
    try {
      const { topology } = await engine.classify(plan);
      topology.forEach((face, i) => {
        const move = history.moves[i];
        if (move && face && !move.topology) move.topology = face;
      });
    } catch {
      // The AI's own face moves stand; the rehearsal checks them anyway.
    }
  }
  const quantum = LINK_QUANTUM * scaleOf(history.boardSize);
  const drift = (a: Point, b: Point) =>
    distance(quantized(a, quantum), quantized(b, quantum)) * quantum;
  const tolerance = DRIFT_TOLERANCE * scaleOf(history.boardSize);
  for (;;) {
    const link = rehearsal(history);
    if (!link) return settle({});
    const rehearsed = await engine.rehearse(link);
    const { refused: stopped, nodes } = rehearsed;
    if (stopped === undefined) {
      let drifted = false;
      nodes.forEach((node, i) => {
        const move = history.moves[i];
        if (node && move && drift(node, move.newNodePos) > tolerance) {
          drifted = demote(history, i) || drifted;
        }
      });
      if (drifted) continue;
      const apart =
        settled !== undefined &&
        (settled.length !== rehearsed.settled.length ||
          settled.some((node, i) => distance(node, rehearsed.settled[i]!) > tolerance));
      if (!apart) return settle({});
      let latest = history.moves.length - 1;
      while (latest >= 0 && !demote(history, latest)) latest--;
      if (latest < 0) return settle({ mismatch: true });
      continue;
    }
    if (demote(history, stopped)) continue;
    let blamed = stopped - 1;
    while (blamed >= 0 && !demote(history, blamed)) blamed--;
    if (blamed < 0) return settle({ refusedAt: stopped });
  }
}

// Whether the saved record carries an index for each of its first `count`
// moves, so a comparison with a link needs no engine.
export function indicesCached(saved: GameRecord, count: number): boolean {
  return (
    saved.moves.length >= count &&
    currentVersions(saved.cacheVersions) &&
    saved.moves.slice(0, count).every((move) => hasStroke(move) && move.cache?.index !== undefined)
  );
}

// Indexing a saved game routes every move again, so a page opening an
// own link indexes a game this short at once and a longer uncached one
// only after the link has opened.
export const SHORT_GAME = 15;

// Whether an index link could be the saved game but the check has to wait
// for the engine: the headers match, the record's cache does not cover
// the link's moves, and the game is SHORT_GAME moves or longer.
export function ownLinkDeferred(saved: GameRecord, linked: GameRecord): boolean {
  const count = linked.moves.length;
  return (
    count >= SHORT_GAME &&
    linked.moves.every(isIndexed) &&
    saved.moves.length >= count &&
    saved.initialNodes === linked.initialNodes &&
    saved.boardSize === linked.boardSize &&
    saved.firstPlayer === linked.firstPlayer &&
    saved.settleVersion === linked.settleVersion &&
    !indicesCached(saved, count)
  );
}

// Whether an index link names the saved game: the two carry the same
// header and the engine indexes the saved record's first moves to the
// link's, so a game played on past the link still reopens live. A link
// with more moves than the saved game is another game and opens in
// review. A link's indices say nothing about a game without the engine,
// so a saved record that cannot be indexed is another game.
export async function sameIndexedGame(
  saved: GameRecord,
  linked: GameRecord,
  index: (plan: Classification) => Promise<Indexed>,
): Promise<boolean> {
  if (!linked.moves.every(isIndexed) || saved.moves.length < linked.moves.length) return false;
  if (saved.initialNodes !== linked.initialNodes || saved.boardSize !== linked.boardSize) {
    return false;
  }
  if (saved.firstPlayer !== linked.firstPlayer || saved.settleVersion !== linked.settleVersion) {
    return false;
  }
  const quantum = LINK_QUANTUM * scaleOf(saved.boardSize);
  if (!sameLayout(saved.nodes, linked.nodes, saved.initialNodes, saved.boardSize, quantum)) {
    return false;
  }
  const shared = linked.moves.length;
  const same = (moves: IndexedMove[]) =>
    moves.length === shared &&
    moves.every((move, i) => {
      const other = linked.moves[i];
      return (
        other !== undefined &&
        isIndexed(other) &&
        other.index === move.index &&
        other.along === move.along
      );
    });
  const cached = History.from({ ...saved, moves: saved.moves.slice(0, shared), cursor: shared });
  if (cached.indexed) return same(cached.moves.map((move) => move.indexed!));
  const plan = classification(cached);
  if (!plan) return false;
  const indexed = await index(plan).catch(() => undefined);
  return indexed !== undefined && indexed.refused === undefined && same(indexed.moves);
}

export function shareUrl(history: History): string {
  return (
    location.origin + location.pathname + location.search + HASH_PREFIX + encodeRecord(history)
  );
}

export function recordFromHash(hash: string): GameRecord | null {
  if (!hash.startsWith(HASH_PREFIX)) return null;
  return decodeRecord(hash.slice(HASH_PREFIX.length));
}

export function dropHash(): void {
  if (location.hash) history.replaceState(null, "", location.pathname + location.search);
}

// Douglas-Peucker: keeps the endpoints and every point farther than the
// tolerance from the chord between kept neighbours.
export function simplify(points: Point[], tolerance: number): Point[] {
  const first = points[0];
  const last = points[points.length - 1];
  if (!first || !last || points.length <= 2) return points;
  let worst = 0;
  let worstIndex = 0;
  for (let i = 1; i < points.length - 1; i++) {
    const point = points[i];
    if (!point) continue;
    const d = distanceToSegment(point, first, last);
    if (d > worst) {
      worst = d;
      worstIndex = i;
    }
  }
  if (worst <= tolerance) return [first, last];
  const head = simplify(points.slice(0, worstIndex + 1), tolerance);
  const tail = simplify(points.slice(worstIndex), tolerance);
  return [...head, ...tail.slice(1)];
}

function distanceToSegment(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length2 = dx * dx + dy * dy;
  const t =
    length2 === 0 ? 0 : Math.min(1, Math.max(0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / length2));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// True when one game is the other played further, so a link to my own game
// reopens the live game rather than a review of it. Endpoints and the new
// node identify a move; a link of either version rounds a stroke's node by
// less than one quantum, and a routed move's node is wherever the visitor's
// router put it, within the drift the share allowed.
export function sameGame(a: GameRecord, b: GameRecord): boolean {
  if (a.initialNodes !== b.initialNodes || a.boardSize !== b.boardSize) return false;
  if (a.firstPlayer !== b.firstPlayer || a.settleVersion !== b.settleVersion) return false;
  const scale = scaleOf(a.boardSize);
  const quantum = LINK_QUANTUM * scale;
  if (!sameLayout(a.nodes, b.nodes, a.initialNodes, a.boardSize, quantum)) return false;
  const shared = Math.min(a.moves.length, b.moves.length);
  for (let i = 0; i < shared; i++) {
    if (!sameMove(a.moves[i], b.moves[i], quantum, DRIFT_TOLERANCE * scale)) return false;
  }
  return true;
}

function sameMove(
  a: RecordMove | undefined,
  b: RecordMove | undefined,
  quantum: number,
  drift: number,
): boolean {
  if (!a || !b) return false;
  if (isIndexed(a) || isIndexed(b)) {
    return isIndexed(a) && isIndexed(b) && a.index === b.index && a.along === b.along;
  }
  if (a.from !== b.from || a.to !== b.to) return false;
  const [ax, ay] = nodeOf(a);
  const [bx, by] = nodeOf(b);
  return Math.hypot(ax - bx, ay - by) <= (hasStroke(a) && hasStroke(b) ? quantum : drift);
}

function nodeOf(move: FixtureMove | TopologyMove): [number, number] {
  return hasStroke(move) ? move.newNode : move.node;
}

function zigzag(n: number): number {
  return n < 0 ? -2 * n - 1 : 2 * n;
}

function unzigzag(n: number): number {
  return n % 2 === 0 ? n / 2 : -(n + 1) / 2;
}

class ByteWriter {
  private buffer = new Uint8Array(1024);
  private length = 0;

  u8(value: number): void {
    if (this.length === this.buffer.length) {
      const grown = new Uint8Array(this.buffer.length * 2);
      grown.set(this.buffer);
      this.buffer = grown;
    }
    this.buffer[this.length++] = value & 0xff;
  }

  u16(value: number): void {
    if (value < 0 || value > 0xffff) throw new RangeError(`${value} does not fit u16`);
    this.u8(value);
    this.u8(value >> 8);
  }

  varint(value: number): void {
    let v = value;
    while (v >= 0x80) {
      this.u8((v & 0x7f) | 0x80);
      v = Math.floor(v / 0x80);
    }
    this.u8(v);
  }

  append(bytes: Uint8Array): void {
    for (const byte of bytes) this.u8(byte);
  }

  bytes(): Uint8Array {
    return this.buffer.slice(0, this.length);
  }
}

class ByteReader {
  private offset = 0;

  constructor(private readonly bytes: Uint8Array) {}

  get done(): boolean {
    return this.offset === this.bytes.length;
  }

  u8(): number {
    const value = this.bytes[this.offset];
    if (value === undefined) throw new RangeError("share link ends early");
    this.offset++;
    return value;
  }

  u16(): number {
    return this.u8() | (this.u8() << 8);
  }

  varint(): number {
    let value = 0;
    let shift = 1;
    for (;;) {
      const byte = this.u8();
      value += (byte & 0x7f) * shift;
      if (byte < 0x80) return value;
      shift *= 0x80;
    }
  }
}

function base64url(bytes: Uint8Array): string {
  let binary = "";
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return btoa(binary).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

function fromBase64url(text: string): Uint8Array {
  const base64 = text.replace(/-/g, "+").replace(/_/g, "/");
  const binary = atob(base64 + "=".repeat((4 - (base64.length % 4)) % 4));
  return Uint8Array.from(binary, (c) => c.charCodeAt(0));
}
