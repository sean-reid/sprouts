import type { Cubic, Line, Move, Player, Point } from "../pkg/sprouts";
import type { Fixture, FixtureMove } from "./record";

// A replay fixture plus the fields a resumed session needs, so a saved
// game loads in the bench unchanged.
export interface GameRecord extends Fixture {
  version: 1;
  firstPlayer: Player;
  cursor: number;
}

// A move with the spline the engine fitted to it, once a snapshot has
// reported one; the saved record keeps only the stroke.
export interface RecordedMove extends Move {
  segments?: Cubic[];
}

export const STORAGE_KEY = "sprouts.game";
const HASH_PREFIX = "#game=";
const RECORD_VERSION = 1;
// Link version 1 carries thinned strokes, version 2 the fitted cubics.
const LINK_VERSION = 2;
const STROKE_LINK_VERSION = 1;
// Coordinates travel as tenths of a world unit in a version 1 link.
const TENTHS = 10;
// Geometry below is in units of a 1000 board and scales with the board.
// The fit tolerance of src/curve.rs: the engine keeps a curve within it of
// the stroke, so a curve flattened at it refits inside the error the
// engine already accepts. A stroke point closer than STROKE_TOLERANCE to
// the chord of its neighbours is left out of the link.
const CURVE_TOLERANCE = 1.5;
export const STROKE_TOLERANCE = 1.5;
// Per move shape in the link; 2 is reserved for the face-model move.
const SHAPE_STROKE = 0;
const SHAPE_CUBICS = 1;
// Points travel in whole units, which moves one by at most 0.71.
export const LINK_QUANTUM = 1;
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

  constructor(
    public initialNodes: number,
    public boardSize: number,
    public firstPlayer: Player = "human",
  ) {}

  static from(record: GameRecord): History {
    const history = new History(record.initialNodes, record.boardSize, record.firstPlayer);
    history.moves = record.moves.map(fromFixtureMove);
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

  // A move made from the cursor discards whatever was ahead of it.
  push(move: Move): void {
    this.moves.length = this.cursor;
    this.moves.push(cloneMove(move));
    this.cursor = this.moves.length;
  }

  truncate(): void {
    this.moves.length = this.cursor;
  }

  // Takes the fitted spline of every line the engine holds for a move that
  // has none yet; the engine's lines are the moves up to the cursor.
  adopt(lines: Line[]): void {
    lines.forEach((line, i) => {
      const move = this.moves[i];
      if (!move || move.segments) return;
      if (line.fromNode === move.fromNode && line.toNode === move.toNode) {
        move.segments = line.segments;
      }
    });
  }

  label(index: number): string {
    const move = this.moves[index];
    if (!move) return "";
    const who = this.moverOf(index) === "human" ? "You" : "AI";
    return `${index + 1}. ${who} ${move.fromNode} to ${move.toNode}`;
  }

  record(): GameRecord {
    return {
      version: 1,
      initialNodes: this.initialNodes,
      boardSize: this.boardSize,
      firstPlayer: this.firstPlayer,
      moves: this.moves.map(toFixtureMove),
      cursor: this.cursor,
    };
  }
}

function cloneMove(move: RecordedMove): RecordedMove {
  const clone: RecordedMove = toMove(move);
  if (move.segments) clone.segments = move.segments;
  return clone;
}

// The move as the engine takes it, without the segments the page keeps.
export function toMove(move: RecordedMove): Move {
  return {
    fromNode: move.fromNode,
    toNode: move.toNode,
    polyline: move.polyline.map((p) => ({ x: p.x, y: p.y })),
    newNodePos: { x: move.newNodePos.x, y: move.newNodePos.y },
  };
}

function toFixtureMove(move: Move): FixtureMove {
  return {
    from: move.fromNode,
    to: move.toNode,
    polyline: move.polyline.map((p): [number, number] => [p.x, p.y]),
    newNode: [move.newNodePos.x, move.newNodePos.y],
  };
}

function fromFixtureMove(move: FixtureMove): Move {
  return {
    fromNode: move.from,
    toNode: move.to,
    polyline: move.polyline.map(([x, y]) => ({ x, y })),
    newNodePos: { x: move.newNode[0], y: move.newNode[1] },
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
  if (!v.moves.every((m, i) => isMove(m) && m.from < nodes + i && m.to < nodes + i)) {
    return null;
  }
  if (v.cursor > v.moves.length) return null;
  return {
    version: 1,
    initialNodes: v.initialNodes,
    boardSize: v.boardSize,
    firstPlayer: v.firstPlayer,
    moves: v.moves,
    cursor: v.cursor,
  };
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

function isMove(value: unknown): value is FixtureMove {
  if (typeof value !== "object" || value === null) return false;
  const m = value as Record<string, unknown>;
  return (
    isCount(m.from) &&
    isCount(m.to) &&
    Array.isArray(m.polyline) &&
    m.polyline.length >= 2 &&
    m.polyline.every(isPair) &&
    isPair(m.newNode)
  );
}

function scaleOf(boardSize: number): number {
  return boardSize / 1000;
}

// Share link layout, little endian: version u8, flags u8 (bit 0 is AI
// first), initial nodes u16, board size u16, move count u16, cursor u16,
// then per move: from and to as varints, a shape u8, a varint count, the
// shape's points and the new node, every point as zigzag varint deltas
// from the previous point in link quanta, starting at the origin. A stroke
// counts its points; cubics count segments and carry the first start then
// the other three control points of each. A move travels as whichever
// encodes shorter. A version 1 link has from, to and point count as u16, the first
// point as two u16 tenths, later points as zigzag varint deltas in tenths,
// and the new node as two u16 tenths.
export function encodeRecord(history: History): string {
  const out = new ByteWriter();
  out.u8(LINK_VERSION);
  out.u8(history.firstPlayer === "ai" ? 1 : 0);
  out.u16(history.initialNodes);
  out.u16(history.boardSize);
  out.u16(history.moves.length);
  out.u16(history.cursor);
  const scale = scaleOf(history.boardSize);
  for (const move of history.moves) {
    out.varint(move.fromNode);
    out.varint(move.toNode);
    const stroke = encodeStroke(move, scale);
    const cubics = encodeCubics(move, scale);
    out.append(cubics && cubics.length < stroke.length ? cubics : stroke);
  }
  return base64url(out.bytes());
}

function encodeStroke(move: RecordedMove, scale: number): Uint8Array {
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
    if (version !== LINK_VERSION && version !== STROKE_LINK_VERSION) return null;
    const flags = reader.u8();
    const initialNodes = reader.u16();
    const boardSize = reader.u16();
    if (!isBoard(boardSize)) return null;
    const count = reader.u16();
    const cursor = reader.u16();
    const moves: Move[] = [];
    for (let m = 0; m < count; m++) {
      const move =
        version === LINK_VERSION ? readCurveMove(reader, boardSize) : readStrokeMove(reader);
      if (!move) return null;
      moves.push(move);
    }
    if (!reader.done) return null;
    return asRecord({
      version: 1,
      initialNodes,
      boardSize,
      firstPlayer: flags & 1 ? "ai" : "human",
      moves: moves.map(toFixtureMove),
      cursor,
    });
  } catch {
    return null;
  }
}

function readCurveMove(reader: ByteReader, boardSize: number): Move | null {
  const fromNode = reader.varint();
  const toNode = reader.varint();
  const shape = reader.u8();
  const count = reader.varint();
  const scale = scaleOf(boardSize);
  const point = deltaReader(reader, LINK_QUANTUM * scale);
  let polyline: Point[];
  if (shape === SHAPE_CUBICS) {
    const segments: Cubic[] = [];
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
  return { fromNode, toNode, polyline, newNodePos: point() };
}

function readStrokeMove(reader: ByteReader): Move | null {
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
  return { fromNode, toNode, polyline, newNodePos };
}

function deltaWriter(out: ByteWriter, quantum: number): (p: Point) => void {
  let last = { x: 0, y: 0 };
  return (p) => {
    const q = { x: Math.round(p.x / quantum), y: Math.round(p.y / quantum) };
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
// node identify a move; a link of either version rounds the node by less
// than one quantum.
export function sameGame(a: GameRecord, b: GameRecord): boolean {
  if (a.initialNodes !== b.initialNodes || a.boardSize !== b.boardSize) return false;
  if (a.firstPlayer !== b.firstPlayer) return false;
  const quantum = LINK_QUANTUM * scaleOf(a.boardSize);
  const shared = Math.min(a.moves.length, b.moves.length);
  for (let i = 0; i < shared; i++) {
    if (!sameMove(a.moves[i], b.moves[i], quantum)) return false;
  }
  return true;
}

function sameMove(
  a: FixtureMove | undefined,
  b: FixtureMove | undefined,
  quantum: number,
): boolean {
  if (!a || !b) return false;
  if (a.from !== b.from || a.to !== b.to) return false;
  return Math.hypot(a.newNode[0] - b.newNode[0], a.newNode[1] - b.newNode[1]) <= quantum;
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
