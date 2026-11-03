import type { Move, Player, Point } from "../pkg/sprouts";
import type { Fixture, FixtureMove } from "./record";

// A replay fixture plus the fields a resumed session needs, so a saved
// game loads in the bench unchanged.
export interface GameRecord extends Fixture {
  version: 1;
  firstPlayer: Player;
  cursor: number;
}

export const STORAGE_KEY = "sprouts.game";
const HASH_PREFIX = "#game=";
const FORMAT_VERSION = 1;
// Coordinates travel as tenths of a world unit in the share link.
const TENTHS = 10;
// A stroke point this close to the chord of its neighbours, in units of a
// 1000 board, is left out of the link; the tightest clearance is 3 units.
const LINK_TOLERANCE = 0.25;
// Bounds for a record that arrived by link; a Sprouts game lasts at most
// 3n - 1 moves from n nodes. Endpoints must name nodes that exist before
// the move; points are unbounded since strokes may leave the board.
const MAX_NODES = 500;
const MIN_BOARD = 100;
const MAX_BOARD = 6000;

export class History {
  moves: Move[] = [];
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

function cloneMove(move: Move): Move {
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
  if (v.version !== FORMAT_VERSION) return null;
  if (!isCount(v.initialNodes) || !isCount(v.boardSize) || !isCount(v.cursor)) return null;
  if (v.initialNodes < 1 || v.initialNodes > MAX_NODES) return null;
  if (v.boardSize < MIN_BOARD || v.boardSize > MAX_BOARD) return null;
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

// Share link layout, little endian: version u8, flags u8 (bit 0 is AI
// first), initial nodes u16, board size u16, move count u16, cursor u16,
// then per move: from u16, to u16, point count u16, the first point as two
// u16 tenths, every later point as zigzag varint deltas in tenths, and the
// new node as two u16 tenths.
export function encodeRecord(record: GameRecord): string {
  const out = new ByteWriter();
  out.u8(FORMAT_VERSION);
  out.u8(record.firstPlayer === "ai" ? 1 : 0);
  out.u16(record.initialNodes);
  out.u16(record.boardSize);
  out.u16(record.moves.length);
  out.u16(record.cursor);
  const tolerance = (LINK_TOLERANCE * record.boardSize) / 1000;
  for (const move of record.moves.map(fromFixtureMove)) {
    const polyline = simplify(move.polyline, tolerance);
    out.u16(move.fromNode);
    out.u16(move.toNode);
    out.u16(polyline.length);
    let last = { x: 0, y: 0 };
    polyline.forEach((point, i) => {
      const q = quantize(point);
      if (i === 0) {
        out.u16(q.x);
        out.u16(q.y);
      } else {
        out.varint(zigzag(q.x - last.x));
        out.varint(zigzag(q.y - last.y));
      }
      last = q;
    });
    const n = quantize(move.newNodePos);
    out.u16(n.x);
    out.u16(n.y);
  }
  return base64url(out.bytes());
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
    if (reader.u8() !== FORMAT_VERSION) return null;
    const flags = reader.u8();
    const initialNodes = reader.u16();
    const boardSize = reader.u16();
    const count = reader.u16();
    const cursor = reader.u16();
    const moves: Move[] = [];
    for (let m = 0; m < count; m++) {
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
      moves.push({ fromNode, toNode, polyline, newNodePos });
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

export function shareUrl(record: GameRecord): string {
  return location.origin + location.pathname + location.search + HASH_PREFIX + encodeRecord(record);
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
// node identify a move; the link carries a thinned polyline.
export function sameGame(a: GameRecord, b: GameRecord): boolean {
  if (a.initialNodes !== b.initialNodes || a.boardSize !== b.boardSize) return false;
  if (a.firstPlayer !== b.firstPlayer) return false;
  const shared = Math.min(a.moves.length, b.moves.length);
  for (let i = 0; i < shared; i++) {
    if (moveKey(a.moves[i]) !== moveKey(b.moves[i])) return false;
  }
  return true;
}

function moveKey(move: FixtureMove | undefined): string {
  if (!move) return "";
  const n = quantize({ x: move.newNode[0], y: move.newNode[1] });
  return [move.from, move.to, n.x, n.y].join(",");
}

function quantize(point: Point): { x: number; y: number } {
  const clamp = (v: number): number => Math.min(0xffff, Math.max(0, Math.round(v * TENTHS)));
  return { x: clamp(point.x), y: clamp(point.y) };
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
