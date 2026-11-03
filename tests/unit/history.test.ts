import { readdirSync, readFileSync } from "node:fs";
import { basename, join } from "node:path";
import { describe, expect, it } from "vitest";
import type { Cubic, Point } from "../../pkg/sprouts";
import {
  decodeRecord,
  encodeRecord,
  History,
  LINK_QUANTUM,
  sameGame,
  STROKE_TOLERANCE,
  type GameRecord,
  type RecordedMove,
} from "../../web/history";
import type { Fixture, FixtureMove } from "../../web/record";

// Header: version, flags, initial nodes, board size, move count and cursor,
// then each move's from and to as one-byte varints before its shape byte.
const SHAPE_OFFSET = 12;
const SHAPE_STROKE = 0;
const SHAPE_CUBICS = 1;
const VERSION_1_PAYLOAD =
  "AQAEAPQBAgACAAAAAQACABwMxAmvCbAJ8ArwCgIAAAAFAGwHxAn4A88BpgpR3gIe5AGEAvsKMwk";

function bytesOf(payload: string): Uint8Array {
  return new Uint8Array(Buffer.from(payload, "base64url"));
}

function payloadOf(bytes: Uint8Array): string {
  return Buffer.from(bytes).toString("base64url");
}

function game(move: RecordedMove, board = 1000): History {
  const history = new History(4, board);
  history.push(move);
  return history;
}

function at(x: number, y: number): Point {
  return { x, y };
}

// A wobbly arc from (300, 500) to (700, 500) the way a finger draws one.
function handDrawn(): Point[] {
  const points: Point[] = [];
  for (let i = 0; i <= 60; i++) {
    const t = i / 60;
    const wobble = Math.sin(t * 40) * Math.sin(Math.PI * t) * 1.2;
    points.push(at(300 + 400 * t, 500 - 160 * Math.sin(Math.PI * t) + wobble));
  }
  return points;
}

function arcCubics(): Cubic[] {
  return [
    { p0: at(300, 500), p1: at(360, 390), p2: at(440, 340), p3: at(500, 340) },
    { p0: at(500, 340), p1: at(560, 340), p2: at(640, 390), p3: at(700, 500) },
  ];
}

function only(record: GameRecord | null) {
  const move = record?.moves[0];
  if (!move) throw new Error("no move decoded");
  return move;
}

describe("the share link codec", () => {
  it("sends a hand-drawn move with a fit as cubics", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: handDrawn(),
      newNodePos: at(500, 340),
      segments: arcCubics(),
    };
    const payload = encodeRecord(game(move));
    expect(bytesOf(payload)[SHAPE_OFFSET]).toBe(SHAPE_CUBICS);
    const decoded = only(decodeRecord(payload));
    expect(decoded.from).toBe(0);
    expect(decoded.to).toBe(1);
    expect(decoded.polyline.length).toBeGreaterThan(2);
    expect(decoded.polyline[0]).toEqual([300, 500]);
    expect(decoded.polyline.at(-1)).toEqual([700, 500]);
    expect(decoded.newNode).toEqual([500, 340]);
  });

  it("sends the same move without a fit as its thinned stroke", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: handDrawn(),
      newNodePos: at(500, 340),
    };
    const payload = encodeRecord(game(move));
    expect(bytesOf(payload)[SHAPE_OFFSET]).toBe(SHAPE_STROKE);
    const decoded = only(decodeRecord(payload));
    expect(decoded.polyline.length).toBeGreaterThan(2);
    expect(decoded.polyline.length).toBeLessThan(30);
    expect(decoded.polyline[0]).toEqual([300, 500]);
    expect(decoded.polyline.at(-1)).toEqual([700, 500]);
  });

  it("keeps a sparse polyline as a stroke when its cubics encode longer", () => {
    const polyline = [at(100, 100), at(200, 150), at(300, 100), at(400, 150), at(500, 100)];
    const segments: Cubic[] = [];
    for (let i = 0; i + 1 < polyline.length; i++) {
      const a = polyline[i];
      const b = polyline[i + 1];
      if (a && b) segments.push({ p0: a, p1: a, p2: b, p3: b });
    }
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 2,
      polyline,
      newNodePos: at(300, 100),
      segments,
    };
    const payload = encodeRecord(game(move));
    expect(bytesOf(payload)[SHAPE_OFFSET]).toBe(SHAPE_STROKE);
    expect(only(decodeRecord(payload)).polyline).toEqual(polyline.map((p) => [p.x, p.y]));
  });

  it("round-trips a two point stroke to within a unit", () => {
    const move: RecordedMove = {
      fromNode: 2,
      toNode: 3,
      polyline: [at(120.4, 880.6), at(640.2, 240.7)],
      newNodePos: at(380.3, 560.6),
    };
    const decoded = only(decodeRecord(encodeRecord(game(move))));
    expect(decoded.polyline).toEqual([
      [120, 881],
      [640, 241],
    ]);
    expect(decoded.newNode).toEqual([380, 561]);
  });

  it("round-trips a self loop", () => {
    const move: RecordedMove = {
      fromNode: 1,
      toNode: 1,
      polyline: [at(500, 500), at(560, 440), at(620, 500), at(560, 560), at(500, 500)],
      newNodePos: at(620, 500),
    };
    const decoded = only(decodeRecord(encodeRecord(game(move))));
    expect(decoded.from).toBe(1);
    expect(decoded.to).toBe(1);
    expect(decoded.polyline).toHaveLength(5);
  });

  it("carries a stroke that leaves the board through negative deltas", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: [at(100, 500), at(-40, 520), at(-45, 480), at(100, 460)],
      newNodePos: at(-42, 500),
    };
    const decoded = only(decodeRecord(encodeRecord(game(move))));
    expect(decoded.polyline[1]).toEqual([-40, 520]);
    expect(decoded.polyline[2]).toEqual([-45, 480]);
    expect(decoded.newNode).toEqual([-42, 500]);
  });

  it("scales the quantum with the board", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: [at(100.26, 200), at(300.74, 200)],
      newNodePos: at(200.26, 200),
    };
    const decoded = only(decodeRecord(encodeRecord(game(move, 500))));
    expect(decoded.polyline).toEqual([
      [100.5, 200],
      [300.5, 200],
    ]);
  });

  it("rejects the reserved shape, trailing bytes and a truncated link", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: [at(100, 100), at(400, 400)],
      newNodePos: at(250, 250),
    };
    const bytes = bytesOf(encodeRecord(game(move)));
    expect(decodeRecord(payloadOf(bytes))).not.toBeNull();
    const reserved = bytes.slice();
    reserved[SHAPE_OFFSET] = 2;
    expect(decodeRecord(payloadOf(reserved))).toBeNull();
    expect(decodeRecord(payloadOf(new Uint8Array([...bytes, 0])))).toBeNull();
    expect(decodeRecord(payloadOf(bytes.slice(0, -1)))).toBeNull();
    expect(decodeRecord("not base64url!")).toBeNull();
  });

  it("decodes a version 1 link", () => {
    const record = decodeRecord(VERSION_1_PAYLOAD);
    expect(record?.boardSize).toBe(500);
    expect(record?.moves.map((m) => [m.from, m.to])).toEqual([
      [0, 1],
      [2, 0],
    ]);
    expect(record?.cursor).toBe(2);
  });
});

function versionOneRecord(): GameRecord {
  const record = decodeRecord(VERSION_1_PAYLOAD);
  if (!record) throw new Error("the version 1 link does not decode");
  return record;
}

describe("sameGame", () => {
  const linked = versionOneRecord();

  // The link rounds a node by at most half a quantum per axis, a quarter
  // unit on this 500 board.
  function nudged(dx: number, dy: number): GameRecord {
    return {
      ...linked,
      moves: linked.moves.map((m) => ({
        ...m,
        newNode: [m.newNode[0] + dx, m.newNode[1] + dy] as [number, number],
      })),
    };
  }

  it("matches a saved game whose nodes the link rounded", () => {
    expect(sameGame(nudged(0.24, -0.24), linked)).toBe(true);
    expect(sameGame(nudged(0.24, -0.24), { ...linked, moves: linked.moves.slice(0, 1) })).toBe(
      true,
    );
  });

  it("tells apart a node a quantum away or a different endpoint", () => {
    expect(sameGame(nudged(0.6, 0), linked)).toBe(false);
    const other = { ...linked, moves: linked.moves.map((m) => ({ ...m, to: m.to + 1 })) };
    expect(sameGame(other, linked)).toBe(false);
  });
});

function distanceToSegment(p: [number, number], a: [number, number], b: [number, number]): number {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const length2 = dx * dx + dy * dy;
  const t =
    length2 === 0
      ? 0
      : Math.min(1, Math.max(0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2));
  return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
}

function distanceToPolyline(p: [number, number], polyline: [number, number][]): number {
  let best = Infinity;
  for (let i = 0; i + 1 < polyline.length; i++) {
    const a = polyline[i];
    const b = polyline[i + 1];
    if (a && b) best = Math.min(best, distanceToSegment(p, a, b));
  }
  return best;
}

const fixtures = ["fixtures/games", "fixtures/saved"].flatMap((dir) =>
  readdirSync(dir)
    .filter((f) => f.endsWith(".json"))
    .map((f) => join(dir, f)),
);

// A link rounds each point to the quantum and thins a stroke to the
// tolerance, so a decoded stroke stays within both of the original and
// the original within both of it.
describe("every recorded game through the link", () => {
  for (const file of fixtures) {
    it(`${basename(file, ".json")} keeps its geometry`, () => {
      const fixture = JSON.parse(readFileSync(file, "utf8")) as Fixture;
      const history = History.from({
        firstPlayer: "human",
        cursor: fixture.moves.length,
        ...fixture,
        version: 1,
      });
      const decoded = decodeRecord(encodeRecord(history));
      expect(decoded).not.toBeNull();
      expect(decoded?.moves).toHaveLength(fixture.moves.length);
      const scale = fixture.boardSize / 1000;
      const quantum = LINK_QUANTUM * scale;
      const slack = STROKE_TOLERANCE * scale + quantum;
      const rounded = (p: [number, number]): [number, number] => [
        Math.round(p[0] / quantum) * quantum,
        Math.round(p[1] / quantum) * quantum,
      ];
      fixture.moves.forEach((move: FixtureMove, i) => {
        const got = decoded?.moves[i];
        if (!got) throw new Error(`move ${i + 1} missing`);
        expect(got.from).toBe(move.from);
        expect(got.to).toBe(move.to);
        expect(got.newNode).toEqual(rounded(move.newNode));
        const first = move.polyline[0];
        const last = move.polyline[move.polyline.length - 1];
        if (!first || !last) throw new Error(`move ${i + 1} has no stroke`);
        expect(got.polyline[0]).toEqual(rounded(first));
        expect(got.polyline[got.polyline.length - 1]).toEqual(rounded(last));
        for (const p of got.polyline) {
          expect(distanceToPolyline(p, move.polyline)).toBeLessThanOrEqual(slack);
        }
        for (const p of move.polyline) {
          expect(distanceToPolyline(p, got.polyline)).toBeLessThanOrEqual(slack);
        }
      });
    });
  }
});
