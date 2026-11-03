import { readdirSync, readFileSync } from "node:fs";
import { basename, join } from "node:path";
import { describe, expect, it } from "vitest";
import type { Cubic, FaceMove, Line, Point, Rehearsal } from "../../pkg/sprouts";
import {
  decodeRecord,
  demote,
  encodeRecord,
  hasStroke,
  History,
  LINK_QUANTUM,
  loadSaved,
  pendingFace,
  prepareLink,
  recordFromHash,
  rehearsal,
  ROUTER_VERSION,
  sameGame,
  StaleLink,
  STROKE_TOLERANCE,
  type GameRecord,
  type LinkEngine,
  type RecordedMove,
  type RecordMove,
  type TopologyMove,
} from "../../web/history";
import { circleLayout, openingLayout } from "../../web/layout";
import type { Fixture, FixtureMove } from "../../web/record";

// Header: version, router version, flags, initial nodes, board size, move
// count and cursor, then each move's from and to as one-byte varints
// before its shape byte.
const SHAPE_OFFSET = 13;
const FLAGS_OFFSET = 2;
const SHAPE_STROKE = 0;
const SHAPE_CUBICS = 1;
const SHAPE_FACE = 2;
const LINK_VERSION = 4;
const VERSION_1_PAYLOAD =
  "AQAEAPQBAgACAAAAAQACABwMxAmvCbAJ8ArwCgIAAAAFAGwHxAn4A88BpgpR3gIe5AGEAvsKMwk";
// The first exchange of fixtures/saved/first-exchange.json as a version 2
// link, which carried no layout.
const VERSION_2_PAYLOAD = "AgAEAOgDAgACAAABAALYCegH7wHwAXh3AgAAA_gF6AeaAyVGJkUl";
// The same exchange as a version 3 link, with its layout.
const VERSION_3_PAYLOAD =
  "AwIEAOgDAgACANgJ6AfvAfAB7wHvAfAB7wEAAQAC2AnoB-8B8AF4dwIAAAP4BegHmgMlRiZFJQ";
// A version 4 link of two moves: a stroke from 0 to 1, then a loop on 2
// carried as its face move, leaving spoke 3 and enclosing nodes 0 and 4,
// with its node at (620, 500).
const VERSION_4_PAYLOAD = "BAEABADoAwIAAgAAAQAC2AToB6AGAI8DAAICAgQEAgAE2AnoBw";
const LOOP_FACE: FaceMove = { a: 2, atA: 3, b: 2, atB: 3, enclosed: [0, 4] };

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

function stroke(move: RecordMove | undefined): FixtureMove {
  if (!move) throw new Error("no move decoded");
  if (!hasStroke(move)) throw new Error("the move decoded as topology");
  return move;
}

function only(record: GameRecord | null) {
  return stroke(record?.moves[0]);
}

function faceOf(move: RecordMove | undefined): FaceMove {
  if (!move) throw new Error("no move decoded");
  if (hasStroke(move)) throw new Error("the move decoded with a stroke");
  return move.face;
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

  it("rejects an unknown shape, trailing bytes and a truncated link", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: [at(100, 100), at(400, 400)],
      newNodePos: at(250, 250),
    };
    const bytes = bytesOf(encodeRecord(game(move)));
    expect(decodeRecord(payloadOf(bytes))).not.toBeNull();
    const unknown = bytes.slice();
    unknown[SHAPE_OFFSET] = 3;
    expect(decodeRecord(payloadOf(unknown))).toBeNull();
    expect(decodeRecord(payloadOf(new Uint8Array([...bytes, 0])))).toBeNull();
    expect(decodeRecord(payloadOf(bytes.slice(0, -1)))).toBeNull();
    expect(decodeRecord("not base64url!")).toBeNull();
  });

  it("sends a move the router redraws as its face move alone", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: handDrawn(),
      newNodePos: at(500, 340),
      segments: arcCubics(),
      topology: { a: 0, atA: undefined, b: 1, atB: undefined, enclosed: [] },
    };
    const payload = encodeRecord(game(move));
    expect(bytesOf(payload)[SHAPE_OFFSET]).toBe(SHAPE_FACE);
    expect(payload.length).toBeLessThanOrEqual(30);
    expect(faceOf(decodeRecord(payload)?.moves[0])).toEqual(move.topology);
  });

  it("keeps the stroke when the topology names other endpoints", () => {
    const move: RecordedMove = {
      fromNode: 0,
      toNode: 1,
      polyline: [at(100, 100), at(400, 400)],
      newNodePos: at(250, 250),
      topology: { a: 1, atA: undefined, b: 0, atB: undefined, enclosed: [] },
    };
    const payload = encodeRecord(game(move));
    expect(bytesOf(payload)[SHAPE_OFFSET]).toBe(SHAPE_STROKE);
    expect(only(decodeRecord(payload)).newNode).toEqual([250, 250]);
  });

  it("decodes a version 4 link with a face move and rejects it in version 3", () => {
    const record = decodeRecord(VERSION_4_PAYLOAD);
    expect(record?.moves).toHaveLength(2);
    expect(only(record).polyline).toEqual([
      [300, 500],
      [700, 500],
    ]);
    expect(faceOf(record?.moves[1])).toEqual(LOOP_FACE);
    const bytes = bytesOf(VERSION_4_PAYLOAD);
    expect(bytes[0]).toBe(LINK_VERSION);
    expect(bytes[1]).toBe(ROUTER_VERSION);
    // A version 3 link has no router byte and no face shape.
    const older = new Uint8Array([3, ...bytes.slice(2)]);
    expect(decodeRecord(payloadOf(older))).toBeNull();
  });

  it("refuses a link made for another router", () => {
    const bytes = bytesOf(VERSION_4_PAYLOAD);
    bytes[1] = ROUTER_VERSION + 1;
    expect(() => decodeRecord(payloadOf(bytes))).toThrow(StaleLink);
    expect(() => recordFromHash("#game=" + payloadOf(bytes))).toThrow(/another version/);
    expect(recordFromHash("#game=" + VERSION_4_PAYLOAD)?.moves).toHaveLength(2);
  });

  it("matches the engine's router version", () => {
    const source = readFileSync("src/protocol.rs", "utf8");
    const declared = /pub const ROUTER_VERSION: u8 = (\d+);/.exec(source);
    expect(Number(declared?.[1])).toBe(ROUTER_VERSION);
  });

  it("refuses a face move that names what does not exist yet", () => {
    const bytes = bytesOf(VERSION_4_PAYLOAD);
    // The second move is the last twelve bytes: from, to, shape, the two
    // spokes one above their half-edge ids, the count, two ids and the
    // node as four bytes. One move before it made four half-edges and one
    // node, so a spoke of 4 or a node of 5 does not exist yet.
    const second = bytes.length - 12;
    expect(bytes[second + 2]).toBe(SHAPE_FACE);
    expect(bytes[second + 3]).toBe(4);
    const lateSpoke = bytes.slice();
    lateSpoke[second + 3] = 6;
    expect(decodeRecord(payloadOf(lateSpoke))).toBeNull();
    const lateReturn = bytes.slice();
    lateReturn[second + 4] = 6;
    expect(decodeRecord(payloadOf(lateReturn))).toBeNull();
    const lateNode = bytes.slice();
    lateNode[second + 7] = 5;
    expect(decodeRecord(payloadOf(lateNode))).toBeNull();
    const twice = bytes.slice();
    twice[second + 7] = 0;
    expect(decodeRecord(payloadOf(twice))).toBeNull();
  });

  it("decodes a version 3 link", () => {
    const record = decodeRecord(VERSION_3_PAYLOAD);
    expect(record?.nodes).toHaveLength(4);
    expect(record?.moves.map((m) => [m.from, m.to])).toEqual([
      [0, 1],
      [2, 0],
    ]);
    expect(record?.moves.every(hasStroke)).toBe(true);
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
      moves: linked.moves.map(stroke).map((m) => ({
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

  it("matches a move carried as topology by its endpoints and node", () => {
    const faces: TopologyMove[] = linked.moves.map(stroke).map((m) => ({
      from: m.from,
      to: m.to,
      face: { a: m.from, atA: undefined, b: m.to, atB: undefined, enclosed: [] },
      node: [m.newNode[0] + 0.2, m.newNode[1]],
    }));
    expect(sameGame(linked, { ...linked, moves: faces })).toBe(true);
    const other = faces.map((m) => ({ ...m, to: m.to + 1 }));
    expect(sameGame(linked, { ...linked, moves: other })).toBe(false);
    const moved = faces.map((m): TopologyMove => ({ ...m, node: [m.node[0] + 2, m.node[1]] }));
    expect(sameGame(linked, { ...linked, moves: moved })).toBe(false);
  });
});

describe("a move that arrives as topology", () => {
  const record = decodeRecord(VERSION_4_PAYLOAD);
  if (!record) throw new Error("the version 4 link does not decode");

  it("is pending until the engine reports the line it drew", () => {
    const history = History.from(record);
    const loop = history.moves[1];
    if (!loop) throw new Error("no second move");
    expect(pendingFace(history.moves[0]!)).toBeUndefined();
    expect(pendingFace(loop)).toEqual(LOOP_FACE);
    expect(history.positionOf(5)).toEqual(at(620, 500));
    // The saved record holds strokes only.
    expect(() => history.record()).toThrow(RangeError);
    expect(encodeRecord(history)).toBe(VERSION_4_PAYLOAD);

    const line: Line = {
      id: 1,
      fromNode: 2,
      toNode: 2,
      polyline: [at(500, 500), at(560, 440), at(620, 500), at(500, 500)],
      segments: [],
      newNodePos: at(620, 500),
      player: "ai",
    };
    history.settle(1, line);
    expect(pendingFace(loop)).toBeUndefined();
    expect(history.positionOf(5)).toEqual(at(620, 500));
    const saved = stroke(history.record().moves[1]);
    expect(saved.polyline).toHaveLength(4);
    expect(saved.newNode).toEqual([620, 500]);
    expect(encodeRecord(history)).toBe(VERSION_4_PAYLOAD);
  });

  it("reads the link back as a rehearsal and demotes a drawn move to geometry", () => {
    const history = History.from(record);
    const plan = rehearsal(history);
    expect(plan?.routerVersion).toBe(ROUTER_VERSION);
    expect(plan?.moves.map((m) => m.kind)).toEqual(["drawn", "routed"]);
    expect(plan?.moves[1]).toEqual({ kind: "routed", face: LOOP_FACE, at: at(620, 500) });
    expect(demote(history, 1)).toBe(false);
    expect(demote(history, 0)).toBe(false);

    const drawnWithTopology = new History(4, 1000);
    drawnWithTopology.push(
      { fromNode: 0, toNode: 1, polyline: [at(300, 500), at(700, 500)], newNodePos: at(500, 500) },
      { a: 0, atA: undefined, b: 1, atB: undefined, enclosed: [] },
    );
    expect(bytesOf(encodeRecord(drawnWithTopology))[SHAPE_OFFSET]).toBe(SHAPE_FACE);
    expect(demote(drawnWithTopology, 0)).toBe(true);
    expect(bytesOf(encodeRecord(drawnWithTopology))[SHAPE_OFFSET]).toBe(SHAPE_STROKE);
    expect(rehearsal(drawnWithTopology)?.moves[0]?.kind).toBe("drawn");
  });

  // An engine that answers from a script: the face moves it would report
  // for the classification, and the refusals for successive rehearsals.
  function scripted(faces: (FaceMove | undefined)[], refusals: (number | undefined)[]) {
    const seen: { classified: number; rehearsed: number } = { classified: 0, rehearsed: 0 };
    const engine: LinkEngine = {
      classify: (plan) => {
        seen.classified++;
        expect(plan.moves).toHaveLength(faces.length);
        return Promise.resolve({ topology: faces });
      },
      rehearse: () => Promise.resolve(refusals[seen.rehearsed++]),
    };
    return { engine, seen };
  }

  const chord: FaceMove = { a: 0, atA: undefined, b: 1, atB: undefined, enclosed: [] };

  function twoDrawnMoves(): History {
    const history = new History(4, 1000);
    history.push({
      fromNode: 0,
      toNode: 1,
      polyline: [at(300, 500), at(700, 500)],
      newNodePos: at(500, 500),
    });
    history.push({
      fromNode: 2,
      toNode: 3,
      polyline: [at(500, 300), at(500, 700)],
      newNodePos: at(500, 650),
    });
    return history;
  }

  it("prepares a link on a copy with the moves the engine says redraw", async () => {
    const source = twoDrawnMoves();
    const { engine, seen } = scripted([undefined, chord], [undefined]);
    const { history, refusedAt } = await prepareLink(source, engine);
    expect(refusedAt).toBeUndefined();
    expect(seen).toEqual({ classified: 1, rehearsed: 1 });
    expect(history.moves.map((m) => m.topology)).toEqual([undefined, chord]);
    expect(source.moves.map((m) => m.topology)).toEqual([undefined, undefined]);
    expect(history).not.toBe(source);
  });

  it("demotes a topology move the rehearsal refuses and rehearses again", async () => {
    const { engine, seen } = scripted([chord, chord], [0, undefined]);
    const { history, refusedAt } = await prepareLink(twoDrawnMoves(), engine);
    expect(refusedAt).toBeUndefined();
    expect(seen.rehearsed).toBe(2);
    expect(history.moves.map((m) => m.topology)).toEqual([undefined, chord]);
  });

  it("blames a refused drawn move on the topology move before it", async () => {
    const { engine, seen } = scripted([chord, undefined], [1, undefined]);
    const { history, refusedAt } = await prepareLink(twoDrawnMoves(), engine);
    expect(refusedAt).toBeUndefined();
    expect(seen.rehearsed).toBe(2);
    expect(history.moves.map((m) => m.topology)).toEqual([undefined, undefined]);
  });

  it("keeps the AI's face moves when the engine will not classify the history", async () => {
    const source = twoDrawnMoves();
    const aiFace: FaceMove = { a: 2, atA: undefined, b: 3, atB: undefined, enclosed: [] };
    source.moves[1]!.topology = aiFace;
    let rehearsed: Rehearsal | undefined;
    const engine: LinkEngine = {
      classify: () => Promise.reject(new Error("move 1: Path crosses an existing line")),
      rehearse: (plan) => {
        rehearsed = plan;
        return Promise.resolve(undefined);
      },
    };
    const { history, refusedAt } = await prepareLink(source, engine);
    expect(refusedAt).toBeUndefined();
    expect(history.moves.map((m) => m.topology)).toEqual([undefined, aiFace]);
    expect(rehearsed?.moves.map((m) => m.kind)).toEqual(["drawn", "routed"]);
  });

  it("passes a failing rehearsal on to the caller", async () => {
    const engine: LinkEngine = {
      classify: (plan) => Promise.resolve({ topology: plan.moves.map(() => undefined) }),
      rehearse: () => Promise.reject(new Error("Worker call 'rehearse' timed out")),
    };
    await expect(prepareLink(twoDrawnMoves(), engine)).rejects.toThrow(/timed out/);
  });

  it("reports a drawn move the rehearsal refuses and stops", async () => {
    const { engine, seen } = scripted([undefined, chord], [0]);
    const { history, refusedAt } = await prepareLink(twoDrawnMoves(), engine);
    expect(refusedAt).toBe(0);
    expect(seen.rehearsed).toBe(1);
    expect(history.moves[1]?.topology).toEqual(chord);
  });

  it("skips the classification while a move is still pending", async () => {
    const { engine, seen } = scripted([], [undefined]);
    const { history } = await prepareLink(History.from(record), engine);
    expect(seen).toEqual({ classified: 0, rehearsed: 1 });
    expect(pendingFace(history.moves[1]!)).toEqual(LOOP_FACE);
  });

  it("takes the topology the engine reports for a drawn move", () => {
    const history = new History(4, 1000);
    history.push(
      { fromNode: 0, toNode: 1, polyline: [at(300, 500), at(700, 500)], newNodePos: at(500, 500) },
      { a: 0, atA: undefined, b: 1, atB: undefined, enclosed: [] },
    );
    expect(bytesOf(encodeRecord(history))[SHAPE_OFFSET]).toBe(SHAPE_FACE);
    expect(stroke(history.record().moves[0]).polyline).toHaveLength(2);
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
        const got = stroke(decoded?.moves[i]);
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

describe("starting positions in a link", () => {
  function withNodes(nodes: [number, number][] | null): History {
    const history = game({
      fromNode: 0,
      toNode: 1,
      polyline: [at(300, 500), at(500, 450), at(700, 500)],
      newNodePos: at(500, 450),
    });
    history.nodes = nodes;
    return history;
  }

  it("leaves the default layout out of the link and reads it back as the default", () => {
    const plain = encodeRecord(withNodes(null));
    const laidOut = encodeRecord(withNodes(openingLayout(4, 1000)));
    expect(laidOut).toBe(plain);
    expect(bytesOf(plain)[0]).toBe(LINK_VERSION);
    expect(bytesOf(plain)[FLAGS_OFFSET]! & 2).toBe(0);
    expect(decodeRecord(plain)?.nodes).toEqual(openingLayout(4, 1000));
  });

  it("carries another layout and brings it back within a quantum", () => {
    const circle = circleLayout(4, 1000);
    const link = encodeRecord(withNodes(circle));
    expect(bytesOf(link)[FLAGS_OFFSET]! & 2).toBe(2);
    const record = decodeRecord(link);
    expect(record?.nodes).toHaveLength(4);
    for (const [i, [x, y]] of circle.entries()) {
      expect(Math.abs(record!.nodes![i]![0] - x)).toBeLessThanOrEqual(LINK_QUANTUM / 2);
      expect(Math.abs(record!.nodes![i]![1] - y)).toBeLessThanOrEqual(LINK_QUANTUM / 2);
    }
    expect(link.length - encodeRecord(withNodes(null)).length).toBeLessThanOrEqual(24);
  });

  it("reads version 1 and 2 links onto the circle they were drawn on", () => {
    expect(decodeRecord(VERSION_1_PAYLOAD)?.nodes).toEqual(circleLayout(4, 500));
    const second = decodeRecord(VERSION_2_PAYLOAD);
    expect(second?.nodes).toEqual(circleLayout(4, 1000));
    expect(second?.moves.map((m) => [m.from, m.to])).toEqual([
      [0, 1],
      [2, 0],
    ]);
  });

  it("reads a saved record without positions as a circle game", () => {
    const saved = JSON.parse(
      readFileSync("fixtures/saved/first-exchange.json", "utf8"),
    ) as GameRecord;
    delete saved.nodes;
    const storage = { getItem: () => JSON.stringify(saved) };
    Object.defineProperty(globalThis, "localStorage", { value: storage, configurable: true });
    const loaded = loadSaved();
    expect(loaded?.nodes).toEqual(circleLayout(4, 1000));
    expect(loaded?.moves).toHaveLength(2);
  });

  it("tells a saved game from a link on another layout", () => {
    const saved = withNodes(circleLayout(4, 1000)).record();
    const linked = decodeRecord(encodeRecord(withNodes(null)))!;
    expect(sameGame(saved, linked)).toBe(false);
    expect(sameGame(saved, decodeRecord(encodeRecord(withNodes(circleLayout(4, 1000))))!)).toBe(
      true,
    );
  });
});
