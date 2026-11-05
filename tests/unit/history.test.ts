import { readdirSync, readFileSync } from "node:fs";
import { basename, join } from "node:path";
import { describe, expect, it } from "vitest";
import type { Classification, Cubic, FaceMove, Line, Point, Rehearsal } from "../../pkg/sprouts";
import type { Indexed } from "../../pkg/sprouts";
import {
  CURVE_TOLERANCE,
  decodeRecord,
  demote,
  DRIFT_TOLERANCE,
  encodeRecord,
  hasStroke,
  History,
  isIndexed,
  LINK_QUANTUM,
  MOVE_LIST_VERSION,
  pendingIndex,
  loadSaved,
  pendingFace,
  prepareLink,
  recordFromHash,
  rehearsal,
  ROUTER_VERSION,
  SETTLE_VERSION,
  sameGame,
  sameIndexedGame,
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

// Header: version, router version, settle version, flags, initial nodes,
// board size, move count and cursor, then each move's from and to as
// one-byte varints before its shape byte.
const SHAPE_OFFSET = 14;
const SETTLE_OFFSET = 2;
const FLAGS_OFFSET = 3;
const SHAPE_STROKE = 0;
const SHAPE_CUBICS = 1;
const SHAPE_FACE = 2;
// The shapes link version; version 6 carries indices.
const LINK_VERSION = 5;
const FACE_LINK_VERSION = 4;
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
const VERSION_4_PAYLOAD = "BAUABADoAwIAAgAAAQAC2AToB6AGAI8DAAICAgQEAgAE2AnoBw";
// The same two moves as a version 5 link, with settle version 0 after the
// router byte.
const VERSION_5_PAYLOAD = "BQUAAAQA6AMCAAIAAAEAAtgE6AegBgCPAwACAgIEBAIABNgJ6Ac";
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
  if (hasStroke(move) || isIndexed(move)) throw new Error("the move decoded without a face");
  return move.face;
}

// The ends of each move, null for one that travelled by index.
function ends(moves: RecordMove[] | undefined): ([number, number] | null)[] | undefined {
  return moves?.map((m) => (isIndexed(m) ? null : [m.from, m.to]));
}

// An engine that cannot index a game, so a link travels as shapes.
const noIndex = (): Promise<Indexed> => Promise.reject(new Error("no index"));

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

  it("keeps a sparse polyline as a stroke when its cubics encode longer, unsettled", () => {
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
    const history = new History(4, 1000, "human", 0);
    history.push(move);
    const payload = encodeRecord(history);
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
    expect(bytes[0]).toBe(FACE_LINK_VERSION);
    expect(bytes[1]).toBe(ROUTER_VERSION);
    // A version 4 link was played with no settling.
    expect(record?.settleVersion).toBe(0);
    // A version 3 link has no router byte and no face shape.
    const older = new Uint8Array([3, ...bytes.slice(2)]);
    expect(decodeRecord(payloadOf(older))).toBeNull();
  });

  it("carries the settle version after the router byte and reads it back", () => {
    const history = new History(4, 1000);
    history.push({
      fromNode: 0,
      toNode: 1,
      polyline: [at(300, 500), at(700, 500)],
      newNodePos: at(500, 500),
    });
    const bytes = bytesOf(encodeRecord(history));
    expect(bytes[0]).toBe(LINK_VERSION);
    expect(bytes[1]).toBe(ROUTER_VERSION);
    expect(bytes[SETTLE_OFFSET]).toBe(SETTLE_VERSION);
    expect(decodeRecord(payloadOf(bytes))?.settleVersion).toBe(SETTLE_VERSION);
    const unsettled = History.from(decodeRecord(VERSION_4_PAYLOAD)!);
    expect(unsettled.settleVersion).toBe(0);
    expect(bytesOf(encodeRecord(unsettled))[SETTLE_OFFSET]).toBe(0);
    expect(decodeRecord(VERSION_5_PAYLOAD)).toEqual(decodeRecord(VERSION_4_PAYLOAD));
  });

  it("refuses a link made for another router", () => {
    const bytes = bytesOf(VERSION_4_PAYLOAD);
    bytes[1] = ROUTER_VERSION + 1;
    expect(() => decodeRecord(payloadOf(bytes))).toThrow(StaleLink);
    expect(() => recordFromHash("#game=" + payloadOf(bytes))).toThrow(/another version/);
    expect(recordFromHash("#game=" + VERSION_4_PAYLOAD)?.moves).toHaveLength(2);
  });

  it("refuses a link or record from a newer settle", () => {
    const bytes = bytesOf(
      encodeRecord(
        game({
          fromNode: 0,
          toNode: 1,
          polyline: [at(300, 500), at(700, 500)],
          newNodePos: at(500, 500),
        }),
      ),
    );
    expect(decodeRecord(payloadOf(bytes))).not.toBeNull();
    bytes[SETTLE_OFFSET] = SETTLE_VERSION + 1;
    expect(() => decodeRecord(payloadOf(bytes))).toThrow(StaleLink);
    const saved = JSON.parse(readFileSync("fixtures/saved/first-exchange.json", "utf8")) as Record<
      string,
      unknown
    >;
    const stub = (value: Record<string, unknown>) =>
      Object.defineProperty(globalThis, "localStorage", {
        value: { getItem: () => JSON.stringify(value) },
        configurable: true,
      });
    stub(saved);
    expect(loadSaved()).not.toBeNull();
    stub({ ...saved, settleVersion: SETTLE_VERSION + 1 });
    expect(loadSaved()).toBeNull();
  });

  it("matches the engine's router and settle versions", () => {
    const source = readFileSync("src/protocol.rs", "utf8");
    const router = /pub const ROUTER_VERSION: u8 = (\d+);/.exec(source);
    expect(Number(router?.[1])).toBe(ROUTER_VERSION);
    const settle = /pub const SETTLE_VERSION: u8 = (\d+);/.exec(source);
    expect(Number(settle?.[1])).toBe(SETTLE_VERSION);
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
    expect(ends(record?.moves)).toEqual([
      [0, 1],
      [2, 0],
    ]);
    expect(record?.moves.every(hasStroke)).toBe(true);
  });

  it("decodes a version 1 link", () => {
    const record = decodeRecord(VERSION_1_PAYLOAD);
    expect(record?.boardSize).toBe(500);
    expect(ends(record?.moves)).toEqual([
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
    const other = {
      ...linked,
      moves: linked.moves.map((m) => (isIndexed(m) ? m : { ...m, to: m.to + 1 })),
    };
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
    expect(encodeRecord(history)).toBe(VERSION_5_PAYLOAD);

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
    expect(encodeRecord(history)).toBe(VERSION_5_PAYLOAD);
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

  // The settled board of twoDrawnMoves as the engine reports it.
  const board = [
    at(100, 100),
    at(900, 100),
    at(900, 900),
    at(100, 900),
    at(500, 500),
    at(500, 650),
  ];

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
      index: noIndex,
      rehearse: () =>
        Promise.resolve({ refused: refusals[seen.rehearsed++], nodes: [], settled: board }),
    };
    return { engine, seen };
  }

  // An engine whose rehearsals all apply, reporting the given node for the
  // second move while it travels as topology.
  const second: FaceMove = { a: 2, atA: undefined, b: 3, atB: undefined, enclosed: [] };
  function drifting(node: Point) {
    const seen = { rehearsed: 0 };
    const engine: LinkEngine = {
      classify: () => Promise.resolve({ topology: [chord, second] }),
      index: noIndex,
      rehearse: (plan) => {
        seen.rehearsed++;
        const routed = plan.moves[1]?.kind === "routed";
        return Promise.resolve({
          refused: undefined,
          nodes: [undefined, routed ? node : undefined],
          settled: board,
        });
      },
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

  it("demotes a topology move whose rehearsed node drifts past the tolerance", async () => {
    const { engine, seen } = drifting(at(500, 650 + DRIFT_TOLERANCE + 0.5));
    const { history, refusedAt } = await prepareLink(twoDrawnMoves(), engine);
    expect(refusedAt).toBeUndefined();
    expect(seen.rehearsed).toBe(2);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, undefined]);
  });

  it("measures drift between the nodes as the link rounds them", async () => {
    // 3.99 apart as recorded, 4.24 apart once both round to whole units.
    const source = twoDrawnMoves();
    source.moves[1]!.newNodePos = at(500.49, 650.49);
    const { engine, seen } = drifting(at(503.31, 653.31));
    const { history } = await prepareLink(source, engine);
    expect(seen.rehearsed).toBe(2);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, undefined]);
  });

  // An engine whose rehearsals apply with the recorded nodes and settle to
  // the given boards in turn.
  function settling(boards: Point[][]) {
    const seen = { rehearsed: 0 };
    const engine: LinkEngine = {
      classify: () => Promise.resolve({ topology: [chord, second] }),
      index: noIndex,
      rehearse: (plan) =>
        Promise.resolve({
          refused: undefined,
          nodes: plan.moves.map((m) => (m.kind === "routed" ? m.at : undefined)),
          settled: boards[seen.rehearsed++] ?? [],
        }),
    };
    return { engine, seen };
  }

  it("demotes the latest topology move when the rehearsed board settles apart", async () => {
    const apart = board.map((p, i) => (i === 4 ? at(p.x, p.y + DRIFT_TOLERANCE + 0.5) : p));
    const { engine, seen } = settling([apart, board]);
    const { history, refusedAt } = await prepareLink(twoDrawnMoves(), engine, board);
    expect(refusedAt).toBeUndefined();
    expect(seen.rehearsed).toBe(2);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, undefined]);
  });

  it("reports a mismatch when no demotion brings the rehearsed board within the tolerance", async () => {
    const apart = board.map((p, i) => (i === 4 ? at(p.x, p.y + DRIFT_TOLERANCE + 0.5) : p));
    const { engine, seen } = settling([apart, apart, apart]);
    const { history, refusedAt, mismatch } = await prepareLink(twoDrawnMoves(), engine, board);
    expect(refusedAt).toBeUndefined();
    expect(mismatch).toBe(true);
    expect(seen.rehearsed).toBe(3);
    expect(history.moves.map((m) => m.topology)).toEqual([undefined, undefined]);
  });

  it("treats a rehearsed board with another node count as apart", async () => {
    const { engine, seen } = settling([board.slice(0, -1), board]);
    const { history, mismatch } = await prepareLink(twoDrawnMoves(), engine, board);
    expect(mismatch).toBeUndefined();
    expect(seen.rehearsed).toBe(2);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, undefined]);
  });

  it("runs only the record check when no settled board is given", async () => {
    const apart = board.map((p, i) => (i === 4 ? at(p.x, p.y + DRIFT_TOLERANCE + 0.5) : p));
    const { engine, seen } = settling([apart]);
    const { history, mismatch } = await prepareLink(twoDrawnMoves(), engine);
    expect(mismatch).toBeUndefined();
    expect(seen.rehearsed).toBe(1);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, second]);
  });

  it("keeps the topology moves when the rehearsed board settles within the tolerance", async () => {
    const near = board.map((p, i) => (i === 4 ? at(p.x, p.y + DRIFT_TOLERANCE) : p));
    const { engine, seen } = settling([near]);
    const { history } = await prepareLink(twoDrawnMoves(), engine, board);
    expect(seen.rehearsed).toBe(1);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, second]);
  });

  it("keeps a topology move whose rehearsed node drifts within the tolerance", async () => {
    const { engine, seen } = drifting(at(500, 650 + DRIFT_TOLERANCE));
    const { history } = await prepareLink(twoDrawnMoves(), engine);
    expect(seen.rehearsed).toBe(1);
    expect(history.moves.map((m) => m.topology)).toEqual([chord, second]);
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
      index: noIndex,
      rehearse: (plan) => {
        rehearsed = plan;
        return Promise.resolve({ refused: undefined, nodes: [], settled: [] });
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
      index: noIndex,
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

// Header: version, router version, settle version, move list version,
// flags, initial nodes, board size, move count and cursor.
const INDEX_HEADER = 13;
const LIST_OFFSET = 3;

describe("a link that names moves by index", () => {
  function twoMoves(): History {
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

  // An engine that indexes the two moves and answers the rehearsals in
  // turn, the first for the indexed link.
  function indexing(refusals: (number | undefined)[]) {
    const seen = { indexed: 0, classified: 0, rehearsed: 0, kinds: [] as string[][] };
    const engine: LinkEngine = {
      classify: (plan) => {
        seen.classified++;
        return Promise.resolve({ topology: plan.moves.map(() => undefined) });
      },
      index: (plan) => {
        seen.indexed++;
        expect(plan.moves).toHaveLength(2);
        return Promise.resolve({
          refused: undefined,
          moves: [
            { index: 3, along: 128 },
            { index: 200, along: 0 },
          ],
        });
      },
      rehearse: (plan) => {
        seen.kinds.push(plan.moves.map((m) => m.kind));
        if (plan.moves[0]?.kind === "indexed") expect(plan.listVersion).toBe(MOVE_LIST_VERSION);
        return Promise.resolve({
          refused: refusals[seen.rehearsed++],
          nodes: [],
          settled: [],
        });
      },
    };
    return { engine, seen };
  }

  it("encodes each move as its index and place and reads them back", () => {
    const history = twoMoves();
    history.moves[0]!.indexed = { index: 3, along: 128 };
    history.moves[1]!.indexed = { index: 200, along: 0 };
    const payload = encodeRecord(history);
    const bytes = bytesOf(payload);
    expect(bytes[0]).toBe(6);
    expect(bytes[LIST_OFFSET]).toBe(MOVE_LIST_VERSION);
    // 3 * 2 + 1 with its place, then 400 as a two-byte varint.
    expect([...bytes.slice(INDEX_HEADER)]).toEqual([7, 128, 0x90, 0x03]);
    const decoded = decodeRecord(payload);
    expect(decoded?.moves).toEqual([
      { index: 3, along: 128 },
      { index: 200, along: 0 },
    ]);
    expect(decoded?.cursor).toBe(2);
    expect(decoded?.settleVersion).toBe(SETTLE_VERSION);
    if (!decoded) throw new Error("the link does not decode");
    expect(sameGame(decoded, decoded)).toBe(true);
    expect(sameGame(decoded, { ...decoded, moves: [{ index: 3, along: 127 }] })).toBe(false);
  });

  it("refuses a link counting in another move list order", () => {
    const history = twoMoves();
    for (const move of history.moves) move.indexed = { index: 0, along: 0 };
    const bytes = bytesOf(encodeRecord(history));
    bytes[LIST_OFFSET] = MOVE_LIST_VERSION + 1;
    expect(() => decodeRecord(payloadOf(bytes))).toThrow(StaleLink);
  });

  it("sends the moves as shapes while one lacks an index", () => {
    const history = twoMoves();
    history.moves[0]!.indexed = { index: 3, along: 128 };
    expect(bytesOf(encodeRecord(history))[0]).toBe(LINK_VERSION);
  });

  it("holds an indexed move as pending until the engine draws it", () => {
    const history = twoMoves();
    for (const move of history.moves) move.indexed = { index: 1, along: 0 };
    const visitor = History.from(decodeRecord(encodeRecord(history))!);
    const first = visitor.moves[0]!;
    expect(pendingIndex(first)).toEqual({ index: 1, along: 0 });
    expect(pendingFace(first)).toBeUndefined();
    expect(visitor.label(0)).toBe("1. You");
    expect(visitor.ids(0)).toBe("new node 4");
    expect(() => visitor.record()).toThrow(RangeError);
    visitor.settle(0, {
      fromNode: 0,
      toNode: 1,
      polyline: [at(300, 500), at(700, 500)],
      segments: [],
      newNodePos: at(500, 500),
    });
    expect(pendingIndex(first)).toBeUndefined();
    expect(visitor.label(0)).toBe("1. You, joins two nodes");
    expect(visitor.ids(0)).toBe("0 to 1, new node 4");
    expect(visitor.positionOf(4)).toEqual(at(500, 500));
  });

  it("prepares an index link when the engine regenerates every move", async () => {
    const { engine, seen } = indexing([]);
    const { history, refusedAt, mismatch } = await prepareLink(twoMoves(), engine);
    expect(refusedAt).toBeUndefined();
    expect(mismatch).toBeUndefined();
    expect(seen).toEqual({ indexed: 1, classified: 0, rehearsed: 0, kinds: [] });
    expect(history.moves.map((m) => m.indexed)).toEqual([
      { index: 3, along: 128 },
      { index: 200, along: 0 },
    ]);
    expect(bytesOf(encodeRecord(history))[0]).toBe(6);
  });

  it("falls back to shapes when the engine cannot index the game", async () => {
    const seen = { rehearsed: 0, classified: 0, kinds: [] as string[][] };
    const engine: LinkEngine = {
      classify: (plan) => {
        seen.classified++;
        return Promise.resolve({ topology: plan.moves.map(() => undefined) });
      },
      index: () => Promise.reject(new Error("move 1: the regenerated board refuses it")),
      rehearse: (plan) => {
        seen.rehearsed++;
        seen.kinds.push(plan.moves.map((m) => m.kind));
        return Promise.resolve({ refused: undefined, nodes: [], settled: [] });
      },
    };
    const { history } = await prepareLink(twoMoves(), engine);
    expect(seen).toEqual({ rehearsed: 1, classified: 1, kinds: [["drawn", "drawn"]] });
    expect(history.moves.every((m) => m.indexed === undefined)).toBe(true);
    expect(bytesOf(encodeRecord(history))[0]).toBe(LINK_VERSION);
  });

  it("passes a failing shapes rehearsal on after the index fell back", async () => {
    const engine: LinkEngine = {
      classify: (plan) => Promise.resolve({ topology: plan.moves.map(() => undefined) }),
      index: noIndex,
      rehearse: () => Promise.reject(new Error("Worker call 'rehearse' timed out")),
    };
    await expect(prepareLink(twoMoves(), engine)).rejects.toThrow(/timed out/);
  });

  it("knows an index link of the saved game by indexing the record", async () => {
    const saved = twoMoves().record();
    const linked = twoMoves();
    linked.moves[0]!.indexed = { index: 3, along: 128 };
    linked.moves[1]!.indexed = { index: 200, along: 0 };
    const link = decodeRecord(encodeRecord(linked))!;
    const calls: Classification[] = [];
    const index = (plan: Classification) => {
      calls.push(plan);
      return Promise.resolve({
        refused: undefined,
        moves: [
          { index: 3, along: 128 },
          { index: 200, along: 0 },
        ],
      });
    };
    expect(await sameIndexedGame(saved, link, index)).toBe(true);
    expect(calls).toHaveLength(1);
    expect(calls[0]?.moves).toHaveLength(2);
    const otherIndices = () =>
      Promise.resolve({
        refused: undefined,
        moves: [
          { index: 3, along: 128 },
          { index: 201, along: 0 },
        ],
      });
    expect(await sameIndexedGame(saved, link, otherIndices)).toBe(false);
    const failing = () => Promise.reject(new Error("no index"));
    expect(await sameIndexedGame(saved, link, failing)).toBe(false);
    // A different header is another game before the engine is asked, and
    // so is a link with more moves than the saved game.
    const shorter = { ...saved, moves: saved.moves.slice(0, 1) };
    expect(await sameIndexedGame(shorter, link, index)).toBe(false);
    expect(await sameIndexedGame({ ...saved, firstPlayer: "ai" }, link, index)).toBe(false);
    expect(calls).toHaveLength(1);
    // A game played on past the link indexes its first moves only.
    const longer = twoMoves();
    longer.push({
      fromNode: 4,
      toNode: 5,
      polyline: [at(500, 500), at(500, 650)],
      newNodePos: at(500, 575),
    });
    expect(await sameIndexedGame(longer.record(), link, index)).toBe(true);
    expect(calls).toHaveLength(2);
    expect(calls[1]?.moves).toHaveLength(2);
    // A shapes link is not compared this way.
    expect(await sameIndexedGame(saved, decodeRecord(encodeRecord(twoMoves()))!, index)).toBe(
      false,
    );
    expect(sameGame(saved, link)).toBe(false);
  });

  it("shares a game whose moves carry their indices without asking the engine", async () => {
    const source = twoMoves();
    source.moves[0]!.indexed = { index: 9, along: 9 };
    source.moves[1]!.indexed = { index: 8, along: 0 };
    const { engine, seen } = indexing([]);
    const { history } = await prepareLink(source, engine);
    expect(seen).toEqual({ indexed: 0, classified: 0, rehearsed: 0, kinds: [] });
    expect(history.moves.map((m) => m.indexed)).toEqual([
      { index: 9, along: 9 },
      { index: 8, along: 0 },
    ]);
    expect(bytesOf(encodeRecord(history))[0]).toBe(6);
  });

  it("takes each applied move's index from the engine and drops it on undo", () => {
    const history = new History(4, 1000);
    const move = {
      fromNode: 0,
      toNode: 1,
      polyline: [at(300, 500), at(700, 500)],
      newNodePos: at(500, 500),
    };
    history.push(move, undefined, 0, { index: 2, along: 100 });
    history.push({ ...move, fromNode: 2, toNode: 3 }, undefined, 0, false);
    expect(history.moves[0]?.indexed).toEqual({ index: 2, along: 100 });
    expect(history.moves[1]?.indexed).toBeUndefined();
    expect(history.moves[1]?.refused).toBe(true);
    expect(history.indexed).toBe(false);
    const saved = history.record();
    expect(saved.moves[0]).toMatchObject({ index: 2, along: 100 });
    expect(saved.moves[1]).toMatchObject({ refused: true });
    const back = History.from(saved);
    expect(back.moves[0]?.indexed).toEqual({ index: 2, along: 100 });
    expect(back.moves[0]?.polyline).toEqual(move.polyline);
    expect(back.moves[0]?.fromNode).toBe(0);
    expect(pendingIndex(back.moves[0]!)).toBeUndefined();
    expect(saved.moves.every(hasStroke)).toBe(true);
    expect(saved.moves.some(isIndexed)).toBe(false);
    expect(back.moves[1]?.refused).toBe(true);
    history.invalidateFrom(1);
    expect(history.moves[0]?.indexed).toEqual({ index: 2, along: 100 });
    expect(history.moves[1]?.refused).toBeUndefined();
    history.cursor = 1;
    history.applied(0, {
      kind: "applied",
      snapshot: {} as never,
      cutOff: 0,
      mov: move,
      indexed: { index: 2, along: 101 },
    });
    expect(history.moves[0]?.indexed).toEqual({ index: 2, along: 101 });
  });

  it("indexes a record from before the cache once and keeps the result", async () => {
    const { engine, seen } = indexing([]);
    const { history } = await prepareLink(twoMoves(), engine);
    expect(seen.indexed).toBe(1);
    expect(history.indexed).toBe(true);
    const again = await prepareLink(history, engine);
    expect(seen.indexed).toBe(1);
    expect(again.history.indexed).toBe(true);
  });

  it("does not index a game with a refused move again and shares it as shapes", async () => {
    const source = twoMoves();
    source.moves[0]!.indexed = { index: 9, along: 9 };
    source.moves[1]!.refused = true;
    const seen = { indexed: 0, rehearsed: 0 };
    const engine: LinkEngine = {
      classify: (plan) => Promise.resolve({ topology: plan.moves.map(() => undefined) }),
      index: () => {
        seen.indexed++;
        return Promise.reject(new Error("not asked"));
      },
      rehearse: () => {
        seen.rehearsed++;
        return Promise.resolve({ refused: undefined, nodes: [], settled: [] });
      },
    };
    const { history } = await prepareLink(source, engine);
    expect(seen).toEqual({ indexed: 0, rehearsed: 1 });
    expect(bytesOf(encodeRecord(history))[0]).toBe(LINK_VERSION);
  });

  it("marks the move the engine's regeneration refused and shares as shapes", async () => {
    const seen = { indexed: 0, rehearsed: 0 };
    const engine: LinkEngine = {
      classify: (plan) => Promise.resolve({ topology: plan.moves.map(() => undefined) }),
      index: () => {
        seen.indexed++;
        return Promise.resolve({ refused: 1, moves: [{ index: 3, along: 128 }] });
      },
      rehearse: () => {
        seen.rehearsed++;
        return Promise.resolve({ refused: undefined, nodes: [], settled: [] });
      },
    };
    const { history } = await prepareLink(twoMoves(), engine);
    expect(seen).toEqual({ indexed: 1, rehearsed: 1 });
    expect(history.moves[0]?.indexed).toEqual({ index: 3, along: 128 });
    expect(history.moves[1]?.refused).toBe(true);
    expect(bytesOf(encodeRecord(history))[0]).toBe(LINK_VERSION);
  });

  it("knows an own link from the cached indices without the engine", async () => {
    const saved = twoMoves();
    saved.moves[0]!.indexed = { index: 3, along: 128 };
    saved.moves[1]!.indexed = { index: 200, along: 0 };
    const link = decodeRecord(encodeRecord(saved))!;
    const never = () => Promise.reject(new Error("not asked"));
    expect(await sameIndexedGame(saved.record(), link, never)).toBe(true);
    const other = decodeRecord(encodeRecord(saved))!;
    (other.moves[1] as { along: number }).along = 1;
    expect(await sameIndexedGame(saved.record(), other, never)).toBe(false);
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
// the original within both of it. A settled record travels as its spline,
// exact on the link grid, and the decoded stroke is that spline flattened
// at the fit tolerance, so the original stroke sits within twice it.
describe("every recorded game through the link", () => {
  for (const file of fixtures) {
    it(`${basename(file, ".json")} keeps its geometry`, () => {
      const fixture = JSON.parse(readFileSync(file, "utf8")) as Fixture;
      const history = History.from({
        firstPlayer: "human",
        cursor: fixture.moves.length,
        ...fixture,
        settleVersion: fixture.settleVersion ?? 0,
        version: 1,
      });
      const decoded = decodeRecord(encodeRecord(history));
      expect(decoded).not.toBeNull();
      expect(decoded?.moves).toHaveLength(fixture.moves.length);
      const scale = fixture.boardSize / 1000;
      const quantum = LINK_QUANTUM * scale;
      const settled = (fixture.settleVersion ?? 0) > 0;
      const slack = settled
        ? 2 * CURVE_TOLERANCE * scale + quantum
        : STROKE_TOLERANCE * scale + quantum;
      const rounded = (p: [number, number]): [number, number] => [
        Math.round(p[0] / quantum) * quantum,
        Math.round(p[1] / quantum) * quantum,
      ];
      fixture.moves.forEach((move: FixtureMove, i) => {
        const got = stroke(decoded?.moves[i]);
        expect(got.from).toBe(move.from);
        expect(got.to).toBe(move.to);
        expect(got.newNode).toEqual(rounded(move.newNode));
        if (settled) expect(got.segments).toEqual(move.segments);
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
    expect(ends(second?.moves)).toEqual([
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
