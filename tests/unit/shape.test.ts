import { describe, expect, it } from "vitest";
import type { Outcome, Point, Snapshot } from "../../pkg/sprouts";
import { History, loadSaved, type GameRecord } from "../../web/history";
import { shapeText } from "../../web/shape";

function at(x: number, y: number): Point {
  return { x, y };
}

// A square loop drawn from node 0 at (100, 100) and back to it.
const SQUARE = [at(100, 100), at(200, 100), at(200, 200), at(100, 200), at(100, 100)];

// The count is joined to "cuts off" with non-breaking spaces.
const cutsOff = (n: number) => `cuts\u00a0off\u00a0${n}`;

// An Applied outcome with the count and no lines, as the engine reports it
// for a move the page already holds the stroke of.
function applied(cutOff: number): Extract<Outcome, { kind: "applied" }> {
  const snapshot = {
    boardSize: 1000,
    initialNodes: 4,
    nodes: [],
    lines: [],
    currentPlayer: "ai",
    activeNodes: [],
    gameOver: false,
  } as unknown as Snapshot;
  return { kind: "applied", snapshot, cutOff };
}

describe("shape text", () => {
  it("names the four shapes", () => {
    expect(shapeText(false, 0)).toBe("joins two nodes");
    expect(shapeText(false, 3)).toBe(`joins two nodes, ${cutsOff(3)}`);
    expect(shapeText(true, 0)).toBe("loop on one node");
    expect(shapeText(true, 2)).toBe(`loop on one node, ${cutsOff(2)}`);
  });
});

describe("move rows", () => {
  it("reads a loop with its count and carries the ids in the title", () => {
    const history = new History(4, 1000);
    const loop = { fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(200, 150) };
    history.push(loop, undefined, 1);
    expect(history.label(0)).toBe(`1. You, loop on one node, ${cutsOff(1)}`);
    expect(history.ids(0)).toBe("0 to 0, new node 4");
  });

  it("reads a join, a splitting chord and a loop around nothing", () => {
    const history = new History(4, 1000);
    const chord = { fromNode: 2, toNode: 3, polyline: [at(300, 300), at(600, 600)] };
    history.push({ ...chord, newNodePos: at(450, 450) }, undefined, 0);
    history.push({ ...chord, newNodePos: at(400, 500) }, undefined, 1);
    history.push(
      {
        fromNode: 3,
        toNode: 3,
        polyline: [at(600, 600), at(700, 600), at(700, 700), at(600, 700), at(600, 600)],
        newNodePos: at(700, 650),
      },
      undefined,
      0,
    );
    expect(history.label(0)).toBe("1. You, joins two nodes");
    expect(history.ids(0)).toBe("2 to 3, new node 4");
    expect(history.label(1)).toBe(`2. AI, joins two nodes, ${cutsOff(1)}`);
    expect(history.label(2)).toBe("3. You, loop on one node");
    expect(history.ids(2)).toBe("3 to 3, new node 6");
  });

  it("reads plain until the engine applies the move, then keeps the count", () => {
    const history = new History(4, 1000);
    history.push({ fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(200, 150) });
    expect(history.label(0)).toBe("1. You, loop on one node");
    expect(history.record().moves[0]).not.toHaveProperty("cutOff");
    history.applied(0, applied(2));
    expect(history.label(0)).toBe(`1. You, loop on one node, ${cutsOff(2)}`);
    const record = history.record();
    expect(record.moves[0]).toHaveProperty("cutOff", 2);
    expect(History.from(record).label(0)).toBe(`1. You, loop on one node, ${cutsOff(2)}`);
  });
});

describe("saved records", () => {
  function saved(cutOff: unknown): GameRecord | null {
    const move: Record<string, unknown> = {
      from: 0,
      to: 1,
      polyline: [
        [100, 100],
        [300, 300],
      ],
      newNode: [200, 200],
    };
    if (cutOff !== undefined) move.cutOff = cutOff;
    const record = {
      version: 1,
      initialNodes: 4,
      boardSize: 1000,
      firstPlayer: "human",
      cursor: 1,
      moves: [move],
    };
    const storage = { getItem: () => JSON.stringify(record) };
    Object.defineProperty(globalThis, "localStorage", { value: storage, configurable: true });
    return loadSaved();
  }

  it("loads a move with a count, one without, and refuses a bad one", () => {
    const counted = saved(3);
    expect(counted?.moves[0]).toHaveProperty("cutOff", 3);
    expect(History.from(counted!).label(0)).toBe(`1. You, joins two nodes, ${cutsOff(3)}`);
    const plain = saved(undefined);
    expect(plain?.moves[0]).not.toHaveProperty("cutOff");
    expect(History.from(plain!).label(0)).toBe("1. You, joins two nodes");
    expect(saved(-1)).toBeNull();
    expect(saved("3")).toBeNull();
  });
});
