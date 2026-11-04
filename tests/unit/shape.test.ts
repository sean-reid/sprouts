import { describe, expect, it } from "vitest";
import type { Point } from "../../pkg/sprouts";
import { History } from "../../web/history";
import { shapeText } from "../../web/shape";

function at(x: number, y: number): Point {
  return { x, y };
}

// A square loop drawn from node 0 at (100, 100) and back to it.
const SQUARE = [at(100, 100), at(200, 100), at(200, 200), at(100, 200), at(100, 100)];

describe("shape text", () => {
  it("names the four shapes", () => {
    expect(shapeText(false, 0)).toBe("joins two nodes");
    expect(shapeText(false, 3)).toBe("joins two nodes, cuts off 3");
    expect(shapeText(true, 0)).toBe("loop on one node");
    expect(shapeText(true, 2)).toBe("loop on one node, encloses 2");
  });
});

describe("move rows", () => {
  it("reads a loop with the nodes it encloses and carries the ids in the title", () => {
    const history = new History(4, 1000);
    const loop = { fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(200, 150) };
    history.push(loop, undefined, 1);
    expect(history.label(0)).toBe("1. You, loop on one node, encloses 1");
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
    expect(history.label(1)).toBe("2. AI, joins two nodes, cuts off 1");
    expect(history.label(2)).toBe("3. You, loop on one node");
    expect(history.ids(2)).toBe("3 to 3, new node 6");
  });

  it("reads a plain shape while the count is unknown and keeps it through a record", () => {
    const history = new History(4, 1000);
    history.push({ fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(200, 150) });
    expect(history.label(0)).toBe("1. You, loop on one node");
    expect(history.record().moves[0]).not.toHaveProperty("cutOff");
    history.moves[0]!.cutOff = 2;
    const record = history.record();
    expect(record.moves[0]).toHaveProperty("cutOff", 2);
    expect(History.from(record).label(0)).toBe("1. You, loop on one node, encloses 2");
  });
});
