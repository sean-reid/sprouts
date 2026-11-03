import { describe, expect, it } from "vitest";
import type { Point } from "../../pkg/sprouts";
import { History } from "../../web/history";
import { enclosedCount, pointInPolygon, shapeText } from "../../web/shape";

function at(x: number, y: number): Point {
  return { x, y };
}

// A square loop drawn from node 0 at (100, 100) and back to it.
const SQUARE = [at(100, 100), at(200, 100), at(200, 200), at(100, 200), at(100, 100)];

describe("ray casting", () => {
  it("tells a point inside a square from one outside", () => {
    expect(pointInPolygon(at(150, 150), SQUARE)).toBe(true);
    expect(pointInPolygon(at(250, 150), SQUARE)).toBe(false);
    expect(pointInPolygon(at(150, 250), SQUARE)).toBe(false);
  });

  it("counts one node inside and none outside", () => {
    expect(enclosedCount(SQUARE, [at(150, 150), at(250, 150)])).toBe(1);
    expect(enclosedCount([at(100, 100), at(200, 200)], [at(150, 150)])).toBe(0);
  });
});

describe("move rows", () => {
  function board(): History {
    const history = new History(4, 1000);
    history.nodes = [
      [100, 100],
      [150, 150],
      [300, 300],
      [600, 600],
    ];
    return history;
  }

  it("reads a loop with the nodes it encloses and carries the ids in the title", () => {
    const history = board();
    history.push({ fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(200, 150) });
    expect(history.label(0)).toBe("1. You, loop on one node, encloses 1");
    expect(history.ids(0)).toBe("0 to 0, new node 4");
  });

  it("reads a join and a loop around nothing", () => {
    const history = board();
    history.push({
      fromNode: 2,
      toNode: 3,
      polyline: [at(300, 300), at(600, 600)],
      newNodePos: at(450, 450),
    });
    history.push({
      fromNode: 3,
      toNode: 3,
      polyline: [at(600, 600), at(700, 600), at(700, 700), at(600, 700), at(600, 600)],
      newNodePos: at(700, 650),
    });
    expect(history.label(0)).toBe("1. You, joins two nodes");
    expect(history.ids(0)).toBe("2 to 3, new node 4");
    expect(history.label(1)).toBe("2. AI, loop on one node");
    expect(history.ids(1)).toBe("3 to 3, new node 5");
  });

  it("counts only nodes that existed before the move", () => {
    const history = board();
    history.push({ fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(150, 100) });
    history.push({
      fromNode: 1,
      toNode: 1,
      polyline: [at(150, 150), at(190, 150), at(190, 190), at(150, 190), at(150, 150)],
      newNodePos: at(190, 170),
    });
    expect(history.label(1)).toBe("2. AI, loop on one node");
  });

  it("skips the count when the layout is unknown", () => {
    const history = new History(4, 1000);
    history.push({ fromNode: 0, toNode: 0, polyline: SQUARE, newNodePos: at(200, 150) });
    expect(history.label(0)).toBe("1. You, loop on one node");
  });

  it("names the shapes", () => {
    expect(shapeText(false, 3)).toBe("joins two nodes");
    expect(shapeText(true, 0)).toBe("loop on one node");
    expect(shapeText(true, 2)).toBe("loop on one node, encloses 2");
  });
});
