import type { GameState, Node } from "./state";

// Matches FIXTURE_VERSION in src/replay.rs.
const FIXTURE_VERSION = 1;

export interface FixtureMove {
  from: number;
  to: number;
  polyline: [number, number][];
  newNode: [number, number];
  // Nodes the move cut off, as the engine reported; absent when unknown.
  cutOff?: number;
}

export interface Fixture {
  version: number;
  initialNodes: number;
  boardSize: number;
  // Where the starting nodes were; the engine's default layout when absent.
  nodes?: [number, number][];
  moves: FixtureMove[];
}

// Positions of the first `count` nodes by id, which are the starting ones.
export function startingNodes(nodes: Node[], count: number): [number, number][] {
  return nodes
    .filter((node) => node.id < count)
    .sort((a, b) => a.id - b.id)
    .map((node) => [node.position.x, node.position.y]);
}

export function toFixture(state: GameState, boardSize: number): Fixture {
  const initialNodes = state.nodes.length - state.lines.length;
  return {
    version: FIXTURE_VERSION,
    initialNodes,
    boardSize,
    nodes: startingNodes(state.nodes, initialNodes),
    moves: state.lines.map((line) => ({
      from: line.fromNode,
      to: line.toNode,
      polyline: line.polyline.map((p): [number, number] => [p.x, p.y]),
      newNode: [line.newNodePos.x, line.newNodePos.y],
    })),
  };
}

function saveGame(state: GameState, boardSize: number): void {
  const fixture = toFixture(state, boardSize);
  const blob = new Blob([JSON.stringify(fixture)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `sprouts-${fixture.initialNodes}n-${boardSize}.json`;
  anchor.click();
  // Revoking in the same task can cancel the download in Firefox and Safari.
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function wireRecording(state: GameState, boardSize: () => number): void {
  if (!new URLSearchParams(window.location.search).has("record")) return;
  const button = document.createElement("button");
  button.id = "saveBtn";
  button.textContent = "Save Game";
  button.addEventListener("click", () => saveGame(state, boardSize()));
  document.querySelector(".controls")?.append(button);
}
