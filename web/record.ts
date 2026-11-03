import type { GameState } from "./state";

// Matches FIXTURE_VERSION in src/replay.rs.
const FIXTURE_VERSION = 1;

interface FixtureMove {
  from: number;
  to: number;
  polyline: [number, number][];
  newNode: [number, number];
}

export interface Fixture {
  version: number;
  initialNodes: number;
  boardSize: number;
  moves: FixtureMove[];
}

export function toFixture(state: GameState, boardSize: number): Fixture {
  return {
    version: FIXTURE_VERSION,
    initialNodes: state.nodes.length - state.lines.length,
    boardSize,
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

export function wireRecording(state: GameState, boardSize: number): void {
  if (!new URLSearchParams(window.location.search).has("record")) return;
  const button = document.createElement("button");
  button.id = "saveBtn";
  button.textContent = "Save Game";
  button.addEventListener("click", () => saveGame(state, boardSize));
  document.querySelector(".controls")?.append(button);
}
