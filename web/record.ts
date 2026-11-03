import type { GameRecord } from "./history";
import type { Node } from "./state";

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

// The saved file is the record the page keeps: the moves as drawn, so the
// bench replays them and settles each one the way the page did.
function saveGame(record: GameRecord): void {
  const blob = new Blob([JSON.stringify(record)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = `sprouts-${record.initialNodes}n-${record.boardSize}.json`;
  anchor.click();
  // Revoking in the same task can cancel the download in Firefox and Safari.
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function wireRecording(record: () => GameRecord): void {
  if (!new URLSearchParams(window.location.search).has("record")) return;
  const button = document.createElement("button");
  button.id = "saveBtn";
  button.textContent = "Save Game";
  button.addEventListener("click", () => saveGame(record()));
  document.querySelector(".controls")?.append(button);
}
