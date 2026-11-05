import type { Cubic } from "../pkg/sprouts";
import type { GameRecord } from "./history";
import type { Node } from "./state";

export interface FixtureMove {
  from: number;
  to: number;
  polyline: [number, number][];
  newNode: [number, number];
  // Nodes the move cut off, as the engine reported; absent when unknown.
  cutOff?: number;
  // The spline the move committed as; a settled game carries it so a
  // replay applies the move exactly.
  segments?: Cubic[];
  // The move's index in the engine's move list and its node's place along
  // the routed curve, once the engine has reported them; `refused` marks
  // a move a regenerated board would not take, so the game shares as
  // shapes without indexing again.
  index?: number;
  along?: number;
  refused?: true;
}

export interface Fixture {
  version: number;
  initialNodes: number;
  boardSize: number;
  // Where the starting nodes were; the engine's default layout when absent.
  nodes?: [number, number][];
  // The settle the game was played under; absent means none ran.
  settleVersion?: number;
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
