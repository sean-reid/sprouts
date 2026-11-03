import init, { SproutsGame } from "../pkg/sprouts.js";
import wasmUrl from "../pkg/sprouts_bg.wasm?url";
import type { Envelope, Reply, Request, Results } from "./protocol";

interface WorkerScope {
  onmessage: ((event: MessageEvent<Envelope>) => void) | null;
  postMessage(reply: Reply): void;
}

const scope = self as unknown as WorkerScope;

let wasmLoaded: Promise<unknown> | null = null;
let game: SproutsGame | null = null;

function ready(): SproutsGame {
  if (!game) throw new Error("WASM not initialized");
  return game;
}

async function handle(request: Request): Promise<Results[Request["type"]]> {
  switch (request.type) {
    case "init":
      wasmLoaded ??= init({ module_or_path: wasmUrl });
      await wasmLoaded;
      game?.free();
      game = null;
      game = SproutsGame.new_with_board_size(request.nodeCount, request.boardSize);
      return null;
    case "get_state":
      return ready().get_state();
    case "get_active_nodes":
      return ready().get_active_nodes();
    case "is_game_over":
      return ready().is_game_over();
    case "get_ai_move":
      return ready().get_ai_move();
    case "undo":
      return ready().undo();
    case "get_closest_point":
      return ready().get_closest_point_on_path(
        Float64Array.from(request.pathData),
        request.x,
        request.y,
      );
    case "validate_placement":
      return ready().validate_placement(
        Float64Array.from(request.pathData),
        request.newNodeX,
        request.newNodeY,
      );
    case "apply_move":
      return ready().apply_move(
        request.fromNode,
        request.toNode,
        Float64Array.from(request.pathData),
        request.newNodeX,
        request.newNodeY,
      );
    case "get_move_error":
      return ready().get_move_error(
        request.fromNode,
        request.toNode,
        Float64Array.from(request.pathData),
        request.newNodeX,
        request.newNodeY,
      );
  }
}

scope.onmessage = async (event) => {
  const { id, request } = event.data;
  try {
    scope.postMessage({ id, result: await handle(request) });
  } catch (error) {
    scope.postMessage({ id, error: error instanceof Error ? error.message : String(error) });
  }
};
