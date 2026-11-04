import type { UndrawnReason } from "../pkg/sprouts";

// Why the AI drew nothing: the engine's reason, or the client giving up on
// the worker call.
export type UndrawnCause = UndrawnReason | "timeout";

const OUT_OF_TIME = "The AI ran out of time drawing its move. Undo or start a new game.";
const NO_MOVE = "The AI could not draw a move here. Undo or start a new game.";

export function undrawnStatus(cause: UndrawnCause): string {
  return cause === "outOfTime" || cause === "timeout" ? OUT_OF_TIME : NO_MOVE;
}
