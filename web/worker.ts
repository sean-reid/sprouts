import init, { SproutsGame } from "../pkg/sprouts.js";
import wasmUrl from "../pkg/sprouts_bg.wasm?url";
import type { Envelope, Reply } from "./protocol";

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

async function handle(envelope: Envelope): Promise<unknown> {
  if (envelope.method === "init") {
    const [nodeCount, boardSize] = envelope.args;
    wasmLoaded ??= init({ module_or_path: wasmUrl });
    await wasmLoaded;
    game?.free();
    game = null;
    game = new SproutsGame(nodeCount, boardSize);
    return undefined;
  }
  const current = ready();
  return Reflect.apply(current[envelope.method], current, envelope.args) as unknown;
}

scope.onmessage = async (event) => {
  const { id } = event.data;
  try {
    scope.postMessage({ id, ok: true, result: await handle(event.data) });
  } catch (error) {
    scope.postMessage({
      id,
      ok: false,
      error: error instanceof Error ? error.message : String(error),
    });
  }
};
