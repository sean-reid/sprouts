import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import init, { SproutsGame } from "../../pkg/sprouts.js";

// The page runs the engine in wasm with Rust's own libm, so the built
// wasm has to hash a fixed board to the value the native test recorded.
describe("the wasm engine", () => {
  it("hashes a fixed board to the native golden", async () => {
    await init({ module_or_path: readFileSync("pkg/sprouts_bg.wasm") });
    const source = readFileSync("src/protocol.rs", "utf8");
    const golden = /const BOARD_HASH: &str = "(0x[0-9a-f]+)";/.exec(source)?.[1];
    expect(golden).toBeDefined();
    const nodes = [
      { x: 632.2875655532296, y: 500.0 },
      { x: 331.0475650550718, y: 654.7742702330713 },
      { x: 525.8608781260419, y: 205.32863223151458 },
      { x: 712.9536013426019, y: 777.7602629520936 },
    ];
    const game = new SproutsGame(4, 1000, "human", { nodes });
    expect(game.hashBoard()).toBe(golden);
  });
});
