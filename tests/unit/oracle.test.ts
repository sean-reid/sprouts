import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import init, { goldenHashes } from "../../pkg/sprouts.js";

// The settled boards of the seeded games as macOS recorded them in
// src/settle/golden.rs; the wasm build has to settle the same, since the
// page replays links with it.
function recorded(name: string): string[] {
  const source = readFileSync("src/settle/golden.rs", "utf8");
  const block = new RegExp(`${name}[^=]*=\\s*&?\\[?([^;]*);`).exec(source);
  if (!block) throw new Error(`${name} is not in src/settle/golden.rs`);
  return [...block[1]!.matchAll(/0x[0-9a-f]+/g)].map((m) => m[0]);
}

describe("the wasm build settles the seeded games as macOS did", () => {
  it("matches the recorded boards move for move", async () => {
    await init({ module_or_path: readFileSync("pkg/sprouts_bg.wasm") });
    const nine = goldenHashes(9);
    const want = recorded("VERSION_1_SEEDED_GAME");
    const first = nine.findIndex((h, i) => h !== want[i]);
    expect(nine, `first differing move ${first}`).toEqual(want);
    const seven = goldenHashes(7);
    expect(seven.at(-1)).toBe(recorded("VERSION_1_GOLDEN")[0]);
  });
});
