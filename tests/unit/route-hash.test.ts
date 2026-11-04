import { readdirSync, readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import init, { type Player, type RouteHashPlan, SproutsGame } from "../../pkg/sprouts.js";

// The page redraws a shared game's AI moves in wasm, with Rust's own
// libm, so the built wasm has to route the fixtures to the bytes the
// native test recorded on a Mac and the Linux runner checks.
describe("the wasm router", () => {
  it("hashes the routing fixtures to the native golden", async () => {
    await init({ module_or_path: readFileSync("pkg/sprouts_bg.wasm") });
    const source = readFileSync("src/route/tests.rs", "utf8");
    const golden = /const ROUTING_HASH: u64 = (0x[0-9a-f_]+);/.exec(source)?.[1]?.replace(/_/g, "");
    const count = Number(/const ROUTING_HASH_FIXTURES: usize = (\d+);/.exec(source)?.[1]);
    const sweep = /const SWEEP_FIXTURE: &str = "(.+)";/.exec(source)?.[1];
    expect(golden).toBeDefined();
    expect(count).toBeGreaterThan(0);
    expect(sweep).toBeDefined();
    const files = readdirSync("fixtures/routing")
      .filter((f) => f.endsWith(".json"))
      .sort()
      .slice(0, count)
      .map((f) => `fixtures/routing/${f}`)
      .concat([sweep as string]);
    const plans: RouteHashPlan["plans"] = files.map((f) => {
      const fixture = JSON.parse(readFileSync(f, "utf8"));
      const point = ([x, y]: number[]) => ({ x, y });
      return {
        initialNodes: fixture.initialNodes,
        boardSize: fixture.boardSize,
        firstPlayer: "ai" as Player,
        nodes: fixture.nodes ? fixture.nodes.map(point) : undefined,
        moves: fixture.moves.map(
          (m: { from: number; to: number; polyline: number[][]; newNode: number[] }) => ({
            fromNode: m.from,
            toNode: m.to,
            polyline: m.polyline.map(([x, y]) => ({ x, y })),
            newNodePos: { x: m.newNode[0], y: m.newNode[1] },
          }),
        ),
        known: fixture.moves.map(() => undefined),
      };
    });
    const game = new SproutsGame(4, 1000, "human");
    expect(game.hashRoutes({ plans })).toBe(golden);
  });
});
