import { execFileSync } from "node:child_process";
import { defineConfig, type Plugin } from "vite";

// Compiles the crate into pkg/ before Vite resolves the worker's import of it,
// so one build command covers Rust and TypeScript.
function wasmPack(): Plugin {
  return {
    name: "sprouts:wasm-pack",
    buildStart() {
      execFileSync("wasm-pack", ["build", "--target", "web", "--release", "--features", "hash"], {
        stdio: "inherit",
      });
    },
  };
}

export default defineConfig({
  plugins: [wasmPack()],
  build: {
    target: "es2022",
    sourcemap: false,
    modulePreload: { polyfill: false },
  },
  worker: {
    format: "es",
  },
});
