# Sprouts Game

Browser-based Sprouts with AI opponent using morphological skeleton pathfinding.

## About

Sprouts is a pen-and-paper game where players draw lines between nodes. Each line adds a new node. Nodes can have at most 3 connections, lines cannot cross. Last player able to move wins.

**Features:**

- HTML5 Canvas with touch/mouse support
- AI using Zhang-Suen thinning + A* pathfinding on morphological skeleton
- Exact play from solved regions of the board, and a time-boxed alpha-beta search over an exact planar model where the position is too big to solve
- Dynamic clearance and adaptive border margins for tight endgames
- Rust/WASM core, Web Worker for async AI

## Build

Needs Rust with the `wasm32-unknown-unknown` target, wasm-pack, and Node 24.

```bash
npm install
npm run dev        # dev server; compiles the crate to wasm on start
npm run build      # production bundle in dist/
npm run test:e2e   # Playwright at 375, 768 and 1024 wide
cargo test
```

Merges to main deploy `dist/` to Cloudflare Pages.

## Architecture

```
HTML5 Canvas (rendering, input)
    ↓
Web Worker (game state)
    ↓
Rust/WASM (logic, AI, validation)
```

**Modules:** `curve.rs` (spline fit, flatten, nearest point), `morphology.rs` (skeleton), `pathfinding.rs` (A*), `abstract_graph.rs` (move choice and time-boxed search), `nimber.rs` (region outcomes and nimbers), `face/region.rs` (regions), `face/live.rs` (live form), `ai.rs` (candidate moves), `validation.rs` (collision), `node_classifier.rs` (reachability), `components.rs` (union-find), `geometry.rs` (segment and polyline helpers)

## How to Play

1. Click-drag-click between nodes (or tap-drag-tap on mobile)
2. Press away from the dot to place the new node
3. Lines can't cross, nodes max out at 3 connections
4. Last valid move wins

## License

MIT
