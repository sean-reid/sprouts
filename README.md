# Sprouts Game

Browser-based Sprouts with AI opponent using morphological skeleton pathfinding.

## About

Sprouts is a pen-and-paper game where players draw lines between nodes. Each line adds a new node. Nodes can have at most 3 connections, lines cannot cross. Last player able to move wins.

**Features:**
- HTML5 Canvas with touch/mouse support
- AI using Zhang-Suen thinning + A* pathfinding on morphological skeleton
- Minimax search (2 ply) with alpha-beta pruning
- Dynamic clearance and adaptive border margins for tight endgames
- Rust/WASM core, Web Worker for async AI

## Build

```bash
# Install wasm-pack
curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh

# Build
wasm-pack build --target web --release

# Serve
python3 -m http.server 8000
```

## Architecture

```
HTML5 Canvas (rendering, input)
    ↓
Web Worker (game state)
    ↓
Rust/WASM (logic, AI, validation)
```

**Modules:** `morphology.rs` (skeleton), `pathfinding.rs` (A*), `ai.rs` (minimax), `validation.rs` (collision), `node_classifier.rs` (reachability), `components.rs` (union-find), `geometry.rs` (smoothing)

## How to Play

1. Click-drag-click between nodes (or tap-drag-tap on mobile)
2. Click path to place new node (tap "Confirm" on mobile)
3. Lines can't cross, nodes max out at 3 connections
4. Last valid move wins

## License

MIT
