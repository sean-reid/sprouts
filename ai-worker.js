let wasmGame = null;
let wasmReady = false;
let cachedWasmModule = null;

self.onmessage = async function(e) {
    const { type, data, id } = e.data;

    try {
        switch (type) {
            case 'init': {
                if (!cachedWasmModule) {
                    cachedWasmModule = await import('./pkg/sprouts.js');
                    await cachedWasmModule.default();
                }
                wasmGame = new cachedWasmModule.SproutsGame(data.nodeCount);
                wasmReady = true;
                self.postMessage({ type: 'init', id });
                break;
            }

            case 'get_state':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_state',
                    state: wasmGame.get_state(),
                    id
                });
                break;

            case 'get_active_nodes':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_active_nodes',
                    activeNodes: wasmGame.get_active_nodes(),
                    id
                });
                break;

            case 'apply_move': {
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                const success = wasmGame.apply_move(
                    data.fromNode, data.toNode, data.pathData,
                    data.newNodeX, data.newNodeY
                );
                self.postMessage({ type: 'apply_move', success, id });
                break;
            }

            case 'undo': {
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                const success = wasmGame.undo();
                self.postMessage({ type: 'undo', success, id });
                break;
            }

            case 'get_ai_move':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_ai_move',
                    move: wasmGame.get_ai_move(),
                    id
                });
                break;

            case 'get_closest_point':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_closest_point',
                    point: wasmGame.get_closest_point_on_path(data.pathData, data.x, data.y),
                    id
                });
                break;

            case 'validate_placement':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'validate_placement',
                    valid: wasmGame.validate_placement(data.pathData, data.newNodeX, data.newNodeY),
                    id
                });
                break;

            case 'is_game_over':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'is_game_over',
                    gameOver: wasmGame.is_game_over(),
                    id
                });
                break;

            case 'get_skeleton_debug':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_skeleton_debug',
                    skeleton: wasmGame.get_skeleton_debug(),
                    id
                });
                break;

            case 'get_classification_debug':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_classification_debug',
                    classification: wasmGame.get_classification_debug(),
                    id
                });
                break;

            case 'test_pair':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'test_pair',
                    result: wasmGame.test_pair(data.fromNode, data.toNode),
                    id
                });
                break;

            case 'get_move_error':
                if (!wasmReady) { self.postMessage({ type: 'error', error: 'WASM not initialized', id }); break; }
                self.postMessage({
                    type: 'get_move_error',
                    error: wasmGame.get_move_error(
                        data.fromNode, data.toNode, data.pathData,
                        data.newNodeX, data.newNodeY
                    ),
                    id
                });
                break;

            default:
                self.postMessage({ type: 'error', error: `Unknown message type: ${type}`, id });
        }
    } catch (error) {
        self.postMessage({ type: 'error', error: error.message || String(error), id });
    }
};
