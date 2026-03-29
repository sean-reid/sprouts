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
                wasmGame = data.boardSize
                    ? cachedWasmModule.SproutsGame.new_with_board_size(data.nodeCount, data.boardSize)
                    : new cachedWasmModule.SproutsGame(data.nodeCount);
                wasmReady = true;
                self.postMessage({ id });
                break;
            }

            case 'get_state':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({ id, state: wasmGame.get_state() });
                break;

            case 'get_active_nodes':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({ id, activeNodes: wasmGame.get_active_nodes() });
                break;

            case 'apply_move': {
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                const success = wasmGame.apply_move(
                    data.fromNode, data.toNode, data.pathData,
                    data.newNodeX, data.newNodeY
                );
                self.postMessage({ id, success });
                break;
            }

            case 'undo': {
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                const success = wasmGame.undo();
                self.postMessage({ id, success });
                break;
            }

            case 'get_ai_move':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({ id, move: wasmGame.get_ai_move() });
                break;

            case 'get_closest_point':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({
                    id,
                    point: wasmGame.get_closest_point_on_path(data.pathData, data.x, data.y),
                });
                break;

            case 'validate_placement':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({
                    id,
                    valid: wasmGame.validate_placement(data.pathData, data.newNodeX, data.newNodeY),
                });
                break;

            case 'is_game_over':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({ id, gameOver: wasmGame.is_game_over() });
                break;

            case 'get_move_error':
                if (!wasmReady) { self.postMessage({ id, error: 'WASM not initialized' }); break; }
                self.postMessage({
                    id,
                    moveError: wasmGame.get_move_error(
                        data.fromNode, data.toNode, data.pathData,
                        data.newNodeX, data.newNodeY
                    ),
                });
                break;

            default:
                self.postMessage({ id, error: `Unknown message type: ${type}` });
        }
    } catch (error) {
        self.postMessage({ id, error: error.message || String(error) });
    }
};
