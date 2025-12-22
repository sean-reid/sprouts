// AI Worker with full debug support
let wasmGame = null;

self.onmessage = async function(e) {
    const { type, data, id } = e.data;
    
    try {
        switch (type) {
            case 'init':
                const wasmModule = await import('./pkg/sprouts.js');
                await wasmModule.default();
                wasmGame = new wasmModule.SproutsGame(data.nodeCount);
                self.postMessage({ type: 'init', id });
                break;
                
            case 'get_state':
                self.postMessage({ 
                    type: 'get_state',
                    state: wasmGame.get_state(),
                    id
                });
                break;
                
            case 'get_active_nodes':
                self.postMessage({
                    type: 'get_active_nodes',
                    activeNodes: wasmGame.get_active_nodes(),
                    id
                });
                break;
                
            case 'apply_move':
                wasmGame.apply_move(
                    data.fromNode, data.toNode, data.pathData,
                    data.newNodeX, data.newNodeY
                );
                self.postMessage({ type: 'apply_move', id });
                break;
                
            case 'get_ai_move':
                self.postMessage({
                    type: 'get_ai_move',
                    move: wasmGame.get_ai_move(),
                    id
                });
                break;
                
            case 'get_closest_point':
                self.postMessage({
                    type: 'get_closest_point',
                    point: wasmGame.get_closest_point_on_path(data.pathData, data.x, data.y),
                    id
                });
                break;
                
            case 'validate_placement':
                self.postMessage({
                    type: 'validate_placement',
                    valid: wasmGame.validate_placement(data.pathData, data.newNodeX, data.newNodeY),
                    id
                });
                break;
                
            case 'is_game_over':
                self.postMessage({
                    type: 'is_game_over',
                    gameOver: wasmGame.is_game_over(),
                    id
                });
                break;
                
            case 'get_skeleton_debug':
                self.postMessage({
                    type: 'get_skeleton_debug',
                    skeleton: wasmGame.get_skeleton_debug(),
                    id
                });
                break;
                
            case 'get_classification_debug':
                self.postMessage({
                    type: 'get_classification_debug',
                    classification: wasmGame.get_classification_debug(),
                    id
                });
                break;
                
            case 'test_pair':
                self.postMessage({
                    type: 'test_pair',
                    result: wasmGame.test_pair(data.fromNode, data.toNode),
                    id
                });
                break;
                
            case 'get_move_error':
                self.postMessage({
                    type: 'get_move_error',
                    error: wasmGame.get_move_error(
                        data.fromNode, data.toNode, data.pathData,
                        data.newNodeX, data.newNodeY
                    ),
                    id
                });
                break;
        }
    } catch (error) {
        self.postMessage({ type: 'error', error: error.message, id });
    }
};
