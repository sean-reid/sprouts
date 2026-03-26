//! Opening book for the AI, which always plays second.
//!
//! Sprouts solved results:
//!   n=4: First player wins (max 11 moves)
//!   n=5: First player wins (max 14 moves)
//!   n=6: Second player wins (max 17 moves)
//!
//! The AI (player 2) is theoretically losing for n=4,5 but wins for n=6.
//! The book provides the AI's first response to common human openings.

use crate::types::GameState;

/// Returns the recommended (from_node, to_node) for the AI's response.
pub fn lookup_opening(state: &GameState) -> Option<(usize, usize)> {
    let n = state.initial_node_count;
    let move_num = state.move_history.len();

    match (n, move_num) {
        // 4 nodes, AI's first move: connect an untouched node to a connected one.
        (4, 1) => find_untouched_to_connected(state, 4),

        // 5 nodes, AI's first move: same strategy.
        (5, 1) => find_untouched_to_connected(state, 5),

        // 6 nodes, AI's first move: AI wins with perfect play!
        // Strategy: connect an untouched node to a connected one, keeping
        // maximum flexibility and controlling parity.
        (6, 1) => find_untouched_to_connected(state, 6),

        _ => None,
    }
}

/// Find an untouched node (0 connections) and connect it to a node the human used (1 connection).
fn find_untouched_to_connected(state: &GameState, initial_count: usize) -> Option<(usize, usize)> {
    let untouched = state
        .nodes
        .iter()
        .find(|node| node.id < initial_count && node.connection_count == 0)
        .map(|node| node.id);
    let connected = state
        .nodes
        .iter()
        .find(|node| node.id < initial_count && node.connection_count == 1)
        .map(|node| node.id);
    match (untouched, connected) {
        (Some(u), Some(c)) => Some((u, c)),
        _ => None,
    }
}
