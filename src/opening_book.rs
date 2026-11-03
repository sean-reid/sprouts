//! Opening book for the AI's first reply when the human opens.
//!
//! Sprouts solved results:
//!   n=4: First player wins (max 11 moves)
//!   n=5: First player wins (max 14 moves)
//!   n=6: Second player wins (max 17 moves)
//!
//! As second player the AI loses with perfect play at 4 and 5 nodes and
//! wins at 6. The book gives its first reply to the common human openings.

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::*;

    /// Simulate the human's first move on a game with `n` initial nodes.
    /// Connects node 0 to node 1 and adds a new node.
    fn state_after_human_first_move(n: usize) -> GameState {
        let mut gs = GameState::new(n);
        let mid = Point::new(
            (gs.nodes[0].position.x + gs.nodes[1].position.x) / 2.0,
            (gs.nodes[0].position.y + gs.nodes[1].position.y) / 2.0 + 50.0,
        );
        let mov = Move::new(
            0,
            1,
            vec![gs.nodes[0].position, mid, gs.nodes[1].position],
            mid,
            Player::Human,
            gs.board_size,
        );
        gs.apply_move(mov);
        gs
    }

    #[test]
    fn returns_none_for_move_0() {
        let gs = GameState::new(4);
        assert!(lookup_opening(&gs).is_none());
    }

    #[test]
    fn returns_some_for_move_1_n4() {
        let gs = state_after_human_first_move(4);
        let result = lookup_opening(&gs);
        assert!(
            result.is_some(),
            "Should suggest AI's first response for n=4"
        );
    }

    #[test]
    fn returns_some_for_move_1_n5() {
        let gs = state_after_human_first_move(5);
        assert!(lookup_opening(&gs).is_some());
    }

    #[test]
    fn returns_some_for_move_1_n6() {
        let gs = state_after_human_first_move(6);
        assert!(lookup_opening(&gs).is_some());
    }

    #[test]
    fn returns_none_for_move_2_plus() {
        let mut gs = state_after_human_first_move(4);
        let (u, c) = lookup_opening(&gs).unwrap();
        let from_pos = gs.find_node(u).unwrap().position;
        let to_pos = gs.find_node(c).unwrap().position;
        let mid = Point::new(
            (from_pos.x + to_pos.x) / 2.0,
            (from_pos.y + to_pos.y) / 2.0 + 50.0,
        );
        let mov = Move::new(
            u,
            c,
            vec![from_pos, mid, to_pos],
            mid,
            Player::AI,
            gs.board_size,
        );
        gs.apply_move(mov);
        assert!(lookup_opening(&gs).is_none());
    }

    #[test]
    fn returned_nodes_are_valid() {
        let gs = state_after_human_first_move(4);
        let (u, c) = lookup_opening(&gs).unwrap();
        let untouched = gs.find_node(u).unwrap();
        let connected = gs.find_node(c).unwrap();
        assert_eq!(untouched.connection_count, 0);
        assert_eq!(connected.connection_count, 1);
    }
}
