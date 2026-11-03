use crate::ai;
use crate::node_classifier::NodeClassification;
use crate::pathfinding;
use crate::types::{GameState, Move, Point};
use crate::validation;

// The classifier only checks spatial connectivity, so a pair it calls legal
// still needs a drawable, validating path before the game counts as live.
pub fn is_game_over(state: &GameState, classification: &NodeClassification) -> bool {
    if classification.active_nodes.is_empty() {
        return true;
    }
    if classification.legal_pairs.is_empty() && classification.self_loop_candidates.is_empty() {
        return true;
    }

    for &(from_id, to_id) in &classification.legal_pairs {
        let paths: [Option<Vec<Point>>; 3] = [
            pathfinding::find_path_on_skeleton(state, from_id, to_id),
            pathfinding::find_path_relaxed(state, from_id, to_id),
            pathfinding::find_path_tight(state, from_id, to_id),
        ];
        if paths
            .into_iter()
            .flatten()
            .any(|path| path.len() >= 3 && drawable(state, from_id, to_id, path))
        {
            return false;
        }
    }
    for &node_id in &classification.self_loop_candidates {
        let paths: [Option<Vec<Point>>; 2] = [
            pathfinding::find_self_loop_on_skeleton(state, node_id),
            pathfinding::generate_geometric_self_loop(state, node_id),
        ];
        if paths
            .into_iter()
            .flatten()
            .any(|path| path.len() >= 5 && drawable(state, node_id, node_id, path))
        {
            return false;
        }
    }
    true
}

fn drawable(state: &GameState, from_node: usize, to_node: usize, polyline: Vec<Point>) -> bool {
    let new_node_pos = ai::find_optimal_node_placement(&polyline, state);
    let mov = Move {
        from_node,
        to_node,
        polyline,
        new_node_pos,
        player: state.current_player,
    };
    validation::validate_ai_move(state, &mov).is_ok()
        || validation::validate_ai_move_tight(state, &mov).is_ok()
}
