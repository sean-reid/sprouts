use crate::abstract_graph::{abstract_minimax, AbstractMove, AbstractState, TranspositionTable};
use crate::node_classifier::classify_nodes;
use crate::opening_book;
use crate::pathfinding::{find_path_on_skeleton, find_self_loop_on_skeleton};
use crate::types::{GameState, Move};
use crate::validation::validate_ai_move;

/// Generate all concrete moves with pixel paths (expensive — called once at root only).
fn generate_concrete_moves(state: &mut GameState) -> Vec<Move> {
    let classification = classify_nodes(state);
    let mut moves = Vec::new();

    for (from_id, to_id) in &classification.legal_pairs {
        if let Some(path) = find_path_on_skeleton(state, *from_id, *to_id) {
            if path.len() < 3 {
                continue;
            }
            let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
            let mov = Move {
                from_node: *from_id,
                to_node: *to_id,
                polyline: path,
                new_node_pos,
                player: state.current_player,
            };
            if validate_ai_move(state, &mov).is_ok() {
                moves.push(mov);
            }
        }
    }

    for &node_id in &classification.self_loop_candidates {
        if let Some(path) = find_self_loop_on_skeleton(state, node_id) {
            if path.len() < 5 {
                continue;
            }
            let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
            let mov = Move {
                from_node: node_id,
                to_node: node_id,
                polyline: path,
                new_node_pos,
                player: state.current_player,
            };
            if validate_ai_move(state, &mov).is_ok() {
                moves.push(mov);
            }
        }
    }

    moves
}

pub fn find_best_move(state: &mut GameState) -> Option<Move> {
    // 1. OPENING BOOK — skip search for well-known positions
    if let Some((from, to)) = opening_book::lookup_opening(state) {
        if let Some(path) = find_path_on_skeleton(state, from, to) {
            if path.len() >= 3 {
                let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
                let mov = Move {
                    from_node: from,
                    to_node: to,
                    polyline: path,
                    new_node_pos,
                    player: state.current_player,
                };
                if validate_ai_move(state, &mov).is_ok() {
                    return Some(mov);
                }
            }
        }
    }

    // 2. GENERATE CONCRETE MOVES (pixel-level, expensive, only at root)
    let concrete_moves = generate_concrete_moves(state);
    if concrete_moves.is_empty() {
        return None;
    }

    // 3. BUILD ABSTRACT STATE for fast deep search
    let abstract_state = AbstractState::from_game_state(state);
    let depth = abstract_state.choose_depth();
    let mut tt = TranspositionTable::new();

    // 4. SCORE each concrete move via abstract minimax
    //    Only the root level uses pixel paths. The entire search tree below
    //    uses the abstract representation (~1000× faster per node).
    let mut scored: Vec<(usize, f64)> = Vec::new();
    for (idx, mov) in concrete_moves.iter().enumerate() {
        let abstract_mov = AbstractMove {
            from: mov.from_node,
            to: mov.to_node,
        };
        let mut sim = abstract_state.clone();
        sim.apply_move(&abstract_mov);

        // After AI's move, it's human's turn → minimizer
        let score = abstract_minimax(
            &sim,
            depth.saturating_sub(1),
            f64::NEG_INFINITY,
            f64::INFINITY,
            false,
            &mut tt,
        );
        scored.push((idx, score));
    }

    // 5. RETURN best concrete move that passes final validation
    scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    for (idx, _score) in scored {
        let mov = &concrete_moves[idx];
        if validate_ai_move(state, mov).is_ok() {
            return Some(mov.clone());
        }
    }

    None
}

pub fn find_optimal_node_placement(
    path: &[crate::types::Point],
    existing_nodes: &[crate::types::Node],
) -> crate::types::Point {
    if path.len() < 3 {
        return crate::types::Point::new(400.0, 400.0);
    }
    if path.len() == 3 {
        return path[1];
    }
    let mut best_pos = path[1];
    let mut max_min_distance = 0.0;
    for i in 1..(path.len() - 1) {
        let candidate = path[i];
        let mut min_dist = f64::INFINITY;
        for node in existing_nodes {
            let dx = candidate.x - node.position.x;
            let dy = candidate.y - node.position.y;
            let dist = (dx * dx + dy * dy).sqrt();
            min_dist = min_dist.min(dist);
        }
        if min_dist > max_min_distance {
            max_min_distance = min_dist;
            best_pos = candidate;
        }
    }
    best_pos
}

pub fn select_ai_move(state: &mut GameState) -> Option<Move> {
    find_best_move(state)
}
