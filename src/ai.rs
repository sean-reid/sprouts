use crate::abstract_graph::{abstract_minimax, AbstractMove, AbstractState, TranspositionTable};
use crate::node_classifier::classify_nodes;
use crate::opening_book;
use crate::pathfinding::{find_path_on_skeleton, find_self_loop_on_skeleton};
use crate::types::{GameState, Move};
use crate::validation::validate_ai_move;

/// Generate all concrete moves with pixel paths (expensive — called once at root).
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
    // 1. OPENING BOOK
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

    // 2. GENERATE CONCRETE MOVES (pixel-level, once)
    let concrete_moves = generate_concrete_moves(state);
    if concrete_moves.is_empty() {
        return None;
    }
    if concrete_moves.len() == 1 {
        return Some(concrete_moves.into_iter().next().unwrap()); // Only one option
    }

    // 3. ABSTRACT SEARCH with iterative deepening.
    //    Each iteration's TT entries improve move ordering for the next.
    let abstract_state = AbstractState::from_game_state(state);
    let max_depth = abstract_state.choose_depth();
    let mut tt = TranspositionTable::new();

    let mut best_idx = 0;

    for depth in 1..=max_depth {
        let mut best_score = f64::NEG_INFINITY;
        for (idx, mov) in concrete_moves.iter().enumerate() {
            let abstract_mov = AbstractMove {
                from: mov.from_node,
                to: mov.to_node,
            };
            let mut sim = abstract_state.clone();
            sim.apply_move(&abstract_mov);

            let score = abstract_minimax(
                &sim,
                depth.saturating_sub(1),
                f64::NEG_INFINITY,
                f64::INFINITY,
                false, // After AI's move, human minimizes
                &mut tt,
            );

            if score > best_score {
                best_score = score;
                best_idx = idx;
            }
        }
    }

    // 4. RETURN best concrete move that passes validation
    // Try the best first, then fall through to others
    if validate_ai_move(state, &concrete_moves[best_idx]).is_ok() {
        return Some(concrete_moves[best_idx].clone());
    }
    for (idx, mov) in concrete_moves.iter().enumerate() {
        if idx == best_idx {
            continue;
        }
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
        let c = crate::types::BOARD_SIZE as f64 / 2.0;
        return crate::types::Point::new(c, c);
    }
    if path.len() == 3 {
        return path[1];
    }

    // Only consider the middle 80% of the path (skip endpoints where nodes cluster)
    let start_idx = (path.len() as f64 * 0.1).ceil() as usize;
    let end_idx = (path.len() as f64 * 0.9).floor() as usize;
    let start_idx = start_idx.max(1);
    let end_idx = end_idx.min(path.len() - 1).max(start_idx + 1);

    let mut best_pos = path[path.len() / 2]; // Default: midpoint
    let mut max_min_distance = 0.0;

    for i in start_idx..end_idx {
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
