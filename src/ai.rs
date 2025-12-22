use crate::node_classifier::classify_nodes;
use crate::pathfinding::find_path_on_skeleton;
use crate::types::{GameState, Move};
use crate::validation::validate_ai_move;

const SEARCH_DEPTH: usize = 3;

pub fn find_best_move(state: &mut GameState, _max_time_ms: u32) -> Option<Move> {
    let classification = classify_nodes(state);
    if classification.legal_pairs.is_empty() && classification.self_loop_candidates.is_empty() {
        return None;
    }
    
    let mut valid_moves = Vec::new();

    for (from_id, to_id) in &classification.legal_pairs {
        if let Some(path) = find_path_on_skeleton(state, *from_id, *to_id) {
            if path.len() < 3 { continue; }
            let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
            let mov = Move {
                from_node: *from_id,
                to_node: *to_id,
                polyline: path,
                new_node_pos,
                player: state.current_player,
            };
            if validate_ai_move(state, &mov).is_ok() {
                valid_moves.push(mov);
            }
        }
    }

    for &node_id in &classification.self_loop_candidates {
        if let Some(path) = find_path_on_skeleton(state, node_id, node_id) {
            if path.len() < 5 { continue; }
            let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
            let mov = Move {
                from_node: node_id,
                to_node: node_id,
                polyline: path,
                new_node_pos,
                player: state.current_player,
            };
            if validate_ai_move(state, &mov).is_ok() {
                valid_moves.push(mov);
            }
        }
    }

    if valid_moves.is_empty() {
        return None;
    }

    // Evaluate and rank ALL valid moves
    let mut scored_moves: Vec<(Move, f64)> = Vec::new();
    for mov in valid_moves {
        let mut sim_state = state.clone();
        sim_state.apply_move(mov.clone());
        let score = -minimax(&mut sim_state, SEARCH_DEPTH - 1, f64::NEG_INFINITY, f64::INFINITY, false);
        scored_moves.push((mov, score));
    }

    // Sort by score (best first)
    scored_moves.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    // Try moves in order until we find one that passes strict validation
    for (mov, _score) in scored_moves {
        // Double-check with strict validation
        if validate_ai_move(state, &mov).is_ok() {
            return Some(mov);
        }
        // This move crossed a line or otherwise failed - try next best
    }

    // All moves failed validation (shouldn't happen)
    None
}

fn minimax(state: &mut GameState, depth: usize, mut alpha: f64, mut beta: f64, is_maximizing: bool) -> f64 {
    if depth == 0 {
        return evaluate_position(state);
    }
    
    let classification = classify_nodes(state);
    
    if classification.active_nodes.is_empty() || 
       (classification.legal_pairs.is_empty() && classification.self_loop_candidates.is_empty()) {
        return evaluate_position(state) * 1000.0;
    }
    
    let mut valid_moves = Vec::new();
    for (from_id, to_id) in &classification.legal_pairs {
        if let Some(path) = find_path_on_skeleton(state, *from_id, *to_id) {
            if path.len() < 3 { continue; }
            let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
            let mov = Move {
                from_node: *from_id,
                to_node: *to_id,
                polyline: path,
                new_node_pos,
                player: state.current_player,
            };
            if validate_ai_move(state, &mov).is_ok() {
                valid_moves.push(mov);
            }
        }
    }
    
    for &node_id in &classification.self_loop_candidates {
        if let Some(path) = find_path_on_skeleton(state, node_id, node_id) {
            if path.len() < 5 { continue; }
            let new_node_pos = find_optimal_node_placement(&path, &state.nodes);
            let mov = Move {
                from_node: node_id,
                to_node: node_id,
                polyline: path,
                new_node_pos,
                player: state.current_player,
            };
            if validate_ai_move(state, &mov).is_ok() {
                valid_moves.push(mov);
            }
        }
    }
    
    if valid_moves.is_empty() {
        return evaluate_position(state) * 1000.0;
    }
    
    if is_maximizing {
        let mut max_eval = f64::NEG_INFINITY;
        for mov in valid_moves {
            let mut sim_state = state.clone();
            sim_state.apply_move(mov);
            let eval = minimax(&mut sim_state, depth - 1, alpha, beta, false);
            max_eval = max_eval.max(eval);
            alpha = alpha.max(eval);
            if beta <= alpha { break; }
        }
        max_eval
    } else {
        let mut min_eval = f64::INFINITY;
        for mov in valid_moves {
            let mut sim_state = state.clone();
            sim_state.apply_move(mov);
            let eval = minimax(&mut sim_state, depth - 1, alpha, beta, true);
            min_eval = min_eval.min(eval);
            beta = beta.min(eval);
            if beta <= alpha { break; }
        }
        min_eval
    }
}

fn evaluate_position(state: &mut GameState) -> f64 {
    let classification = classify_nodes(state);
    let mut score = 0.0;
    let ai_move_count = classification.legal_pairs.len() + classification.self_loop_candidates.len();
    score += ai_move_count as f64 * 10.0;
    let active_count = classification.active_nodes.len();
    score += active_count as f64 * 5.0;
    score += classification.legal_pairs.len() as f64 * 3.0;
    if ai_move_count == 0 {
        score -= 10000.0;
    }
    for node in &state.nodes {
        if node.connection_count == 2 {
            score += 2.0;
        }
    }
    score
}

pub fn find_optimal_node_placement(path: &[crate::types::Point], existing_nodes: &[crate::types::Node]) -> crate::types::Point {
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
    find_best_move(state, 3000)
}
