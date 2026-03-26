use crate::node_classifier::classify_nodes;
use crate::pathfinding::{find_path_on_skeleton, find_self_loop_on_skeleton};
use crate::types::{GameState, Move, Player};
use crate::validation::validate_ai_move;

/// Generate all valid moves for the current player in the given state.
fn generate_valid_moves(state: &mut GameState) -> Vec<Move> {
    let classification = classify_nodes(state);
    let mut valid_moves = Vec::new();

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
                valid_moves.push(mov);
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
                valid_moves.push(mov);
            }
        }
    }

    valid_moves
}

/// Quick heuristic score for move ordering (higher = try first).
/// This makes alpha-beta pruning dramatically more effective.
fn quick_move_score(state: &GameState, mov: &Move) -> f64 {
    let mut score = 0.0;

    let from_node = state.find_node(mov.from_node);
    let to_node = state.find_node(mov.to_node);

    // Prefer moves that kill nodes (bring to 3 connections)
    if let Some(n) = from_node {
        if n.connection_count == 2 {
            score += 20.0; // This move kills the source node
        }
    }
    if mov.from_node != mov.to_node {
        if let Some(n) = to_node {
            if n.connection_count == 2 {
                score += 20.0; // This move kills the target node
            }
        }
    }

    // Prefer connecting high-degree nodes (they're nearly dead anyway)
    if let Some(n) = from_node {
        score += n.connection_count as f64 * 5.0;
    }
    if let Some(n) = to_node {
        score += n.connection_count as f64 * 5.0;
    }

    // Slight preference for shorter paths (tighter, more constraining moves)
    score -= mov.polyline.len() as f64 * 0.1;

    score
}

/// Adaptive search depth based on game state complexity.
/// Deeper search when fewer moves remain (endgame).
fn choose_depth(num_moves: usize) -> usize {
    match num_moves {
        0 => 0,
        1..=2 => 4,  // Very few moves: search deep
        3..=4 => 3,  // Few moves: search moderately deep
        _ => 2,      // Many moves: standard depth
    }
}

pub fn find_best_move(state: &mut GameState) -> Option<Move> {
    let mut valid_moves = generate_valid_moves(state);
    if valid_moves.is_empty() {
        return None;
    }

    // Sort by quick heuristic for better pruning
    valid_moves.sort_by(|a, b| {
        quick_move_score(state, b)
            .partial_cmp(&quick_move_score(state, a))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let depth = choose_depth(valid_moves.len());

    let mut scored_moves: Vec<(Move, f64)> = Vec::new();
    for mov in valid_moves {
        let mut sim_state = state.clone_for_simulation();
        sim_state.apply_move(mov.clone());
        let score = minimax(
            &mut sim_state,
            depth - 1,
            f64::NEG_INFINITY,
            f64::INFINITY,
            false,
        );
        scored_moves.push((mov, score));
    }

    scored_moves.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    for (mov, _score) in scored_moves {
        if validate_ai_move(state, &mov).is_ok() {
            return Some(mov);
        }
    }

    None
}

fn minimax(
    state: &mut GameState,
    depth: usize,
    mut alpha: f64,
    mut beta: f64,
    is_maximizing: bool,
) -> f64 {
    if depth == 0 {
        return evaluate_position(state);
    }

    let mut valid_moves = generate_valid_moves(state);

    if valid_moves.is_empty() {
        return evaluate_position(state);
    }

    // Move ordering for better pruning
    let state_ref = &*state;
    valid_moves.sort_by(|a, b| {
        quick_move_score(state_ref, b)
            .partial_cmp(&quick_move_score(state_ref, a))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    if is_maximizing {
        let mut max_eval = f64::NEG_INFINITY;
        for mov in valid_moves {
            let mut sim_state = state.clone_for_simulation();
            sim_state.apply_move(mov);
            let eval = minimax(&mut sim_state, depth - 1, alpha, beta, false);
            max_eval = max_eval.max(eval);
            alpha = alpha.max(eval);
            if beta <= alpha {
                break;
            }
        }
        max_eval
    } else {
        let mut min_eval = f64::INFINITY;
        for mov in valid_moves {
            let mut sim_state = state.clone_for_simulation();
            sim_state.apply_move(mov);
            let eval = minimax(&mut sim_state, depth - 1, alpha, beta, true);
            min_eval = min_eval.min(eval);
            beta = beta.min(eval);
            if beta <= alpha {
                break;
            }
        }
        min_eval
    }
}

/// Evaluate position from AI's perspective.
/// Positive = good for AI, negative = good for human.
fn evaluate_position(state: &mut GameState) -> f64 {
    let classification = classify_nodes(state);
    let move_count =
        classification.legal_pairs.len() + classification.self_loop_candidates.len();
    let is_ai_turn = state.current_player == Player::AI;

    // Terminal: current player has no moves — they lose
    if move_count == 0 {
        return if is_ai_turn { -10000.0 } else { 10000.0 };
    }

    let mut score = 0.0;

    // 1. PARITY — the most important strategic concept in Sprouts.
    // Total remaining "lives" = sum of (3 - connection_count) for all nodes.
    // Each move consumes exactly 1 net life (uses 2 from endpoints, creates node with 1 remaining).
    // So the game lasts at most `remaining_lives - 1` more moves (need >=2 lives to make a move).
    // If that number is odd, the current player makes the last move (wins).
    let total_remaining_lives: i32 = state
        .nodes
        .iter()
        .map(|n| 3i32 - n.connection_count as i32)
        .filter(|&lives| lives > 0)
        .sum();
    let estimated_remaining_moves = (total_remaining_lives - 1).max(0);
    let current_player_makes_last_move = estimated_remaining_moves % 2 == 1;
    let parity_favors_ai = (is_ai_turn && current_player_makes_last_move)
        || (!is_ai_turn && !current_player_makes_last_move);
    score += if parity_favors_ai { 80.0 } else { -80.0 };

    // 2. MOBILITY — more available moves = more flexibility
    let move_value = move_count as f64 * 8.0;
    score += if is_ai_turn { move_value } else { -move_value };

    // 3. NODE CONTROL — constrained nodes (2 connections) are near death.
    // Having constrained nodes when it's your turn is bad — your options are limited.
    let mut constrained_active = 0;
    let mut flexible_active = 0;
    for node in &state.nodes {
        if !classification.active_nodes.contains(&node.id) {
            continue;
        }
        if node.connection_count == 2 {
            constrained_active += 1;
        } else {
            flexible_active += 1;
        }
    }
    // Constrained nodes hurt the current player (fewer choices)
    let constraint_penalty = constrained_active as f64 * 5.0;
    score += if is_ai_turn {
        -constraint_penalty
    } else {
        constraint_penalty
    };
    // Flexible nodes help the current player
    let flex_bonus = flexible_active as f64 * 3.0;
    score += if is_ai_turn { flex_bonus } else { -flex_bonus };

    // 4. ISOLATION — dead nodes reduce the total game length,
    // which can shift parity. Nodes with 3 connections are already dead.
    let dead_count = state.nodes.iter().filter(|n| n.connection_count >= 3).count();
    // More dead nodes = shorter game = parity may have shifted (already captured above)
    // But also: fewer nodes in play = less room to maneuver
    score -= dead_count as f64 * 1.0;

    score
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
