use crate::abstract_graph::{abstract_minimax, AbstractState, NodeBudget, TranspositionTable};
use crate::embedding;
use crate::face::Move as FaceMove;
use crate::node_classifier::{classify_nodes, NodeClassification};
use crate::opening_book;
use crate::pathfinding::{
    find_path_on_skeleton, find_path_relaxed, find_path_tight, find_self_loop_on_skeleton,
    generate_geometric_self_loop,
};
use crate::types::{GameState, Move, Player};
use crate::validation::{validate_ai_move, validate_ai_move_tight};

/// Generate all concrete moves with pixel paths (expensive — called once at root).
pub(crate) fn generate_concrete_moves_from_classification(
    state: &mut GameState,
    classification: &NodeClassification,
) -> Vec<Move> {
    let mut moves = Vec::new();
    let mut failed_pairs = Vec::new();

    for (from_id, to_id) in &classification.legal_pairs {
        let mut found = false;
        if let Some(path) = find_path_on_skeleton(state, *from_id, *to_id) {
            if path.len() >= 3 {
                let new_node_pos = find_optimal_node_placement(&path, state);
                let mov = Move::new(
                    *from_id,
                    *to_id,
                    path,
                    new_node_pos,
                    state.current_player,
                    state.board_size,
                );
                if validate_ai_move(state, &mov).is_ok() {
                    moves.push(mov);
                    found = true;
                }
            }
        }
        if !found {
            failed_pairs.push((*from_id, *to_id));
        }
    }

    for &node_id in &classification.self_loop_candidates {
        // Try skeleton-based first, then geometric fallback immediately
        let path = find_self_loop_on_skeleton(state, node_id)
            .or_else(|| generate_geometric_self_loop(state, node_id));
        if let Some(path) = path {
            if path.len() < 5 {
                continue;
            }
            let new_node_pos = find_optimal_node_placement(&path, state);
            let mov = Move::new(
                node_id,
                node_id,
                path,
                new_node_pos,
                state.current_player,
                state.board_size,
            );
            if validate_ai_move(state, &mov).is_ok() || validate_ai_move_tight(state, &mov).is_ok()
            {
                moves.push(mov);
            }
        }
    }

    // Retry ALL failed pairs (pathfinding failed OR validation failed) with
    // free-space A*.  No `moves.is_empty()` guard — a pair that failed the
    // skeleton pathfinder might succeed with the relaxed one.
    if !failed_pairs.is_empty() {
        let mut still_failed = Vec::new();
        for (from_id, to_id) in &failed_pairs {
            if let Some(path) = find_path_relaxed(state, *from_id, *to_id) {
                if path.len() < 3 {
                    still_failed.push((*from_id, *to_id));
                    continue;
                }
                let new_node_pos = find_optimal_node_placement(&path, state);
                let mov = Move::new(
                    *from_id,
                    *to_id,
                    path,
                    new_node_pos,
                    state.current_player,
                    state.board_size,
                );
                if validate_ai_move(state, &mov).is_ok() {
                    moves.push(mov);
                } else {
                    still_failed.push((*from_id, *to_id));
                }
            } else {
                still_failed.push((*from_id, *to_id));
            }
        }
        // Last resort: tightest buffer + tightest validation for narrow corridors
        for (from_id, to_id) in &still_failed {
            // Try all pathfinding methods with tight validation
            let paths: Vec<Option<Vec<crate::types::Point>>> = vec![
                find_path_tight(state, *from_id, *to_id),
                find_path_relaxed(state, *from_id, *to_id),
                find_path_on_skeleton(state, *from_id, *to_id),
            ];
            for path_opt in paths.into_iter().flatten() {
                if path_opt.len() < 3 {
                    continue;
                }
                let new_node_pos = find_optimal_node_placement(&path_opt, state);
                let mov = Move::new(
                    *from_id,
                    *to_id,
                    path_opt,
                    new_node_pos,
                    state.current_player,
                    state.board_size,
                );
                if validate_ai_move(state, &mov).is_ok()
                    || validate_ai_move_tight(state, &mov).is_ok()
                {
                    moves.push(mov);
                    break;
                }
            }
        }
    }

    moves
}

/// The drawn moves the face model can place, each with its topology.
pub(crate) fn embedded_moves(state: &GameState, moves: Vec<Move>) -> Vec<(Move, FaceMove)> {
    moves
        .into_iter()
        .filter_map(|mov| {
            let topology = embedding::embed(state, &mov).ok()?;
            Some((mov, topology))
        })
        .collect()
}

pub fn find_best_move(state: &mut GameState) -> Option<Move> {
    // 1. OPENING BOOK
    if let Some((from, to)) = opening_book::lookup_opening(state) {
        if let Some(path) = find_path_on_skeleton(state, from, to) {
            if path.len() >= 3 {
                let new_node_pos = find_optimal_node_placement(&path, state);
                let mov = Move::new(
                    from,
                    to,
                    path,
                    new_node_pos,
                    state.current_player,
                    state.board_size,
                );
                if validate_ai_move(state, &mov).is_ok() && embedding::embed(state, &mov).is_ok() {
                    return Some(mov);
                }
            }
        }
    }

    // 2. GENERATE CONCRETE MOVES (pixel-level, once)
    let classification = classify_nodes(state);
    let generated = generate_concrete_moves_from_classification(state, &classification);
    let mut candidates = embedded_moves(state, generated);
    if candidates.is_empty() {
        return None;
    }
    if candidates.len() == 1 {
        return candidates.pop().map(|(mov, _)| mov);
    }

    // 3. SEARCH over the face model from each candidate.
    let topologies: Vec<FaceMove> = candidates.iter().map(|(_, t)| t.clone()).collect();
    let best_idx = search_best_index(state, &topologies);

    // 4. RETURN best concrete move (already validated during generation)
    candidates.into_iter().nth(best_idx).map(|(mov, _)| mov)
}

/// Positions one search may visit across all its depths.
const SEARCH_NODES: usize = 50_000;

/// Index into `candidates` of the move the search prefers: the pick of the
/// deepest iteration that finished within the node budget.
pub(crate) fn search_best_index(state: &GameState, candidates: &[FaceMove]) -> usize {
    let mut root = AbstractState::new(state.face.clone(), state.current_player == Player::AI);
    let max_depth = root.choose_depth();
    let mut tt = TranspositionTable::new();
    let mut budget = NodeBudget::new(SEARCH_NODES);

    let mut best_idx = 0;

    for depth in 1..=max_depth {
        let mut best_score = f64::NEG_INFINITY;
        let mut best_at_depth = 0;
        for (idx, mov) in candidates.iter().enumerate() {
            root.apply_move(mov);
            let score = abstract_minimax(
                &mut root,
                depth.saturating_sub(1),
                f64::NEG_INFINITY,
                f64::INFINITY,
                false, // After AI's move, human minimizes
                &mut tt,
                &mut budget,
            );
            root.undo_move();
            if budget.exhausted() {
                break;
            }

            if score > best_score {
                best_score = score;
                best_at_depth = idx;
            }
        }
        if budget.exhausted() {
            break;
        }
        best_idx = best_at_depth;
    }
    best_idx
}

pub fn find_optimal_node_placement(
    path: &[crate::types::Point],
    state: &crate::types::GameState,
) -> crate::types::Point {
    if path.len() < 3 {
        if path.len() == 2 {
            return crate::types::Point::new(
                (path[0].x + path[1].x) / 2.0,
                (path[0].y + path[1].y) / 2.0,
            );
        }
        let c = crate::types::BOARD_SIZE as f64 / 2.0;
        return crate::types::Point::new(c, c);
    }
    if path.len() == 3 {
        return path[1];
    }
    rank_node_placements(path, state)
        .first()
        .copied()
        .unwrap_or(path[path.len() / 2])
}

/// Points of the middle 80% of the path ordered by clearance, the farthest
/// from every node and line first.
pub fn rank_node_placements(
    path: &[crate::types::Point],
    state: &crate::types::GameState,
) -> Vec<crate::types::Point> {
    use crate::geometry::point_to_segment_distance;

    let start_idx = ((path.len() as f64 * 0.1).ceil() as usize).max(1);
    let end_idx = ((path.len() as f64 * 0.9).floor() as usize)
        .min(path.len().saturating_sub(1))
        .max(start_idx + 1);

    let mut scored: Vec<(f64, crate::types::Point)> = path
        .iter()
        .take(end_idx)
        .skip(start_idx)
        .map(|candidate| {
            let mut min_dist = f64::INFINITY;
            for node in &state.nodes {
                min_dist = min_dist.min(candidate.distance_to(&node.position));
            }
            for line in &state.lines {
                for pair in line.polyline.windows(2) {
                    let dist = point_to_segment_distance(
                        candidate.x,
                        candidate.y,
                        pair[0].x,
                        pair[0].y,
                        pair[1].x,
                        pair[1].y,
                    );
                    min_dist = min_dist.min(dist);
                }
            }
            (min_dist, *candidate)
        })
        .collect();
    scored.sort_by(|a, b| b.0.total_cmp(&a.0));
    scored.into_iter().map(|(_, point)| point).collect()
}

pub fn select_ai_move(state: &mut GameState) -> Option<Move> {
    find_best_move(state)
}
