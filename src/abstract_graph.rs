//! Lightweight abstract game state for fast minimax search.
//!
//! Tracks only node connection counts and current player — no pixel grids,
//! no skeleton, no pathfinding. Move generation and simulation are O(n²)
//! where n is the number of nodes (typically 3–15), compared to the pixel-level
//! approach which costs ~50–100ms per node due to 800×800 skeleton regeneration.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

/// Abstract game state: just node connection counts + whose turn it is.
#[derive(Clone)]
pub struct AbstractState {
    /// (node_id, connection_count) for every node in the game.
    pub nodes: Vec<(usize, u8)>,
    /// True if it's the AI's turn.
    pub is_ai_turn: bool,
    /// Next node ID for creating midpoints.
    pub next_node_id: usize,
}

/// An abstract move: connect `from` to `to` (self-loop when from == to).
#[derive(Clone, Copy, Debug)]
pub struct AbstractMove {
    pub from: usize,
    pub to: usize,
}

/// Transposition table for caching minimax results.
pub struct TranspositionTable {
    table: HashMap<u64, (f64, usize)>, // hash → (score, depth)
}

impl TranspositionTable {
    pub fn new() -> Self {
        Self {
            table: HashMap::with_capacity(1024),
        }
    }

    pub fn get(&self, key: u64, min_depth: usize) -> Option<f64> {
        self.table
            .get(&key)
            .and_then(|&(score, d)| if d >= min_depth { Some(score) } else { None })
    }

    pub fn insert(&mut self, key: u64, score: f64, depth: usize) {
        match self.table.get(&key) {
            Some(&(_, d)) if d >= depth => {} // Don't overwrite deeper entries
            _ => {
                self.table.insert(key, (score, depth));
            }
        }
    }
}

impl AbstractState {
    /// Build abstract state from the full pixel-level game state.
    pub fn from_game_state(state: &crate::types::GameState) -> Self {
        let nodes = state
            .nodes
            .iter()
            .map(|n| (n.id, n.connection_count))
            .collect();
        let is_ai_turn = state.current_player == crate::types::Player::AI;
        Self {
            nodes,
            is_ai_turn,
            next_node_id: state.next_node_id,
        }
    }

    /// Generate all abstract moves (over-approximates: any two active nodes
    /// can connect, any node with ≤1 connections can self-loop).
    pub fn generate_moves(&self) -> Vec<AbstractMove> {
        let active: Vec<(usize, u8)> = self
            .nodes
            .iter()
            .filter(|(_, cc)| *cc < 3)
            .copied()
            .collect();

        let mut moves = Vec::new();

        // All pairs of active nodes
        for i in 0..active.len() {
            for j in (i + 1)..active.len() {
                moves.push(AbstractMove {
                    from: active[i].0,
                    to: active[j].0,
                });
            }
        }

        // Self-loops for nodes with ≤1 connections (need 2 free)
        for &(id, cc) in &active {
            if cc <= 1 {
                moves.push(AbstractMove { from: id, to: id });
            }
        }

        moves
    }

    /// Apply a move (mutates in place).
    pub fn apply_move(&mut self, mov: &AbstractMove) {
        if mov.from == mov.to {
            if let Some(node) = self.nodes.iter_mut().find(|(id, _)| *id == mov.from) {
                node.1 += 2;
            }
        } else {
            if let Some(node) = self.nodes.iter_mut().find(|(id, _)| *id == mov.from) {
                node.1 += 1;
            }
            if let Some(node) = self.nodes.iter_mut().find(|(id, _)| *id == mov.to) {
                node.1 += 1;
            }
        }

        // Midpoint node starts with 2 connections
        self.nodes.push((self.next_node_id, 2));
        self.next_node_id += 1;
        self.is_ai_turn = !self.is_ai_turn;
    }

    /// Canonical hash for transposition table.
    pub fn hash_key(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        let mut counts: Vec<u8> = self.nodes.iter().map(|(_, cc)| *cc).collect();
        counts.sort_unstable();
        counts.hash(&mut hasher);
        self.is_ai_turn.hash(&mut hasher);
        hasher.finish()
    }

    /// Quick heuristic for move ordering (higher = try first).
    pub fn quick_move_score(&self, mov: &AbstractMove) -> f64 {
        let mut score = 0.0;
        if let Some(&(_, cc)) = self.nodes.iter().find(|(id, _)| *id == mov.from) {
            if cc == 2 {
                score += 20.0; // Kills the node
            }
            score += cc as f64 * 5.0;
        }
        if mov.from != mov.to {
            if let Some(&(_, cc)) = self.nodes.iter().find(|(id, _)| *id == mov.to) {
                if cc == 2 {
                    score += 20.0;
                }
                score += cc as f64 * 5.0;
            }
        }
        if mov.from == mov.to {
            score -= 5.0; // Self-loops are usually suboptimal
        }
        score
    }

    /// Choose search depth based on game complexity.
    /// With abstract moves costing microseconds, we can search much deeper.
    pub fn choose_depth(&self) -> usize {
        let total_lives: i32 = self
            .nodes
            .iter()
            .map(|(_, cc)| 3i32 - *cc as i32)
            .filter(|&l| l > 0)
            .sum();
        let max_remaining = (total_lives - 1).max(0) as usize;
        let num_moves = self.generate_moves().len();

        if max_remaining <= 12 {
            max_remaining // Endgame: solve to terminal
        } else if num_moves <= 3 {
            8
        } else if num_moves <= 6 {
            6
        } else {
            5
        }
    }

    /// Evaluate position from AI's perspective.
    /// Positive = good for AI, negative = good for human.
    pub fn evaluate(&self) -> f64 {
        let moves = self.generate_moves();
        let move_count = moves.len();

        // Terminal: current player has no moves → they lose
        if move_count == 0 {
            return if self.is_ai_turn { -10000.0 } else { 10000.0 };
        }

        let mut score = 0.0;

        // 1. PARITY — the dominant strategic factor in Sprouts.
        let total_lives: i32 = self
            .nodes
            .iter()
            .map(|(_, cc)| 3i32 - *cc as i32)
            .filter(|&l| l > 0)
            .sum();
        let est_remaining = (total_lives - 1).max(0);
        let current_makes_last = est_remaining % 2 == 1;
        let parity_favors_ai = (self.is_ai_turn && current_makes_last)
            || (!self.is_ai_turn && !current_makes_last);
        score += if parity_favors_ai { 100.0 } else { -100.0 };

        // 2. MOBILITY — more moves = more flexibility for the current player
        let move_value = move_count as f64 * 8.0;
        score += if self.is_ai_turn {
            move_value
        } else {
            -move_value
        };

        // 3. NODE CONSTRAINTS — constrained nodes (2 connections) hurt the current player
        let active_count = self.nodes.iter().filter(|(_, cc)| *cc < 3).count();
        let constrained = self.nodes.iter().filter(|(_, cc)| *cc == 2).count();
        let flexible = active_count.saturating_sub(constrained);

        let constraint_penalty = constrained as f64 * 5.0;
        score += if self.is_ai_turn {
            -constraint_penalty
        } else {
            constraint_penalty
        };

        let flex_bonus = flexible as f64 * 3.0;
        score += if self.is_ai_turn {
            flex_bonus
        } else {
            -flex_bonus
        };

        // 4. NEAR-TERMINAL bonus: if only 1-2 moves remain, weight parity more heavily
        if est_remaining <= 3 {
            score += if parity_favors_ai { 200.0 } else { -200.0 };
        }

        score
    }
}

/// Alpha-beta minimax on the abstract state with transposition table.
pub fn abstract_minimax(
    state: &AbstractState,
    depth: usize,
    mut alpha: f64,
    mut beta: f64,
    is_maximizing: bool,
    tt: &mut TranspositionTable,
) -> f64 {
    let hash = state.hash_key();
    if let Some(score) = tt.get(hash, depth) {
        return score;
    }

    if depth == 0 {
        let score = state.evaluate();
        tt.insert(hash, score, depth);
        return score;
    }

    let mut moves = state.generate_moves();

    if moves.is_empty() {
        let score = state.evaluate();
        tt.insert(hash, score, depth);
        return score;
    }

    // Move ordering for better alpha-beta pruning
    moves.sort_by(|a, b| {
        state
            .quick_move_score(b)
            .partial_cmp(&state.quick_move_score(a))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let score = if is_maximizing {
        let mut max_eval = f64::NEG_INFINITY;
        for mov in &moves {
            let mut sim = state.clone();
            sim.apply_move(mov);
            let eval = abstract_minimax(&sim, depth - 1, alpha, beta, false, tt);
            max_eval = max_eval.max(eval);
            alpha = alpha.max(eval);
            if beta <= alpha {
                break;
            }
        }
        max_eval
    } else {
        let mut min_eval = f64::INFINITY;
        for mov in &moves {
            let mut sim = state.clone();
            sim.apply_move(mov);
            let eval = abstract_minimax(&sim, depth - 1, alpha, beta, true, tt);
            min_eval = min_eval.min(eval);
            beta = beta.min(eval);
            if beta <= alpha {
                break;
            }
        }
        min_eval
    };

    tt.insert(hash, score, depth);
    score
}
