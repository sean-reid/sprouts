//! Lightweight abstract game state for fast minimax search.
//!
//! No pixel grids, no skeleton, no pathfinding. Move generation and
//! simulation are O(n²) where n is the number of nodes (typically 3–20).

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

#[derive(Clone)]
pub struct AbstractState {
    pub nodes: Vec<(usize, u8)>, // (node_id, connection_count)
    pub is_ai_turn: bool,
    pub next_node_id: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct AbstractMove {
    pub from: usize,
    pub to: usize,
}

pub struct TranspositionTable {
    table: HashMap<u64, (f64, usize)>,
}

impl TranspositionTable {
    pub fn new() -> Self {
        Self {
            table: HashMap::with_capacity(4096),
        }
    }

    pub fn get(&self, key: u64, min_depth: usize) -> Option<f64> {
        self.table
            .get(&key)
            .and_then(|&(score, d)| if d >= min_depth { Some(score) } else { None })
    }

    pub fn insert(&mut self, key: u64, score: f64, depth: usize) {
        match self.table.get(&key) {
            Some(&(_, d)) if d >= depth => {}
            _ => {
                self.table.insert(key, (score, depth));
            }
        }
    }
}

impl AbstractState {
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

    pub fn generate_moves(&self) -> Vec<AbstractMove> {
        let active: Vec<(usize, u8)> = self
            .nodes
            .iter()
            .filter(|(_, cc)| *cc < 3)
            .copied()
            .collect();

        let mut moves = Vec::new();
        for i in 0..active.len() {
            for j in (i + 1)..active.len() {
                moves.push(AbstractMove {
                    from: active[i].0,
                    to: active[j].0,
                });
            }
        }
        for &(id, cc) in &active {
            if cc <= 1 {
                moves.push(AbstractMove { from: id, to: id });
            }
        }
        moves
    }

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
        self.nodes.push((self.next_node_id, 2));
        self.next_node_id += 1;
        self.is_ai_turn = !self.is_ai_turn;
    }

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
                score += 30.0; // Kills node — high priority
            }
            score += cc as f64 * 5.0;
        }
        if mov.from != mov.to {
            if let Some(&(_, cc)) = self.nodes.iter().find(|(id, _)| *id == mov.to) {
                if cc == 2 {
                    score += 30.0;
                }
                score += cc as f64 * 5.0;
            }
        }
        score
    }

    /// Search depth based on game complexity.
    /// The abstract model is fast enough for very deep search.
    pub fn choose_depth(&self) -> usize {
        let total_lives: i32 = self
            .nodes
            .iter()
            .map(|(_, cc)| 3i32 - *cc as i32)
            .filter(|&l| l > 0)
            .sum();
        let max_remaining = (total_lives - 1).max(0) as usize;
        let num_moves = self.generate_moves().len();

        if max_remaining <= 16 {
            max_remaining // Endgame: solve completely
        } else if num_moves <= 4 {
            12
        } else if num_moves <= 8 {
            10
        } else if num_moves <= 15 {
            8
        } else {
            6
        }
    }

    /// Evaluate position from AI's perspective.
    pub fn evaluate(&self) -> f64 {
        let moves = self.generate_moves();
        let move_count = moves.len();

        if move_count == 0 {
            return if self.is_ai_turn { -10000.0 } else { 10000.0 };
        }

        let mut score = 0.0;

        // 1. PARITY — dominant strategic factor
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

        // Weight parity more heavily as the game progresses
        let parity_weight = if est_remaining <= 4 {
            300.0 // Near-terminal: parity is almost everything
        } else if est_remaining <= 8 {
            150.0
        } else {
            100.0
        };
        score += if parity_favors_ai {
            parity_weight
        } else {
            -parity_weight
        };

        // 2. MOBILITY — more moves = more flexibility for the current player
        let move_value = move_count as f64 * 6.0;
        score += if self.is_ai_turn {
            move_value
        } else {
            -move_value
        };

        // 3. NODE CONSTRAINTS
        let constrained = self
            .nodes
            .iter()
            .filter(|(_, cc)| *cc == 2)
            .count();
        let active = self.nodes.iter().filter(|(_, cc)| *cc < 3).count();
        let flexible = active.saturating_sub(constrained);

        // Constrained nodes hurt the current player
        score += if self.is_ai_turn {
            -(constrained as f64 * 6.0)
        } else {
            constrained as f64 * 6.0
        };
        // Flexible nodes help
        score += if self.is_ai_turn {
            flexible as f64 * 3.0
        } else {
            -(flexible as f64 * 3.0)
        };

        // 4. OPPONENT RESTRICTION — 1-ply lookahead within eval.
        // For each of our moves, count opponent's responses. Prefer positions
        // where the opponent has fewer options on average.
        // Only do this when the move count is small (not too expensive).
        if move_count <= 10 {
            let mut total_opp_moves = 0usize;
            for mov in &moves {
                let mut sim = self.clone();
                sim.apply_move(mov);
                total_opp_moves += sim.generate_moves().len();
            }
            let avg_opp = total_opp_moves as f64 / move_count as f64;
            // Fewer opponent moves = better for current player
            score += if self.is_ai_turn {
                -avg_opp * 3.0
            } else {
                avg_opp * 3.0
            };
        }

        score
    }
}

/// Alpha-beta minimax with transposition table.
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

    // Move ordering
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
