//! Fixed-depth alpha-beta search over the face model.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use crate::face::{FaceModel, Move};

#[derive(Clone)]
pub struct AbstractState {
    face: FaceModel,
    pub is_ai_turn: bool,
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

/// Work the search may do, counted in visited positions so a run is
/// repeatable. Once spent, the search returns junk and inserts nothing.
pub struct NodeBudget {
    spent: usize,
    limit: usize,
}

impl NodeBudget {
    pub fn new(limit: usize) -> Self {
        Self { spent: 0, limit }
    }

    pub fn exhausted(&self) -> bool {
        self.spent > self.limit
    }
}

impl AbstractState {
    pub fn new(face: FaceModel, is_ai_turn: bool) -> Self {
        Self { face, is_ai_turn }
    }

    /// Lists every legal move, so it costs what `FaceModel::moves` costs.
    pub fn generate_moves(&self) -> Vec<Move> {
        self.face.moves()
    }

    pub fn apply_move(&mut self, mov: &Move) {
        self.face.apply(mov).expect("a listed move applies");
        self.is_ai_turn = !self.is_ai_turn;
    }

    pub fn undo_move(&mut self) {
        self.face.undo();
        self.is_ai_turn = !self.is_ai_turn;
    }

    fn degrees(&self) -> impl Iterator<Item = u8> + '_ {
        (0..self.face.node_count()).map(|v| self.face.degree(v))
    }

    fn lives(&self) -> i32 {
        self.degrees().map(|d| 3 - i32::from(d)).sum()
    }

    pub fn hash_key(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        self.face.hash(&mut hasher);
        self.is_ai_turn.hash(&mut hasher);
        hasher.finish()
    }

    pub fn quick_move_score(&self, mov: &Move) -> f64 {
        let ends = if mov.a == mov.b {
            vec![mov.a]
        } else {
            vec![mov.a, mov.b]
        };
        ends.into_iter()
            .map(|v| {
                let d = f64::from(self.face.degree(v));
                if d == 2.0 {
                    30.0 + d * 5.0
                } else {
                    d * 5.0
                }
            })
            .sum()
    }

    pub fn choose_depth(&self) -> usize {
        let max_remaining = (self.lives() - 1).max(0) as usize;
        let num_moves = self.generate_moves().len();

        if max_remaining <= 16 {
            max_remaining
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

    pub fn evaluate(&self) -> f64 {
        let moves = self.generate_moves();
        let move_count = moves.len();

        if move_count == 0 {
            return if self.is_ai_turn { -10000.0 } else { 10000.0 };
        }

        let mut score = 0.0;

        // 1. PARITY — dominant strategic factor
        let est_remaining = (self.lives() - 1).max(0);
        let current_makes_last = est_remaining % 2 == 1;
        let parity_favors_ai =
            (self.is_ai_turn && current_makes_last) || (!self.is_ai_turn && !current_makes_last);

        let parity_weight = if est_remaining <= 4 {
            300.0
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
        let constrained = self.degrees().filter(|&d| d == 2).count();
        let active = self.degrees().filter(|&d| d < 3).count();
        let flexible = active.saturating_sub(constrained);

        score += if self.is_ai_turn {
            -(constrained as f64 * 6.0)
        } else {
            constrained as f64 * 6.0
        };
        score += if self.is_ai_turn {
            flexible as f64 * 3.0
        } else {
            -(flexible as f64 * 3.0)
        };

        // 4. OPPONENT RESTRICTION
        if move_count <= 10 {
            let mut sim = self.clone();
            let mut total_opp_moves = 0usize;
            for mov in &moves {
                sim.apply_move(mov);
                total_opp_moves += sim.generate_moves().len();
                sim.undo_move();
            }
            let avg_opp = total_opp_moves as f64 / move_count as f64;
            score += if self.is_ai_turn {
                -avg_opp * 3.0
            } else {
                avg_opp * 3.0
            };
        }

        score
    }
}

/// Alpha-beta minimax with transposition table. Leaves `state` as it found
/// it. The result means nothing once `budget` is exhausted.
pub fn abstract_minimax(
    state: &mut AbstractState,
    depth: usize,
    mut alpha: f64,
    mut beta: f64,
    is_maximizing: bool,
    tt: &mut TranspositionTable,
    budget: &mut NodeBudget,
) -> f64 {
    let hash = state.hash_key();
    if let Some(score) = tt.get(hash, depth) {
        return score;
    }
    budget.spent += 1;
    if budget.exhausted() {
        return 0.0;
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
    let scores: Vec<f64> = moves.iter().map(|m| state.quick_move_score(m)).collect();
    let mut indices: Vec<usize> = (0..moves.len()).collect();
    indices.sort_by(|&a, &b| {
        scores[b]
            .partial_cmp(&scores[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    moves = indices.into_iter().map(|i| moves[i].clone()).collect();

    let score = if is_maximizing {
        let mut max_eval = f64::NEG_INFINITY;
        for mov in &moves {
            state.apply_move(mov);
            let eval = abstract_minimax(state, depth - 1, alpha, beta, false, tt, budget);
            state.undo_move();
            if budget.exhausted() {
                break;
            }
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
            state.apply_move(mov);
            let eval = abstract_minimax(state, depth - 1, alpha, beta, true, tt, budget);
            state.undo_move();
            if budget.exhausted() {
                break;
            }
            min_eval = min_eval.min(eval);
            beta = beta.min(eval);
            if beta <= alpha {
                break;
            }
        }
        min_eval
    };

    if !budget.exhausted() {
        tt.insert(hash, score, depth);
    }
    score
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh(n: usize) -> AbstractState {
        AbstractState::new(FaceModel::new(n), true)
    }

    /// One node played out: a loop, then the last chord. Nobody can move.
    fn finished() -> AbstractState {
        let mut s = fresh(1);
        while let Some(mov) = s.generate_moves().first().cloned() {
            s.apply_move(&mov);
        }
        s
    }

    #[test]
    fn generate_moves_2_fresh_nodes() {
        assert_eq!(fresh(2).generate_moves().len(), 3);
    }

    #[test]
    fn apply_move_pair_updates_degrees_and_turn() {
        let mut s = fresh(2);
        let chord = s.generate_moves().into_iter().find(|m| m.a != m.b).unwrap();
        s.apply_move(&chord);
        assert_eq!(s.face.degree(0), 1);
        assert_eq!(s.face.degree(1), 1);
        assert_eq!(s.face.degree(2), 2);
        assert!(!s.is_ai_turn);
    }

    #[test]
    fn undo_restores_the_position_and_turn() {
        let mut s = fresh(3);
        let before = s.clone();
        let mov = s.generate_moves()[0].clone();
        s.apply_move(&mov);
        s.undo_move();
        assert_eq!(s.face, before.face);
        assert_eq!(s.is_ai_turn, before.is_ai_turn);
        assert_eq!(s.hash_key(), before.hash_key());
    }

    #[test]
    fn apply_self_loop() {
        let mut s = fresh(2);
        let lp = s
            .generate_moves()
            .into_iter()
            .find(|m| m.a == 0 && m.b == 0)
            .unwrap();
        s.apply_move(&lp);
        assert_eq!(s.face.degree(0), 2);
    }

    #[test]
    fn evaluate_terminal_depends_on_whose_turn_it_is() {
        let mut s = finished();
        assert!(s.generate_moves().is_empty());
        s.is_ai_turn = true;
        assert!(s.evaluate() < 0.0);
        s.is_ai_turn = false;
        assert!(s.evaluate() > 0.0);
    }

    #[test]
    fn same_state_same_hash() {
        assert_eq!(fresh(2).hash_key(), fresh(2).hash_key());
    }

    #[test]
    fn different_states_different_hash() {
        let mut s2 = fresh(2);
        let mov = s2.generate_moves()[0].clone();
        s2.apply_move(&mov);
        s2.is_ai_turn = true;
        assert_ne!(fresh(2).hash_key(), s2.hash_key());
    }

    #[test]
    fn different_turn_different_hash() {
        let mut s2 = fresh(2);
        s2.is_ai_turn = false;
        assert_ne!(fresh(2).hash_key(), s2.hash_key());
    }

    #[test]
    fn minimax_2_node_game() {
        let mut s = fresh(2);
        let mut tt = TranspositionTable::new();
        let depth = s.choose_depth();
        let before = s.face.clone();
        let score = abstract_minimax(
            &mut s,
            depth,
            f64::NEG_INFINITY,
            f64::INFINITY,
            true,
            &mut tt,
            &mut NodeBudget::new(usize::MAX),
        );
        assert!(score.is_finite());
        assert_eq!(s.face, before);
    }

    #[test]
    fn an_exhausted_budget_stops_the_search_and_poisons_nothing() {
        let mut s = fresh(4);
        let mut tt = TranspositionTable::new();
        let mut budget = NodeBudget::new(5);
        abstract_minimax(
            &mut s,
            6,
            f64::NEG_INFINITY,
            f64::INFINITY,
            true,
            &mut tt,
            &mut budget,
        );
        assert!(budget.exhausted());
        assert_eq!(budget.spent, 6);
        assert!(tt.table.is_empty());
    }

    #[test]
    fn minimax_terminal_state() {
        let mut s = finished();
        s.is_ai_turn = true;
        let mut tt = TranspositionTable::new();
        let score = abstract_minimax(
            &mut s,
            5,
            f64::NEG_INFINITY,
            f64::INFINITY,
            true,
            &mut tt,
            &mut NodeBudget::new(usize::MAX),
        );
        assert!(score < 0.0);
    }

    #[test]
    fn enclosing_a_node_removes_its_moves() {
        let mut open = fresh(3);
        let mut walled = fresh(3);
        let lp = walled
            .generate_moves()
            .into_iter()
            .find(|m| m.a == 0 && m.b == 0 && !m.enclosed.is_empty())
            .unwrap();
        walled.apply_move(&lp);
        let plain = open
            .generate_moves()
            .into_iter()
            .find(|m| m.a == 0 && m.b == 0 && m.enclosed.is_empty())
            .unwrap();
        open.apply_move(&plain);
        assert!(open.generate_moves().len() > walled.generate_moves().len());
    }
}
