//! Time-boxed alpha-beta search over the face model.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use crate::face::{FaceModel, Move};

/// Thinking time per move.
pub const SEARCH_BUDGET_MS: f64 = 800.0;
/// Searched positions between two reads of the clock.
pub const CHECK_EVERY: usize = 128;

/// Milliseconds since the clock started.
pub trait Clock {
    fn elapsed_ms(&mut self) -> f64;
}

#[cfg(not(target_arch = "wasm32"))]
pub struct WallClock(std::time::Instant);

#[cfg(target_arch = "wasm32")]
pub struct WallClock(f64);

impl WallClock {
    #[cfg(not(target_arch = "wasm32"))]
    pub fn start() -> Self {
        Self(std::time::Instant::now())
    }

    #[cfg(target_arch = "wasm32")]
    pub fn start() -> Self {
        Self(js_sys::Date::now())
    }
}

impl Clock for WallClock {
    #[cfg(not(target_arch = "wasm32"))]
    fn elapsed_ms(&mut self) -> f64 {
        self.0.elapsed().as_secs_f64() * 1000.0
    }

    #[cfg(target_arch = "wasm32")]
    fn elapsed_ms(&mut self) -> f64 {
        js_sys::Date::now() - self.0
    }
}

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

/// Thinking time, charged in positions whose moves are listed and checked
/// against the clock every `CHECK_EVERY` of them while armed. Once over,
/// it stays over: the search returns junk and inserts nothing.
pub struct Budget<C: Clock> {
    clock: C,
    limit_ms: f64,
    spent: usize,
    armed: bool,
    over: bool,
}

impl<C: Clock> Budget<C> {
    pub fn new(clock: C, limit_ms: f64) -> Self {
        Self {
            clock,
            limit_ms,
            spent: 0,
            armed: false,
            over: false,
        }
    }

    fn spend(&mut self) {
        self.spent += 1;
        if self.armed && !self.over && self.spent.is_multiple_of(CHECK_EVERY) {
            self.over = self.clock.elapsed_ms() >= self.limit_ms;
        }
    }

    pub fn exhausted(&self) -> bool {
        self.over
    }

    pub fn spent(&self) -> usize {
        self.spent
    }
}

/// What a budgeted search settled on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pick {
    pub index: usize,
    /// Depth of the last iteration that finished within the budget.
    pub depth: usize,
    pub cut_off: bool,
    /// Positions charged to the budget over the whole search.
    pub spent: usize,
}

/// Index into `candidates` of the move the search prefers from `root`:
/// iterative deepening as far as the game can last, keeping the pick of
/// the last depth that finished. Depth 1 always finishes.
pub fn best_candidate<C: Clock>(
    root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
) -> Pick {
    let max_depth = root.max_depth();
    best_candidate_to(root, candidates, budget, max_depth)
}

pub fn best_candidate_to<C: Clock>(
    mut root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
    max_depth: usize,
) -> Pick {
    let mut tt = TranspositionTable::new();
    let mut pick = Pick {
        index: 0,
        depth: 0,
        cut_off: false,
        spent: 0,
    };
    for depth in 1..=max_depth {
        budget.armed = depth > 1;
        let mut best_score = f64::NEG_INFINITY;
        let mut best_at_depth = 0;
        for (idx, mov) in candidates.iter().enumerate() {
            root.apply_move(mov);
            let score = abstract_minimax(
                &mut root,
                depth - 1,
                f64::NEG_INFINITY,
                f64::INFINITY,
                false,
                &mut tt,
                budget,
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
            pick.cut_off = true;
            break;
        }
        pick.index = best_at_depth;
        pick.depth = depth;
    }
    pick.spent = budget.spent();
    pick
}

impl AbstractState {
    pub fn new(face: FaceModel, is_ai_turn: bool) -> Self {
        Self { face, is_ai_turn }
    }

    /// One move per class of isomorphic moves, at `FaceModel::search_moves`
    /// cost.
    pub fn generate_moves(&self) -> Vec<Move> {
        self.face.search_moves()
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

    /// Plies left in the game at most: every move spends one life net.
    pub fn max_depth(&self) -> usize {
        (self.lives() - 1).max(1) as usize
    }

    /// Lists this position's moves and, when there are ten or fewer, each
    /// reply's, charging `budget` for every list after the first.
    pub fn evaluate<C: Clock>(&self, budget: &mut Budget<C>) -> f64 {
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
                budget.spend();
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
pub fn abstract_minimax<C: Clock>(
    state: &mut AbstractState,
    depth: usize,
    mut alpha: f64,
    mut beta: f64,
    is_maximizing: bool,
    tt: &mut TranspositionTable,
    budget: &mut Budget<C>,
) -> f64 {
    let hash = state.hash_key();
    if let Some(score) = tt.get(hash, depth) {
        return score;
    }
    budget.spend();
    if budget.exhausted() {
        return 0.0;
    }

    if depth == 0 {
        let score = state.evaluate(budget);
        if !budget.exhausted() {
            tt.insert(hash, score, depth);
        }
        return score;
    }

    let mut moves = state.generate_moves();

    if moves.is_empty() {
        let score = state.evaluate(budget);
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

    /// Stands still, so a budget on it never runs out.
    struct Frozen;

    impl Clock for Frozen {
        fn elapsed_ms(&mut self) -> f64 {
            0.0
        }
    }

    /// One millisecond per searched position, read on the check cadence.
    struct NodeClock {
        reads: usize,
    }

    impl Clock for NodeClock {
        fn elapsed_ms(&mut self) -> f64 {
            self.reads += 1;
            (self.reads * CHECK_EVERY) as f64
        }
    }

    fn unlimited() -> Budget<Frozen> {
        Budget::new(Frozen, f64::INFINITY)
    }

    fn nodes(limit: usize) -> Budget<NodeClock> {
        Budget::new(NodeClock { reads: 0 }, limit as f64)
    }

    fn fresh(n: usize) -> AbstractState {
        AbstractState::new(FaceModel::new(n), true)
    }

    /// `n` nodes after `played` moves, the first listed each time.
    fn after(n: usize, played: usize) -> AbstractState {
        let mut s = fresh(n);
        for _ in 0..played {
            let mov = s.generate_moves()[0].clone();
            s.apply_move(&mov);
        }
        s
    }

    /// One node played out: a loop, then the last chord. Nobody can move.
    fn finished() -> AbstractState {
        let mut s = fresh(1);
        while let Some(mov) = s.generate_moves().first().cloned() {
            s.apply_move(&mov);
        }
        s
    }

    /// Plain minimax to a fixed depth with the same evaluation, no table
    /// and no pruning.
    fn plain_minimax(state: &mut AbstractState, depth: usize, maximizing: bool) -> f64 {
        let moves = state.generate_moves();
        if depth == 0 || moves.is_empty() {
            return state.evaluate(&mut unlimited());
        }
        let mut best = if maximizing {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
        for mov in &moves {
            state.apply_move(mov);
            let score = plain_minimax(state, depth - 1, !maximizing);
            state.undo_move();
            best = if maximizing {
                best.max(score)
            } else {
                best.min(score)
            };
        }
        best
    }

    fn plain_best(mut root: AbstractState, candidates: &[Move], depth: usize) -> usize {
        let mut best = (f64::NEG_INFINITY, 0);
        for (idx, mov) in candidates.iter().enumerate() {
            root.apply_move(mov);
            let score = plain_minimax(&mut root, depth - 1, false);
            root.undo_move();
            if score > best.0 {
                best = (score, idx);
            }
        }
        best.1
    }

    #[test]
    fn generate_moves_2_fresh_nodes() {
        assert_eq!(fresh(2).generate_moves().len(), 2);
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
        assert!(s.evaluate(&mut unlimited()) < 0.0);
        s.is_ai_turn = false;
        assert!(s.evaluate(&mut unlimited()) > 0.0);
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
        let depth = s.max_depth();
        let before = s.face.clone();
        let score = abstract_minimax(
            &mut s,
            depth,
            f64::NEG_INFINITY,
            f64::INFINITY,
            true,
            &mut tt,
            &mut unlimited(),
        );
        assert!(score.is_finite());
        assert_eq!(s.face, before);
    }

    #[test]
    fn evaluate_charges_the_budget_for_each_reply_it_lists() {
        let s = fresh(2);
        let mut budget = unlimited();
        s.evaluate(&mut budget);
        assert_eq!(budget.spent(), 2);
    }

    #[test]
    fn the_clock_is_read_once_per_check_interval_and_the_cut_lands_in_it() {
        let root = after(20, 3);
        assert_eq!(root.face.node_count(), 23);
        let candidates = root.generate_moves();
        let limit = 5000;
        let pick = best_candidate(root, &candidates, &mut nodes(limit));
        assert!(pick.cut_off);
        assert!(pick.depth >= 1);
        assert!(pick.spent >= limit, "{pick:?}");
        assert!(pick.spent <= limit + CHECK_EVERY, "{pick:?}");
    }

    #[test]
    fn a_cut_off_search_keeps_the_last_finished_depth_and_repeats_itself() {
        let root = fresh(6);
        let candidates: Vec<Move> = root.face.moves().into_iter().take(12).collect();
        let first = best_candidate(root.clone(), &candidates, &mut nodes(200));
        let second = best_candidate(root.clone(), &candidates, &mut nodes(200));
        assert_eq!(first, second);
        assert!(first.cut_off);
        assert_eq!(first.depth, 1);

        let mut by_eval = (f64::NEG_INFINITY, 0);
        let mut probe = root.clone();
        for (idx, mov) in candidates.iter().enumerate() {
            probe.apply_move(mov);
            let score = probe.evaluate(&mut unlimited());
            probe.undo_move();
            if score > by_eval.0 {
                by_eval = (score, idx);
            }
        }
        assert_eq!(first.index, by_eval.1);

        let small = fresh(2);
        let all = small.generate_moves();
        let roomy = best_candidate(small, &all, &mut unlimited());
        assert!(!roomy.cut_off);
        assert!(roomy.depth > 1);
    }

    #[test]
    fn a_cut_off_search_keeps_the_pick_of_the_depth_it_finished() {
        let root = fresh(5);
        let candidates = root.generate_moves();
        let two = best_candidate_to(root.clone(), &candidates, &mut unlimited(), 2);
        assert_eq!(two.depth, 2);
        let cut = best_candidate(root.clone(), &candidates, &mut nodes(two.spent + 1));
        assert!(cut.cut_off);
        assert!(cut.depth >= 2);
        let same = best_candidate_to(root, &candidates, &mut unlimited(), cut.depth);
        assert_eq!(cut.index, same.index);
    }

    #[test]
    fn depth_one_finishes_under_any_budget() {
        let root = fresh(4);
        let candidates = root.face.moves();
        let pick = best_candidate(root, &candidates, &mut nodes(0));
        assert_eq!(pick.depth, 1);
        assert!(pick.cut_off);
        assert!(pick.spent > 0);
    }

    #[test]
    fn a_search_that_reaches_the_end_of_the_game_is_not_cut_off() {
        let root = fresh(2);
        let candidates = root.face.moves();
        let pick = best_candidate(root.clone(), &candidates, &mut nodes(1 << 20));
        assert!(!pick.cut_off);
        assert_eq!(pick.depth, root.max_depth());
    }

    #[test]
    fn the_time_boxed_search_agrees_with_plain_minimax_at_depth_four() {
        for played in [0, 1] {
            let root = after(4, played);
            let candidates = root.generate_moves();
            let boxed = best_candidate_to(root.clone(), &candidates, &mut unlimited(), 4);
            assert_eq!(boxed.depth, 4);
            assert_eq!(
                boxed.index,
                plain_best(root, &candidates, 4),
                "after {played}"
            );
        }
    }

    #[test]
    fn twenty_nodes_answer_within_the_budget_on_the_wall_clock() {
        let root = after(20, 3);
        let candidates = root.generate_moves();
        let mut times = Vec::new();
        for _ in 0..10 {
            let started = std::time::Instant::now();
            let mut budget = Budget::new(WallClock::start(), SEARCH_BUDGET_MS);
            let pick = best_candidate(root.clone(), &candidates, &mut budget);
            let took = started.elapsed().as_secs_f64() * 1000.0;
            println!(
                "{took:.1} ms, depth {}, {} positions",
                pick.depth, pick.spent
            );
            times.push(took);
        }
        for took in times {
            assert!(took <= SEARCH_BUDGET_MS * 1.1, "{took:.1} ms");
        }
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
            &mut unlimited(),
        );
        assert!(score < 0.0);
    }

    #[test]
    fn enclosing_a_node_removes_its_moves() {
        let mut open = fresh(3);
        let mut walled = fresh(3);
        let lp = walled
            .face
            .moves()
            .into_iter()
            .find(|m| m.a == 0 && m.b == 0 && !m.enclosed.is_empty())
            .unwrap();
        walled.apply_move(&lp);
        let plain = open
            .face
            .moves()
            .into_iter()
            .find(|m| m.a == 0 && m.b == 0 && m.enclosed.is_empty())
            .unwrap();
        open.apply_move(&plain);
        assert!(open.face.moves().len() > walled.face.moves().len());
    }
}
