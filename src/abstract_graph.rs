//! Time-boxed alpha-beta search over the face model.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use crate::face::{canonical_form, FaceModel, Move, NodeId};

/// Thinking time per move.
pub const SEARCH_BUDGET_MS: f64 = 800.0;
/// Searched positions between two reads of the clock.
pub const CHECK_EVERY: usize = 64;

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

/// What a stored score says about the position's true value at its depth.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bound {
    Exact,
    /// The search was cut off, so the value is at least this.
    Lower,
    /// Every move fell short of the window, so the value is at most this.
    Upper,
}

struct Entry {
    score: f64,
    depth: usize,
    bound: Bound,
    best: Option<Move>,
}

#[derive(Default)]
pub struct TranspositionTable {
    table: HashMap<u64, Entry>,
}

impl TranspositionTable {
    pub fn new() -> Self {
        Self {
            table: HashMap::with_capacity(4096),
        }
    }

    /// A stored score the window can use at `depth`: an exact one, a lower
    /// bound at or past beta, or an upper bound at or under alpha.
    pub fn probe(&self, key: u64, depth: usize, alpha: f64, beta: f64) -> Option<f64> {
        let entry = self.table.get(&key)?;
        if entry.depth < depth {
            return None;
        }
        match entry.bound {
            Bound::Exact => Some(entry.score),
            Bound::Lower if entry.score >= beta => Some(entry.score),
            Bound::Upper if entry.score <= alpha => Some(entry.score),
            _ => None,
        }
    }

    pub fn best_move(&self, key: u64) -> Option<&Move> {
        self.table.get(&key).and_then(|e| e.best.as_ref())
    }

    /// Keeps the deeper of the old and new entries, the new one on a tie.
    pub fn store(&mut self, key: u64, score: f64, depth: usize, bound: Bound, best: Option<Move>) {
        if self.table.get(&key).is_some_and(|e| e.depth > depth) {
            return;
        }
        self.table.insert(
            key,
            Entry {
                score,
                depth,
                bound,
                best,
            },
        );
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

    /// Reads the clock every `CHECK_EVERY` positions; only an armed budget
    /// runs out.
    fn spend(&mut self) {
        self.spent += 1;
        if !self.over && self.spent.is_multiple_of(CHECK_EVERY) {
            let late = self.clock.elapsed_ms() >= self.limit_ms;
            self.over = self.armed && late;
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

/// One search: the table, one killer move per ply, and the budget.
pub struct Search<'a, C: Clock> {
    pub tt: TranspositionTable,
    killers: Vec<Option<Move>>,
    budget: &'a mut Budget<C>,
}

impl<'a, C: Clock> Search<'a, C> {
    pub fn new(budget: &'a mut Budget<C>) -> Self {
        Self {
            tt: TranspositionTable::new(),
            killers: Vec::new(),
            budget,
        }
    }

    /// Alpha-beta minimax, fail-soft. Leaves `state` as it found it. The
    /// result means nothing once the budget is exhausted.
    pub fn minimax(
        &mut self,
        state: &mut AbstractState,
        depth: usize,
        ply: usize,
        mut alpha: f64,
        mut beta: f64,
        maximizing: bool,
    ) -> f64 {
        let key = state.hash_key();
        if let Some(score) = self.tt.probe(key, depth, alpha, beta) {
            return score;
        }
        self.budget.spend();
        if self.budget.exhausted() {
            return 0.0;
        }
        if depth == 0 || state.face.is_game_over() {
            let score = state.evaluate();
            self.tt.store(key, score, depth, Bound::Exact, None);
            return score;
        }

        let moves = self.ordered_moves(state, key, ply);
        let (alpha_in, beta_in) = (alpha, beta);
        let mut best_score = if maximizing {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
        let mut best_move = None;
        for mov in &moves {
            state.apply_move(mov);
            let score = self.minimax(state, depth - 1, ply + 1, alpha, beta, !maximizing);
            state.undo_move();
            if self.budget.exhausted() {
                return 0.0;
            }
            if maximizing {
                if score > best_score {
                    best_score = score;
                    best_move = Some(mov.clone());
                }
                alpha = alpha.max(score);
            } else {
                if score < best_score {
                    best_score = score;
                    best_move = Some(mov.clone());
                }
                beta = beta.min(score);
            }
            if beta <= alpha {
                self.remember_killer(ply, mov);
                break;
            }
        }
        let bound = if best_score <= alpha_in {
            Bound::Upper
        } else if best_score >= beta_in {
            Bound::Lower
        } else {
            Bound::Exact
        };
        self.tt.store(key, best_score, depth, bound, best_move);
        best_score
    }

    /// The table's best move first, this ply's killer next, the rest by
    /// the degree heuristic.
    fn ordered_moves(&self, state: &AbstractState, key: u64, ply: usize) -> Vec<Move> {
        let mut moves = state.generate_moves();
        let scores: Vec<f64> = moves.iter().map(|m| state.quick_move_score(m)).collect();
        let mut indices: Vec<usize> = (0..moves.len()).collect();
        indices.sort_by(|&a, &b| scores[b].total_cmp(&scores[a]));
        moves = indices.into_iter().map(|i| moves[i].clone()).collect();
        let lead = [
            self.tt.best_move(key),
            self.killers.get(ply).and_then(Option::as_ref),
        ];
        for preferred in lead.into_iter().rev().flatten() {
            if let Some(at) = moves.iter().position(|m| m == preferred) {
                let mov = moves.remove(at);
                moves.insert(0, mov);
            }
        }
        moves
    }

    fn remember_killer(&mut self, ply: usize, mov: &Move) {
        if self.killers.len() <= ply {
            self.killers.resize(ply + 1, None);
        }
        self.killers[ply] = Some(mov.clone());
    }
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

/// Each iteration tries the candidates in the order the last one ranked
/// them; ties at the root go to the lower index.
pub fn best_candidate_to<C: Clock>(
    mut root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
    max_depth: usize,
) -> Pick {
    let mut pick = Pick {
        index: 0,
        depth: 0,
        cut_off: false,
        spent: 0,
    };
    if candidates.is_empty() {
        return pick;
    }
    let mut search = Search::new(budget);
    let mut order: Vec<usize> = (0..candidates.len()).collect();
    let mut scores = vec![f64::NEG_INFINITY; candidates.len()];
    for depth in 1..=max_depth {
        search.budget.armed = depth > 1;
        let mut best = (f64::NEG_INFINITY, usize::MAX);
        for &idx in &order {
            root.apply_move(&candidates[idx]);
            let score = search.minimax(
                &mut root,
                depth - 1,
                1,
                f64::NEG_INFINITY,
                f64::INFINITY,
                false,
            );
            root.undo_move();
            if search.budget.exhausted() {
                break;
            }
            scores[idx] = score;
            if score > best.0 || (score == best.0 && idx < best.1) {
                best = (score, idx);
            }
        }
        if search.budget.exhausted() {
            pick.cut_off = true;
            break;
        }
        pick.index = best.1;
        pick.depth = depth;
        order.sort_by(|&a, &b| scores[b].total_cmp(&scores[a]));
    }
    drop(search);
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

    /// Hashes the position up to renaming, and whose turn it is.
    pub fn hash_key(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        canonical_form(&self.face).hash(&mut hasher);
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

    /// Room to move, read off the boundary walks: pairs of live nodes that
    /// share a face, plus the nodes with two lives for a loop.
    pub fn mobility(&self) -> usize {
        let mut live_in_face: Vec<Vec<NodeId>> = vec![Vec::new(); self.face.face_count()];
        for v in 0..self.face.node_count() {
            match self.face.degree(v) {
                3 => {}
                0 => live_in_face[self.face.corner_face(v, None).expect("home face")].push(v),
                _ => {
                    for &h in self.face.rotation(v) {
                        live_in_face[self.face.face_of(h)].push(v);
                    }
                }
            }
        }
        let pairs: usize = live_in_face
            .iter_mut()
            .map(|nodes| {
                nodes.sort_unstable();
                nodes.dedup();
                nodes.len() * nodes.len().saturating_sub(1) / 2
            })
            .sum();
        pairs + self.degrees().filter(|&d| d <= 1).count()
    }

    /// Static score from the AI's side, without listing a move.
    pub fn evaluate(&self) -> f64 {
        if self.face.is_game_over() {
            return if self.is_ai_turn { -10000.0 } else { 10000.0 };
        }

        let mut score = 0.0;

        // Parity of the moves left decides who moves last.
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

        let move_value = self.mobility() as f64 * 6.0;
        score += if self.is_ai_turn {
            move_value
        } else {
            -move_value
        };

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

        score
    }
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

    /// `n` nodes after `played` moves, picking the move at `pick` in each
    /// list, wrapping round.
    fn after_picking(n: usize, played: usize, pick: usize) -> AbstractState {
        let mut s = fresh(n);
        for _ in 0..played {
            let moves = s.generate_moves();
            if moves.is_empty() {
                break;
            }
            let mov = moves[pick % moves.len()].clone();
            s.apply_move(&mov);
        }
        s
    }

    /// `n` nodes after `played` moves, the first listed each time.
    fn after(n: usize, played: usize) -> AbstractState {
        after_picking(n, played, 0)
    }

    /// One node played out: a loop, then the last chord. Nobody can move.
    fn finished() -> AbstractState {
        let mut s = fresh(1);
        while let Some(mov) = s.generate_moves().first().cloned() {
            s.apply_move(&mov);
        }
        s
    }

    fn search_to(state: &mut AbstractState, depth: usize, maximizing: bool) -> f64 {
        let mut budget = unlimited();
        let mut search = Search::new(&mut budget);
        search.minimax(
            state,
            depth,
            0,
            f64::NEG_INFINITY,
            f64::INFINITY,
            maximizing,
        )
    }

    /// Plain minimax to a fixed depth with the same evaluation, no table
    /// and no pruning.
    fn plain_minimax(state: &mut AbstractState, depth: usize, maximizing: bool) -> f64 {
        let moves = state.generate_moves();
        if depth == 0 || moves.is_empty() {
            return state.evaluate();
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

    /// Alpha-beta with a table that files every returned score as exact,
    /// the way the search did before bounds were stored.
    fn unsound_alpha_beta(
        state: &mut AbstractState,
        depth: usize,
        mut alpha: f64,
        mut beta: f64,
        maximizing: bool,
        table: &mut HashMap<u64, (f64, usize)>,
    ) -> f64 {
        let key = state.hash_key();
        if let Some(&(score, d)) = table.get(&key) {
            if d >= depth {
                return score;
            }
        }
        let mut moves = state.generate_moves();
        if depth == 0 || moves.is_empty() {
            let score = state.evaluate();
            table.insert(key, (score, depth));
            return score;
        }
        let scores: Vec<f64> = moves.iter().map(|m| state.quick_move_score(m)).collect();
        let mut indices: Vec<usize> = (0..moves.len()).collect();
        indices.sort_by(|&a, &b| scores[b].total_cmp(&scores[a]));
        moves = indices.into_iter().map(|i| moves[i].clone()).collect();
        let mut best = if maximizing {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
        for mov in &moves {
            state.apply_move(mov);
            let score = unsound_alpha_beta(state, depth - 1, alpha, beta, !maximizing, table);
            state.undo_move();
            if maximizing {
                best = best.max(score);
                alpha = alpha.max(score);
            } else {
                best = best.min(score);
                beta = beta.min(score);
            }
            if beta <= alpha {
                break;
            }
        }
        table.insert(key, (best, depth));
        best
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
        assert!(s.evaluate() < 0.0);
        s.is_ai_turn = false;
        assert!(s.evaluate() > 0.0);
    }

    /// Four lone nodes: six pairs share the one face and each can loop.
    /// A chord adds a node that shares the face with the four, and the
    /// two it joined still have a life for a loop.
    #[test]
    fn mobility_counts_pairs_sharing_a_face_and_nodes_with_a_loop_left() {
        let mut s = fresh(4);
        assert_eq!(s.mobility(), 6 + 4);
        let chord = s.generate_moves().into_iter().find(|m| m.a != m.b).unwrap();
        s.apply_move(&chord);
        assert_eq!(s.mobility(), 10 + 4);
        let walled = {
            let mut w = fresh(3);
            let lp = w
                .face
                .moves()
                .into_iter()
                .find(|m| m.a == 0 && m.b == 0 && m.enclosed == vec![2])
                .unwrap();
            w.apply_move(&lp);
            w
        };
        // Outside: 0, 1 and 3 share a face (3 pairs); inside: 0, 2, 3 (3
        // pairs); 1 and 2 can still loop.
        assert_eq!(walled.mobility(), 3 + 3 + 2);
    }

    #[test]
    fn same_state_same_hash() {
        assert_eq!(fresh(2).hash_key(), fresh(2).hash_key());
    }

    #[test]
    fn isomorphic_states_share_a_hash_and_labels_do_not_matter() {
        let mut a = fresh(4);
        let mut b = fresh(4);
        let chords: Vec<Move> = a
            .generate_moves()
            .into_iter()
            .filter(|m| m.a != m.b)
            .collect();
        a.apply_move(&chords[0]);
        let other = b
            .face
            .moves()
            .into_iter()
            .find(|m| m.a == 2 && m.b == 3)
            .unwrap();
        b.apply_move(&other);
        assert_ne!(a.face, b.face);
        assert_eq!(a.hash_key(), b.hash_key());
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
        let depth = s.max_depth();
        let before = s.face.clone();
        let score = search_to(&mut s, depth, true);
        assert!(score.is_finite());
        assert_eq!(s.face, before);
    }

    #[test]
    fn the_table_hands_back_only_what_the_window_can_use() {
        let mut tt = TranspositionTable::new();
        tt.store(1, 50.0, 3, Bound::Lower, None);
        assert_eq!(tt.probe(1, 3, 0.0, 40.0), Some(50.0));
        assert_eq!(tt.probe(1, 3, 0.0, 60.0), None);
        assert_eq!(tt.probe(1, 4, 0.0, 40.0), None);
        tt.store(2, -20.0, 2, Bound::Upper, None);
        assert_eq!(tt.probe(2, 1, -10.0, 10.0), Some(-20.0));
        assert_eq!(tt.probe(2, 1, -30.0, 10.0), None);
        tt.store(3, 7.0, 1, Bound::Exact, None);
        assert_eq!(tt.probe(3, 1, 100.0, 200.0), Some(7.0));
        tt.store(3, 9.0, 0, Bound::Exact, None);
        assert_eq!(tt.probe(3, 1, 100.0, 200.0), Some(7.0));
        tt.store(3, 11.0, 1, Bound::Exact, None);
        assert_eq!(tt.probe(3, 1, 100.0, 200.0), Some(11.0));
    }

    /// Filing a cut-off score as exact lets a later probe through a wider
    /// window read a bound as the value: on the fresh 4-node board at
    /// depth 3 the old table said 108 where plain minimax says 111. Over
    /// these positions it disagrees eight times; the bounded table never
    /// does.
    #[test]
    fn storing_bounds_keeps_the_search_exact_where_an_exact_only_table_was_not() {
        let depth = 3;
        let mut disagreements = 0;
        for n in 3..=4 {
            for played in 0..4 {
                for pick in 0..3 {
                    let root = after_picking(n, played, pick);
                    let plain = plain_minimax(&mut root.clone(), depth, true);
                    let mut table = HashMap::new();
                    let unsound = unsound_alpha_beta(
                        &mut root.clone(),
                        depth,
                        f64::NEG_INFINITY,
                        f64::INFINITY,
                        true,
                        &mut table,
                    );
                    if unsound != plain {
                        disagreements += 1;
                    }
                    let sound = search_to(&mut root.clone(), depth, true);
                    assert_eq!(sound, plain, "{n} nodes after {played} picking {pick}");
                }
            }
        }
        assert_eq!(disagreements, 8);
    }

    #[test]
    fn the_table_move_and_the_killer_lead_the_ordering() {
        let state = fresh(5);
        let key = state.hash_key();
        let moves = state.generate_moves();
        assert!(moves.len() >= 3);
        let mut budget = unlimited();
        let mut search = Search::new(&mut budget);
        let plain = search.ordered_moves(&state, key, 0);
        let killer = plain[plain.len() - 1].clone();
        let table_move = plain[plain.len() - 2].clone();
        search.remember_killer(0, &killer);
        assert_eq!(search.ordered_moves(&state, key, 0)[0], killer);
        search
            .tt
            .store(key, 0.0, 1, Bound::Exact, Some(table_move.clone()));
        let led = search.ordered_moves(&state, key, 0);
        assert_eq!(led[0], table_move);
        assert_eq!(led[1], killer);
        assert_eq!(led.len(), plain.len());
        assert_eq!(search.ordered_moves(&state, key, 1)[0], table_move);
        assert_ne!(search.ordered_moves(&state, key, 1)[1], killer);
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
    fn a_cut_off_search_keeps_a_finished_depth_and_repeats_itself() {
        let root = fresh(6);
        let candidates: Vec<Move> = root.face.moves().into_iter().take(12).collect();
        let first = best_candidate(root.clone(), &candidates, &mut nodes(0));
        let second = best_candidate(root.clone(), &candidates, &mut nodes(0));
        assert_eq!(first, second);
        assert!(first.cut_off);
        assert!(first.depth >= 1);
        let same = best_candidate_to(root.clone(), &candidates, &mut unlimited(), first.depth);
        assert_eq!(first.index, same.index);

        let small = fresh(2);
        let all = small.generate_moves();
        let roomy = best_candidate(small, &all, &mut unlimited());
        assert!(!roomy.cut_off);
        assert!(roomy.depth > 1);
    }

    #[test]
    fn the_depth_one_pick_is_the_best_by_evaluation() {
        let root = fresh(6);
        let candidates: Vec<Move> = root.face.moves().into_iter().take(12).collect();
        let one = best_candidate_to(root.clone(), &candidates, &mut unlimited(), 1);
        assert_eq!(one.depth, 1);
        let mut by_eval = (f64::NEG_INFINITY, 0);
        let mut probe = root;
        for (idx, mov) in candidates.iter().enumerate() {
            probe.apply_move(mov);
            let score = probe.evaluate();
            probe.undo_move();
            if score > by_eval.0 {
                by_eval = (score, idx);
            }
        }
        assert_eq!(one.index, by_eval.1);
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

    /// Depth 2 from here lists far more than one check interval of
    /// positions, so a budget that is over at the first read still leaves
    /// the depth-1 pick.
    #[test]
    fn depth_one_finishes_under_any_budget() {
        let root = after(24, 2);
        let candidates = root.generate_moves();
        let pick = best_candidate(root, &candidates, &mut nodes(0));
        assert_eq!(pick.depth, 1);
        assert!(pick.cut_off);
        assert!(pick.spent >= candidates.len());
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
    fn no_candidates_give_the_empty_pick() {
        let pick = best_candidate(fresh(2), &[], &mut unlimited());
        assert_eq!(pick.depth, 0);
        assert_eq!(pick.index, 0);
    }

    #[test]
    fn the_time_boxed_search_agrees_with_plain_minimax_at_depth_four() {
        for played in [0, 1, 2] {
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
        assert!(search_to(&mut s, 5, true) < 0.0);
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
