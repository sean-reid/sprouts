//! Move choice: exact play from region nimbers where every region
//! solves, else a time-boxed alpha-beta search over the face model that
//! scores solved positions exactly.

use std::collections::HashMap;
use std::hash::{Hash, Hasher};

use crate::face::{parts, FaceModel, LiveScratch, Move, NodeId};
use crate::nimber::{Nimbers, EXACT_LIVES, NIMBER_BUDGET};

/// Thinking time per move.
pub const SEARCH_BUDGET_MS: f64 = 800.0;
/// Searched positions between two reads of the clock.
pub const CHECK_EVERY: usize = 64;
/// Score of a position the nimbers settle, from the winner's side.
const SOLVED: f64 = 10000.0;

/// Milliseconds since the clock started.
pub trait Clock {
    fn elapsed_ms(&mut self) -> f64;

    /// Called once per searched position, for a clock that counts them.
    fn tick(&mut self) {}
}

#[cfg(not(target_arch = "wasm32"))]
pub struct WallClock(std::time::Instant);

#[cfg(target_arch = "wasm32")]
pub struct WallClock(f64);

/// performance.now() of the worker's global scope, which is monotonic.
#[cfg(target_arch = "wasm32")]
fn performance_now() -> f64 {
    use wasm_bindgen::{JsCast, JsValue};
    let performance = js_sys::Reflect::get(&js_sys::global(), &JsValue::from_str("performance"))
        .expect("a global performance object");
    let now: js_sys::Function = js_sys::Reflect::get(&performance, &JsValue::from_str("now"))
        .expect("performance.now")
        .unchecked_into();
    now.call0(&performance)
        .expect("performance.now() returns")
        .as_f64()
        .expect("a number of milliseconds")
}

impl WallClock {
    #[cfg(not(target_arch = "wasm32"))]
    pub fn start() -> Self {
        Self(std::time::Instant::now())
    }

    #[cfg(target_arch = "wasm32")]
    pub fn start() -> Self {
        Self(performance_now())
    }
}

impl Clock for WallClock {
    #[cfg(not(target_arch = "wasm32"))]
    fn elapsed_ms(&mut self) -> f64 {
        self.0.elapsed().as_secs_f64() * 1000.0
    }

    #[cfg(target_arch = "wasm32")]
    fn elapsed_ms(&mut self) -> f64 {
        performance_now() - self.0
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
        self.clock.tick();
        if !self.over && self.spent.is_multiple_of(CHECK_EVERY) {
            self.check();
        }
    }

    fn check(&mut self) {
        let late = self.clock.elapsed_ms() >= self.limit_ms;
        self.over = self.armed && late;
    }

    /// Only an armed budget runs out. Arming reads the clock at once, so
    /// time the depth-1 pass already used up is noticed before the next
    /// position; disarming clears a cut, so that pass always finishes.
    pub fn arm(&mut self, armed: bool) {
        self.armed = armed;
        if armed {
            self.check();
        } else {
            self.over = false;
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
    /// Chosen from the nimbers of every region, with no search.
    pub exact: bool,
}

/// One search: the table, one killer move per ply, the budget, and the
/// solved regions it scores exactly.
pub struct Search<'a, C: Clock> {
    pub tt: TranspositionTable,
    killers: Vec<Option<Move>>,
    budget: &'a mut Budget<C>,
    nimbers: &'a mut Nimbers,
    live: LiveScratch,
}

impl<'a, C: Clock> Search<'a, C> {
    pub fn new(budget: &'a mut Budget<C>, nimbers: &'a mut Nimbers) -> Self {
        Self {
            tt: TranspositionTable::new(),
            killers: Vec::new(),
            budget,
            nimbers,
            live: LiveScratch::default(),
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
        let form = self.live.form(&state.face, 0);
        let key = key_of(form, state.is_ai_turn);
        if let Some(score) = self.tt.probe(key, depth, alpha, beta) {
            return score;
        }
        self.budget.spend();
        if self.budget.exhausted() {
            return 0.0;
        }
        if let Some(lost) = self.nimbers.known_lost(&parts(form)) {
            let score = if lost == state.is_ai_turn {
                -SOLVED
            } else {
                SOLVED
            };
            self.tt.store(key, score, usize::MAX, Bound::Exact, None);
            return score;
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

/// Positions the budgeted search charges and the milliseconds it takes on
/// 20 nodes after three moves, for the bench's rate line.
#[cfg(feature = "bench")]
pub fn search_rate() -> (usize, f64) {
    let root = twenty_nodes_after_three_moves();
    let candidates = root.generate_moves();
    let mut budget = Budget::new(WallClock::start(), SEARCH_BUDGET_MS);
    let started = std::time::Instant::now();
    let pick = best_candidate(root, &candidates, &mut budget, &mut Nimbers::new());
    (pick.spent, started.elapsed().as_secs_f64() * 1000.0)
}

#[cfg(any(test, feature = "bench"))]
fn twenty_nodes_after_three_moves() -> AbstractState {
    let mut s = AbstractState::new(FaceModel::new(20), true);
    for _ in 0..3 {
        let mov = s.generate_moves().first().cloned().expect("a move");
        s.apply_move(&mov);
    }
    s
}

/// Index into `candidates` of the move to play from `root`. When no
/// region of the root has more than `EXACT_LIVES` lives and each
/// candidate's child settles as won or lost within `NIMBER_BUDGET` child
/// positions, the pick is exact: a move leaving a lost position, the one
/// the quick heuristic ranks highest among them, or when none does the
/// move leaving the opponent the most room by `mobility`. Otherwise the
/// time-boxed search decides.
pub fn choose<C: Clock>(
    root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
    nimbers: &mut Nimbers,
) -> Pick {
    if root.face.regions().iter().all(|r| r.lives() <= EXACT_LIVES) {
        choose_within(root, candidates, budget, nimbers, NIMBER_BUDGET)
    } else {
        best_candidate(root, candidates, budget, nimbers)
    }
}

/// `choose` with `work` child positions for settling instead of
/// `NIMBER_BUDGET`, whatever the regions' size.
pub fn choose_within<C: Clock>(
    mut root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
    nimbers: &mut Nimbers,
    work: usize,
) -> Pick {
    let mut work = work;
    let mut children = Vec::with_capacity(candidates.len());
    let solved = candidates.iter().all(|mov| {
        root.apply_move(mov);
        let lost = nimbers.lost_in(&root.face, &mut work);
        let room = root.mobility();
        root.undo_move();
        children.push((lost.unwrap_or(false), room));
        lost.is_some()
    });
    if !solved || candidates.is_empty() {
        return best_candidate(root, candidates, budget, nimbers);
    }
    let rank = |idx: usize| {
        let (winning, room) = children[idx];
        let heuristic = root.quick_move_score(&candidates[idx]);
        (winning, if winning { 0 } else { room }, heuristic)
    };
    // Reversed so `max_by`, which keeps the last of equals, leaves the lowest index winning.
    let index = (0..candidates.len())
        .rev()
        .max_by(|&a, &b| rank(a).partial_cmp(&rank(b)).expect("finite scores"))
        .expect("a candidate");
    Pick {
        index,
        depth: root.max_depth(),
        cut_off: false,
        spent: 0,
        exact: true,
    }
}

/// Index into `candidates` of the move the search prefers from `root`:
/// iterative deepening as far as the game can last, keeping the pick of
/// the last depth that finished. Depth 1 always finishes.
pub fn best_candidate<C: Clock>(
    root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
    nimbers: &mut Nimbers,
) -> Pick {
    let max_depth = root.max_depth();
    best_candidate_to(root, candidates, budget, nimbers, max_depth)
}

/// Each iteration tries the candidates in the order the last one ranked
/// them; ties at the root go to the lower index.
pub fn best_candidate_to<C: Clock>(
    mut root: AbstractState,
    candidates: &[Move],
    budget: &mut Budget<C>,
    nimbers: &mut Nimbers,
    max_depth: usize,
) -> Pick {
    let mut pick = Pick {
        index: 0,
        depth: 0,
        cut_off: false,
        spent: 0,
        exact: false,
    };
    if candidates.is_empty() {
        return pick;
    }
    let mut search = Search::new(budget, nimbers);
    let mut order: Vec<usize> = (0..candidates.len()).collect();
    let mut scores = vec![f64::NEG_INFINITY; candidates.len()];
    for depth in 1..=max_depth {
        search.budget.arm(depth > 1);
        if search.budget.exhausted() {
            pick.cut_off = true;
            break;
        }
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

/// Two positions with the same live form have the same game and the same
/// evaluation, whatever dead pieces lie elsewhere.
fn key_of(form: &[u32], is_ai_turn: bool) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    form.hash(&mut hasher);
    is_ai_turn.hash(&mut hasher);
    hasher.finish()
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

    /// Hashes the position's regions up to renaming, and whose turn it is.
    #[cfg(test)]
    pub fn hash_key(&self) -> u64 {
        key_of(&crate::face::live_form(&self.face, 0), self.is_ai_turn)
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

    /// One millisecond per searched position.
    struct NodeClock {
        positions: usize,
    }

    impl Clock for NodeClock {
        fn elapsed_ms(&mut self) -> f64 {
            self.positions as f64
        }

        fn tick(&mut self) {
            self.positions += 1;
        }
    }

    fn unlimited() -> Budget<Frozen> {
        Budget::new(Frozen, f64::INFINITY)
    }

    fn nodes(limit: usize) -> Budget<NodeClock> {
        Budget::new(NodeClock { positions: 0 }, limit as f64)
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
        let mut nimbers = Nimbers::new();
        let mut search = Search::new(&mut budget, &mut nimbers);
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

    /// A table that files a cut-off score as exact answers a later probe
    /// through a wider window with a bound; one with bounds matches plain
    /// minimax on every position where that table does not.
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
        assert!(disagreements > 0);
    }

    #[test]
    fn the_table_move_and_the_killer_lead_the_ordering() {
        let state = fresh(5);
        let key = state.hash_key();
        let moves = state.generate_moves();
        assert!(moves.len() >= 3);
        let mut budget = unlimited();
        let mut nimbers = Nimbers::new();
        let mut search = Search::new(&mut budget, &mut nimbers);
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
        let pick = best_candidate(root, &candidates, &mut nodes(limit), &mut Nimbers::new());
        assert!(pick.cut_off);
        assert!(pick.depth >= 1);
        assert!(pick.spent >= limit, "{pick:?}");
        assert!(pick.spent < limit + CHECK_EVERY, "{pick:?}");
    }

    /// A depth-1 pass that alone outruns the budget is noticed as depth 2
    /// is armed, before it lists a position.
    #[test]
    fn a_budget_spent_by_depth_one_stops_before_depth_two_starts() {
        let root = after(24, 2);
        let candidates = root.generate_moves();
        let one = best_candidate_to(
            root.clone(),
            &candidates,
            &mut unlimited(),
            &mut Nimbers::new(),
            1,
        );
        let pick = best_candidate(root, &candidates, &mut nodes(1), &mut Nimbers::new());
        assert_eq!(pick.depth, 1);
        assert!(pick.cut_off);
        assert_eq!(pick.spent, one.spent);
        assert_eq!(pick.index, one.index);
    }

    #[test]
    fn a_cut_off_search_keeps_a_finished_depth_and_repeats_itself() {
        let root = fresh(6);
        let candidates: Vec<Move> = root.face.moves().into_iter().take(12).collect();
        let first = best_candidate(
            root.clone(),
            &candidates,
            &mut nodes(0),
            &mut Nimbers::new(),
        );
        let second = best_candidate(
            root.clone(),
            &candidates,
            &mut nodes(0),
            &mut Nimbers::new(),
        );
        assert_eq!(first, second);
        assert!(first.cut_off);
        assert!(first.depth >= 1);
        let same = best_candidate_to(
            root.clone(),
            &candidates,
            &mut unlimited(),
            &mut Nimbers::new(),
            first.depth,
        );
        assert_eq!(first.index, same.index);

        let small = fresh(2);
        let all = small.generate_moves();
        let roomy = best_candidate(small, &all, &mut unlimited(), &mut Nimbers::new());
        assert!(!roomy.cut_off);
        assert!(roomy.depth > 1);
    }

    #[test]
    fn the_depth_one_pick_is_the_best_by_evaluation() {
        let root = fresh(6);
        let candidates: Vec<Move> = root.face.moves().into_iter().take(12).collect();
        let one = best_candidate_to(
            root.clone(),
            &candidates,
            &mut unlimited(),
            &mut Nimbers::new(),
            1,
        );
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
        let two = best_candidate_to(
            root.clone(),
            &candidates,
            &mut unlimited(),
            &mut Nimbers::new(),
            2,
        );
        assert_eq!(two.depth, 2);
        let cut = best_candidate(
            root.clone(),
            &candidates,
            &mut nodes(two.spent + 1),
            &mut Nimbers::new(),
        );
        assert!(cut.cut_off);
        assert!(cut.depth >= 2);
        let same = best_candidate_to(
            root,
            &candidates,
            &mut unlimited(),
            &mut Nimbers::new(),
            cut.depth,
        );
        assert_eq!(cut.index, same.index);
    }

    #[test]
    fn depth_one_finishes_under_any_budget() {
        let root = fresh(6);
        let candidates = root.face.moves();
        let pick = best_candidate(root, &candidates, &mut nodes(0), &mut Nimbers::new());
        assert_eq!(pick.depth, 1);
        assert!(pick.cut_off);
        assert!(pick.spent > 0);
    }

    #[test]
    fn a_search_that_reaches_the_end_of_the_game_is_not_cut_off() {
        let root = fresh(2);
        let candidates = root.face.moves();
        let pick = best_candidate(
            root.clone(),
            &candidates,
            &mut nodes(1 << 20),
            &mut Nimbers::new(),
        );
        assert!(!pick.cut_off);
        assert_eq!(pick.depth, root.max_depth());
    }

    /// Always over, like a budget the depth-1 pass has already used up.
    struct Expired;

    impl Clock for Expired {
        fn elapsed_ms(&mut self) -> f64 {
            f64::INFINITY
        }
    }

    /// A search handed a budget that has already run out changes nothing
    /// and stores nothing; a later search on the same budget still gets
    /// its depth-1 pick.
    #[test]
    fn an_expired_budget_stops_the_search_before_it_changes_or_stores_anything() {
        let mut budget = Budget::new(Expired, SEARCH_BUDGET_MS);
        let mut nimbers = Nimbers::new();
        budget.arm(true);
        Search::new(&mut budget, &mut nimbers).minimax(
            &mut fresh(6),
            6,
            0,
            f64::NEG_INFINITY,
            f64::INFINITY,
            true,
        );
        assert!(budget.exhausted());
        let spent = budget.spent();

        let mut state = fresh(5);
        let before = state.clone();
        let mut search = Search::new(&mut budget, &mut nimbers);
        let score = search.minimax(&mut state, 4, 0, f64::NEG_INFINITY, f64::INFINITY, true);
        assert_eq!(score, 0.0);
        assert!(search.tt.table.is_empty());
        assert_eq!(state.face, before.face);
        assert_eq!(state.is_ai_turn, before.is_ai_turn);
        assert_eq!(budget.spent(), spent + 1);

        let pick = best_candidate(
            fresh(5),
            &fresh(5).generate_moves(),
            &mut budget,
            &mut nimbers,
        );
        assert_eq!(pick.depth, 1);
        assert!(pick.cut_off);
    }

    /// Prints what the search makes of the 5-node openings: each one's
    /// value searched to the end of the game, and the pick at the shipped
    /// budget. Run with `cargo test --release opening_values -- --ignored
    /// --nocapture`; about a minute.
    #[test]
    #[ignore = "about a minute in release"]
    fn five_node_opening_values() {
        let root = fresh(5);
        let candidates = root.generate_moves();
        let mut budget = unlimited();
        let mut nimbers = Nimbers::new();
        let mut search = Search::new(&mut budget, &mut nimbers);
        let mut probe = root.clone();
        let mut values = Vec::new();
        for mov in &candidates {
            probe.apply_move(mov);
            let depth = probe.max_depth();
            let value = search.minimax(
                &mut probe,
                depth,
                1,
                f64::NEG_INFINITY,
                f64::INFINITY,
                false,
            );
            probe.undo_move();
            values.push(value);
        }
        println!("openings {candidates:?}");
        println!(
            "values to the end of the game {values:?}, {} positions",
            budget.spent()
        );
        let mut boxed = Budget::new(WallClock::start(), SEARCH_BUDGET_MS);
        let pick = best_candidate(root, &candidates, &mut boxed, &mut Nimbers::new());
        println!(
            "at {SEARCH_BUDGET_MS} ms: opening {} at depth {}, worth {} to the end",
            pick.index, pick.depth, values[pick.index]
        );
    }

    /// Plays `n` nodes out, the AI choosing through `choose` over the
    /// model's move list and the opponent through `opponent`; whether the
    /// AI made the last move. Every AI choice must be exact.
    fn ai_wins(
        n: usize,
        ai_first: bool,
        nimbers: &mut Nimbers,
        work: usize,
        opponent: impl Fn(&AbstractState, &[Move]) -> usize,
    ) -> bool {
        let mut state = AbstractState::new(FaceModel::new(n), ai_first);
        loop {
            let candidates = state.generate_moves();
            if candidates.is_empty() {
                return !state.is_ai_turn;
            }
            let index = if state.is_ai_turn {
                let pick =
                    choose_within(state.clone(), &candidates, &mut unlimited(), nimbers, work);
                assert!(pick.exact, "{n} nodes, {} lives left", state.lives());
                pick.index
            } else {
                opponent(&state, &candidates)
            };
            state.apply_move(&candidates[index]);
        }
    }

    fn greedy(state: &AbstractState, candidates: &[Move]) -> usize {
        let mut probe = state.clone();
        let mut best = (f64::NEG_INFINITY, 0);
        for (idx, mov) in candidates.iter().enumerate() {
            probe.apply_move(mov);
            let score = -probe.evaluate();
            probe.undo_move();
            if score > best.0 {
                best = (score, idx);
            }
        }
        best.1
    }

    /// The AI in the seat the known result favours, with `work` child
    /// positions a turn, against the first listed move, the one-ply
    /// evaluation, and itself.
    fn exact_play_wins(n: usize, ai_first: bool, nimbers: &mut Nimbers, work: usize) {
        assert!(ai_wins(n, ai_first, nimbers, work, |_, _| 0), "{n} first");
        assert!(ai_wins(n, ai_first, nimbers, work, greedy), "{n} greedy");
        let mirror = |state: &AbstractState, candidates: &[Move]| {
            let mut flipped = state.clone();
            flipped.is_ai_turn = true;
            choose_within(
                flipped,
                candidates,
                &mut unlimited(),
                &mut Nimbers::new(),
                work,
            )
            .index
        };
        assert!(ai_wins(n, ai_first, nimbers, work, mirror), "{n} self");
    }

    /// The first player wins from 3 and 4 nodes, within the shipped
    /// budget from a cold table.
    #[test]
    fn exact_play_wins_the_three_and_four_node_games_as_first_player() {
        let mut nimbers = Nimbers::new();
        exact_play_wins(3, true, &mut nimbers, NIMBER_BUDGET);
        exact_play_wins(4, true, &mut nimbers, NIMBER_BUDGET);
    }

    /// The first player wins from 5 nodes and loses from 6. A fresh
    /// 5-node board needs more child positions than a turn gets, so the
    /// choice is given five million. Run with `cargo test --release
    /// exact_play_wins_the_five -- --ignored`, about 85 s.
    #[test]
    #[ignore = "solves five and six nodes, run in release"]
    fn exact_play_wins_the_five_and_six_node_games_from_the_winning_seat() {
        let mut nimbers = Nimbers::new();
        exact_play_wins(5, true, &mut nimbers, 5_000_000);
        exact_play_wins(6, false, &mut nimbers, 5_000_000);
    }

    /// Among winning moves the quick heuristic decides; a losing position
    /// keeps the move that leaves the opponent the most room.
    #[test]
    fn the_exact_pick_prefers_a_winning_move_by_the_heuristic() {
        let mut nimbers = Nimbers::new();
        let root = fresh(3);
        let candidates = root.generate_moves();
        let pick = choose(root.clone(), &candidates, &mut unlimited(), &mut nimbers);
        assert!(pick.exact);
        let mut probe = root.clone();
        let mut winning = Vec::new();
        for (idx, mov) in candidates.iter().enumerate() {
            probe.apply_move(mov);
            if nimbers.lost_in(&probe.face, &mut 100_000) == Some(true) {
                winning.push(idx);
            }
            probe.undo_move();
        }
        assert!(winning.contains(&pick.index));
        let best = winning
            .iter()
            .map(|&idx| root.quick_move_score(&candidates[idx]))
            .fold(f64::NEG_INFINITY, f64::max);
        assert_eq!(root.quick_move_score(&candidates[pick.index]), best);

        let lost = fresh(2);
        let candidates = lost.generate_moves();
        let pick = choose(lost.clone(), &candidates, &mut unlimited(), &mut nimbers);
        assert!(pick.exact);
        let mut probe = lost.clone();
        let rooms: Vec<usize> = candidates
            .iter()
            .map(|mov| {
                probe.apply_move(mov);
                let room = probe.mobility();
                probe.undo_move();
                room
            })
            .collect();
        assert_eq!(rooms[pick.index], *rooms.iter().max().unwrap());
    }

    /// With the budget spent the choice is the search's, which still
    /// scores a position exactly once its regions are in the table.
    #[test]
    fn a_solved_table_ends_the_search_at_the_first_solved_position() {
        let root = fresh(3);
        let candidates = root.generate_moves();
        let mut nimbers = Nimbers::new();
        nimbers.solve_all(&root.face, &mut 100_000).unwrap();
        let mut budget = unlimited();
        let pick = best_candidate(root.clone(), &candidates, &mut budget, &mut nimbers);
        assert!(!pick.exact);
        assert!(!pick.cut_off);
        assert!(pick.spent <= candidates.len() * root.max_depth());
        let mut probe = root;
        probe.apply_move(&candidates[pick.index]);
        assert_eq!(nimbers.lost_in(&probe.face, &mut 100_000), Some(true));
    }

    #[test]
    fn no_candidates_give_the_empty_pick() {
        let pick = best_candidate(fresh(2), &[], &mut unlimited(), &mut Nimbers::new());
        assert_eq!(pick.depth, 0);
        assert_eq!(pick.index, 0);
    }

    #[test]
    fn the_time_boxed_search_agrees_with_plain_minimax_at_depth_four() {
        for played in [0, 1, 2] {
            let root = after(4, played);
            let candidates = root.generate_moves();
            let boxed = best_candidate_to(
                root.clone(),
                &candidates,
                &mut unlimited(),
                &mut Nimbers::new(),
                4,
            );
            assert_eq!(boxed.depth, 4);
            assert_eq!(
                boxed.index,
                plain_best(root, &candidates, 4),
                "after {played}"
            );
        }
    }

    /// A third of the least a quiet machine searches in the budget with
    /// the live form as the key.
    #[cfg(not(debug_assertions))]
    const POSITIONS_FLOOR: usize = 12_000;

    /// Depth 1 cannot be cut, so the bound only measures the budget where
    /// depth 1 is well inside it, which a debug build is not.
    #[cfg(not(debug_assertions))]
    #[test]
    fn twenty_nodes_answer_within_the_budget_on_the_wall_clock() {
        let root = twenty_nodes_after_three_moves();
        let candidates = root.generate_moves();
        let mut times = Vec::new();
        let mut spent = Vec::new();
        for _ in 0..10 {
            let started = std::time::Instant::now();
            let mut budget = Budget::new(WallClock::start(), SEARCH_BUDGET_MS);
            let pick = best_candidate(root.clone(), &candidates, &mut budget, &mut Nimbers::new());
            let took = started.elapsed().as_secs_f64() * 1000.0;
            println!(
                "{took:.1} ms, depth {}, {} positions",
                pick.depth, pick.spent
            );
            times.push(took);
            spent.push(pick.spent);
        }
        for took in times {
            assert!(took <= SEARCH_BUDGET_MS * 1.1, "{took:.1} ms");
        }
        let slowest = spent.iter().min().copied().unwrap_or(0);
        assert!(
            slowest >= POSITIONS_FLOOR,
            "{slowest} positions in {SEARCH_BUDGET_MS} ms"
        );
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
