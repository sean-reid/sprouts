//! Topology-aware abstract game state for minimax search.
//!
//! Unlike a pure combinatorial model, this tracks an adjacency graph of which
//! nodes can physically reach each other on the board.  At the root the
//! adjacency is seeded from the real spatial classifier; during simulation
//! moves update it conservatively so the search doesn't hallucinate moves
//! that are geometrically impossible.

use std::collections::{HashMap, HashSet};
use std::hash::{Hash, Hasher};

#[derive(Clone)]
pub struct AbstractState {
    pub nodes: Vec<(usize, u8)>,        // (node_id, connection_count)
    pub is_ai_turn: bool,
    pub next_node_id: usize,
    /// Set of node pairs (a, b) where a < b that can reach each other.
    /// Self-loop candidates are stored as (id, id).
    adjacency: HashSet<(usize, usize)>,
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

fn pair_key(a: usize, b: usize) -> (usize, usize) {
    if a <= b { (a, b) } else { (b, a) }
}

impl AbstractState {
    /// Build from real game state + classifier results.
    pub fn from_game_state_with_classification(
        state: &crate::types::GameState,
        legal_pairs: &[(usize, usize)],
        self_loop_candidates: &[usize],
    ) -> Self {
        let nodes = state
            .nodes
            .iter()
            .map(|n| (n.id, n.connection_count))
            .collect();
        let is_ai_turn = state.current_player == crate::types::Player::AI;

        let mut adjacency = HashSet::new();
        for &(a, b) in legal_pairs {
            adjacency.insert(pair_key(a, b));
        }
        for &id in self_loop_candidates {
            adjacency.insert((id, id));
        }

        Self {
            nodes,
            is_ai_turn,
            next_node_id: state.next_node_id,
            adjacency,
        }
    }

    /// Build from game state without classification (assumes all active pairs
    /// are reachable). Used by tests; the AI uses from_game_state_with_classification.
    #[cfg(test)]
    pub fn from_game_state(state: &crate::types::GameState) -> Self {
        let nodes: Vec<(usize, u8)> = state
            .nodes
            .iter()
            .map(|n| (n.id, n.connection_count))
            .collect();
        let is_ai_turn = state.current_player == crate::types::Player::AI;

        let active: Vec<usize> = nodes
            .iter()
            .filter(|(_, cc)| *cc < 3)
            .map(|(id, _)| *id)
            .collect();

        let mut adjacency = HashSet::new();
        for i in 0..active.len() {
            for j in (i + 1)..active.len() {
                adjacency.insert(pair_key(active[i], active[j]));
            }
        }
        for &(id, cc) in &nodes {
            if cc <= 1 {
                adjacency.insert((id, id));
            }
        }

        Self {
            nodes,
            is_ai_turn,
            next_node_id: state.next_node_id,
            adjacency,
        }
    }

    pub fn generate_moves(&self) -> Vec<AbstractMove> {
        let active: HashSet<usize> = self
            .nodes
            .iter()
            .filter(|(_, cc)| *cc < 3)
            .map(|(id, _)| *id)
            .collect();

        let mut moves = Vec::new();

        for &(a, b) in &self.adjacency {
            if a == b {
                // Self-loop: need cc <= 1
                if let Some(&(_, cc)) = self.nodes.iter().find(|(id, _)| *id == a) {
                    if cc <= 1 && active.contains(&a) {
                        moves.push(AbstractMove { from: a, to: b });
                    }
                }
            } else {
                // Pair: both must be active
                if active.contains(&a) && active.contains(&b) {
                    moves.push(AbstractMove { from: a, to: b });
                }
            }
        }

        moves
    }

    pub fn apply_move(&mut self, mov: &AbstractMove) {
        // Update connection counts
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

        // Add new node with 2 connections
        let new_id = self.next_node_id;
        self.nodes.push((new_id, 2));
        self.next_node_id += 1;

        // Update adjacency: the new node is placed on the line between
        // from and to, so it inherits their connectivity.
        //
        // The new node can reach both endpoints:
        self.adjacency.insert(pair_key(new_id, mov.from));
        if mov.from != mov.to {
            self.adjacency.insert(pair_key(new_id, mov.to));
        }

        // The new node can also reach any node that BOTH endpoints can reach
        // (conservative: only nodes reachable from both sides of the new line).
        // Additionally, nodes reachable from just one endpoint remain reachable
        // from that endpoint (the line doesn't necessarily block those paths).
        let neighbors_from: HashSet<usize> = self.neighbors_of(mov.from);
        let neighbors_to: HashSet<usize> = if mov.from == mov.to {
            neighbors_from.clone()
        } else {
            self.neighbors_of(mov.to)
        };

        // New node inherits adjacency to nodes reachable from either endpoint
        for &n in neighbors_from.union(&neighbors_to) {
            if n != new_id && n != mov.from && n != mov.to {
                self.adjacency.insert(pair_key(new_id, n));
            }
        }

        // Self-loop eligibility for new node: it has cc=2, so no self-loop.
        // (Self-loops require cc <= 1.)

        // Remove the used pair from adjacency (the line now occupies that path).
        // The nodes can still reach each other through the new node, but the
        // direct connection is consumed.  We keep the pair if both nodes still
        // have capacity, since in Sprouts multiple lines can connect the same
        // pair through different spatial routes.
        // However, remove self-loop eligibility for nodes that used up capacity:
        if mov.from == mov.to {
            // Self-loop used: node now has cc >= 2, remove self-loop entry
            self.adjacency.remove(&(mov.from, mov.from));
        }

        // Remove adjacency entries involving dead nodes (cc >= 3)
        let dead: HashSet<usize> = self
            .nodes
            .iter()
            .filter(|(_, cc)| *cc >= 3)
            .map(|(id, _)| *id)
            .collect();

        if !dead.is_empty() {
            self.adjacency.retain(|&(a, b)| !dead.contains(&a) && !dead.contains(&b));
        }

        // Remove self-loop entries for nodes with cc > 1
        let no_selfloop: Vec<usize> = self
            .nodes
            .iter()
            .filter(|(_, cc)| *cc > 1)
            .map(|(id, _)| *id)
            .collect();
        for id in no_selfloop {
            self.adjacency.remove(&(id, id));
        }

        self.is_ai_turn = !self.is_ai_turn;
    }

    fn neighbors_of(&self, node_id: usize) -> HashSet<usize> {
        let mut result = HashSet::new();
        for &(a, b) in &self.adjacency {
            if a == node_id && b != node_id {
                result.insert(b);
            } else if b == node_id && a != node_id {
                result.insert(a);
            }
        }
        result
    }

    pub fn hash_key(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        let mut node_data: Vec<(usize, u8)> = self.nodes.iter().copied().collect();
        node_data.sort_unstable();
        node_data.hash(&mut hasher);
        self.is_ai_turn.hash(&mut hasher);
        // Include adjacency in hash so different topologies don't collide
        let mut adj_sorted: Vec<(usize, usize)> = self.adjacency.iter().copied().collect();
        adj_sorted.sort_unstable();
        adj_sorted.hash(&mut hasher);
        hasher.finish()
    }

    pub fn quick_move_score(&self, mov: &AbstractMove) -> f64 {
        let mut score = 0.0;

        if let Some(&(_, cc)) = self.nodes.iter().find(|(id, _)| *id == mov.from) {
            if cc == 2 {
                score += 30.0;
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
        let constrained = self.nodes.iter().filter(|(_, cc)| *cc == 2).count();
        let active = self.nodes.iter().filter(|(_, cc)| *cc < 3).count();
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
            let mut total_opp_moves = 0usize;
            for mov in &moves {
                let mut sim = self.clone();
                sim.apply_move(mov);
                total_opp_moves += sim.generate_moves().len();
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

#[cfg(test)]
mod tests {
    use super::*;

    fn state_2nodes() -> AbstractState {
        AbstractState {
            nodes: vec![(0, 0), (1, 0)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::from([(0, 1), (0, 0), (1, 1)]),
        }
    }

    #[test]
    fn generate_moves_2_fresh_nodes() {
        let s = state_2nodes();
        let moves = s.generate_moves();
        assert_eq!(moves.len(), 3);
    }

    #[test]
    fn generate_moves_no_self_loop_when_2_connections() {
        let s = AbstractState {
            nodes: vec![(0, 2), (1, 0)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::from([(0, 1), (1, 1)]),
        };
        let moves = s.generate_moves();
        assert_eq!(moves.len(), 2);
    }

    #[test]
    fn generate_moves_dead_node_excluded() {
        let s = AbstractState {
            nodes: vec![(0, 3), (1, 0)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::from([(1, 1)]),
        };
        let moves = s.generate_moves();
        assert_eq!(moves.len(), 1);
        assert_eq!(moves[0].from, 1);
        assert_eq!(moves[0].to, 1);
    }

    #[test]
    fn generate_moves_respects_adjacency() {
        // Two active nodes but NOT adjacent — no pair move
        let s = AbstractState {
            nodes: vec![(0, 0), (1, 0)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::from([(0, 0), (1, 1)]), // only self-loops
        };
        let moves = s.generate_moves();
        assert_eq!(moves.len(), 2); // only self-loops, no pair
        assert!(moves.iter().all(|m| m.from == m.to));
    }

    #[test]
    fn apply_move_pair_updates_connections() {
        let mut s = state_2nodes();
        s.apply_move(&AbstractMove { from: 0, to: 1 });
        let n0 = s.nodes.iter().find(|(id, _)| *id == 0).unwrap().1;
        let n1 = s.nodes.iter().find(|(id, _)| *id == 1).unwrap().1;
        assert_eq!(n0, 1);
        assert_eq!(n1, 1);
    }

    #[test]
    fn apply_move_adds_new_node() {
        let mut s = state_2nodes();
        s.apply_move(&AbstractMove { from: 0, to: 1 });
        assert_eq!(s.nodes.len(), 3);
        let new_node = s.nodes.iter().find(|(id, _)| *id == 2).unwrap();
        assert_eq!(new_node.1, 2);
    }

    #[test]
    fn apply_move_new_node_is_adjacent_to_endpoints() {
        let mut s = state_2nodes();
        s.apply_move(&AbstractMove { from: 0, to: 1 });
        assert!(s.adjacency.contains(&(0, 2)));
        assert!(s.adjacency.contains(&(1, 2)));
    }

    #[test]
    fn apply_move_switches_turn() {
        let mut s = state_2nodes();
        assert!(s.is_ai_turn);
        s.apply_move(&AbstractMove { from: 0, to: 1 });
        assert!(!s.is_ai_turn);
    }

    #[test]
    fn apply_self_loop() {
        let mut s = state_2nodes();
        s.apply_move(&AbstractMove { from: 0, to: 0 });
        let n0 = s.nodes.iter().find(|(id, _)| *id == 0).unwrap().1;
        assert_eq!(n0, 2);
    }

    #[test]
    fn evaluate_terminal_ai_turn_loses() {
        let s = AbstractState {
            nodes: vec![(0, 3), (1, 3)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::new(),
        };
        let score = s.evaluate();
        assert!(score < 0.0);
    }

    #[test]
    fn evaluate_terminal_human_turn_ai_wins() {
        let s = AbstractState {
            nodes: vec![(0, 3), (1, 3)],
            is_ai_turn: false,
            next_node_id: 2,
            adjacency: HashSet::new(),
        };
        let score = s.evaluate();
        assert!(score > 0.0);
    }

    #[test]
    fn same_state_same_hash() {
        let s1 = state_2nodes();
        let s2 = state_2nodes();
        assert_eq!(s1.hash_key(), s2.hash_key());
    }

    #[test]
    fn different_states_different_hash() {
        let s1 = state_2nodes();
        let s2 = AbstractState {
            nodes: vec![(0, 1), (1, 0)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::from([(0, 1)]),
        };
        assert_ne!(s1.hash_key(), s2.hash_key());
    }

    #[test]
    fn different_turn_different_hash() {
        let mut s1 = state_2nodes();
        s1.is_ai_turn = true;
        let mut s2 = state_2nodes();
        s2.is_ai_turn = false;
        assert_ne!(s1.hash_key(), s2.hash_key());
    }

    #[test]
    fn minimax_2_node_game() {
        let s = state_2nodes();
        let mut tt = TranspositionTable::new();
        let depth = s.choose_depth();
        let score = abstract_minimax(
            &s,
            depth,
            f64::NEG_INFINITY,
            f64::INFINITY,
            s.is_ai_turn,
            &mut tt,
        );
        assert!(score.is_finite());
    }

    #[test]
    fn minimax_terminal_state() {
        let s = AbstractState {
            nodes: vec![(0, 3), (1, 3)],
            is_ai_turn: true,
            next_node_id: 2,
            adjacency: HashSet::new(),
        };
        let mut tt = TranspositionTable::new();
        let score = abstract_minimax(&s, 5, f64::NEG_INFINITY, f64::INFINITY, true, &mut tt);
        assert!(score < 0.0);
    }

    #[test]
    fn disconnected_nodes_have_fewer_moves() {
        // All connected
        let s1 = AbstractState {
            nodes: vec![(0, 0), (1, 0), (2, 0)],
            is_ai_turn: true,
            next_node_id: 3,
            adjacency: HashSet::from([(0, 1), (0, 2), (1, 2), (0, 0), (1, 1), (2, 2)]),
        };
        // Node 2 disconnected from 0 and 1
        let s2 = AbstractState {
            nodes: vec![(0, 0), (1, 0), (2, 0)],
            is_ai_turn: true,
            next_node_id: 3,
            adjacency: HashSet::from([(0, 1), (0, 0), (1, 1), (2, 2)]),
        };
        assert!(s1.generate_moves().len() > s2.generate_moves().len());
    }
}
