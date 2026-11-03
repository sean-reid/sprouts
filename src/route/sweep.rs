//! Random games drawn through the router, shared by the property test and
//! the bench's tally.

use std::collections::BTreeMap;

use super::{route, Unroutable};
use crate::face::Move as FaceMove;
use crate::types::{GameState, Move, Player};
use crate::validation::validate_ai_move;

/// Routes a search move the way the AI does: the move itself, else the
/// other members of its class, reporting the last failure.
pub(crate) fn draws(state: &GameState, mv: &FaceMove) -> Result<Move, Unroutable> {
    let mut last = route(state, mv);
    if last.is_ok() {
        return last;
    }
    for member in crate::ai::class_members(state, mv) {
        last = route(state, &member);
        if last.is_ok() {
            break;
        }
    }
    last
}

pub(crate) fn kind_name(why: &Unroutable) -> String {
    match why {
        Unroutable::Bug(what) => format!("Bug({what})"),
        other => format!("{other:?}"),
    }
}

/// The game so far in the bench fixture shape, for pasting under
/// fixtures/routing when a route fails.
pub(crate) fn fixture_json(state: &GameState) -> String {
    let moves: Vec<serde_json::Value> = state
        .move_history
        .iter()
        .map(|m| {
            serde_json::json!({
                "from": m.from_node,
                "to": m.to_node,
                "polyline": m.polyline.iter().map(|q| [q.x, q.y]).collect::<Vec<_>>(),
                "newNode": [m.new_node_pos.x, m.new_node_pos.y],
            })
        })
        .collect();
    let nodes: Vec<[f64; 2]> = state.nodes[..state.initial_node_count]
        .iter()
        .map(|n| [n.position.x, n.position.y])
        .collect();
    serde_json::json!({
        "version": 1,
        "initialNodes": state.initial_node_count,
        "boardSize": state.board_size,
        "nodes": nodes,
        "moves": moves,
    })
    .to_string()
}

/// Routes tried and how each failure went: no room, a read-back mismatch,
/// or a router bug, with every bug by name.
#[derive(Debug, Default, Clone)]
pub struct Tally {
    pub tried: usize,
    pub room: usize,
    pub topology: usize,
    pub bug: usize,
    pub bugs: BTreeMap<String, usize>,
}

impl Tally {
    pub fn add(&mut self, other: &Tally) {
        self.tried += other.tried;
        self.room += other.room;
        self.topology += other.topology;
        self.bug += other.bug;
        for (kind, n) in &other.bugs {
            *self.bugs.entry(kind.clone()).or_default() += n;
        }
    }

    fn count(&mut self, why: &Unroutable) {
        match why {
            Unroutable::Room => self.room += 1,
            Unroutable::Topology => self.topology += 1,
            Unroutable::Bug(_) => {
                self.bug += 1;
                *self.bugs.entry(kind_name(why)).or_default() += 1;
            }
        }
    }
}

impl std::fmt::Display for Tally {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} routes: {} no room, {} topology, {} bugs {:?}",
            self.tried, self.room, self.topology, self.bug, self.bugs
        )
    }
}

/// One random game on `nodes` nodes: each pick names a search move to try
/// first, the rest follow in order until one draws. Every drawn move has to
/// pass the AI validator and no route may fail as a bug; otherwise the
/// position comes back as a fixture with the reasons.
pub fn game(nodes: usize, picks: &[u32]) -> Result<Tally, String> {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::AI;
    let mut tally = Tally::default();
    for &pick in picks {
        let moves = state.face.search_moves();
        if moves.is_empty() {
            break;
        }
        let first = pick as usize % moves.len();
        let mut drawn = None;
        for k in (0..moves.len()).map(|k| (first + k) % moves.len()) {
            tally.tried += 1;
            match draws(&state, &moves[k]) {
                Ok(mov) => {
                    if let Err(e) = validate_ai_move(&state, &mov) {
                        return Err(format!(
                            "a drawn move was rejected: {:?} in\n{}",
                            e.kind,
                            fixture_json(&state)
                        ));
                    }
                    drawn = Some(mov);
                    break;
                }
                Err(why) => {
                    if let Unroutable::Bug(what) = why {
                        return Err(format!(
                            "{:?} failed as a bug: {what} in\n{}",
                            moves[k],
                            fixture_json(&state)
                        ));
                    }
                    tally.count(&why);
                }
            }
        }
        let Some(mov) = drawn else {
            break;
        };
        state.apply_move(mov).unwrap();
        state.current_player = Player::AI;
    }
    Ok(tally)
}

/// `games` games of 4 to 12 nodes and up to 36 picks each from a fixed
/// seed, tallied together; a game that fails is counted as one bug.
pub fn sweep(seed: u64, games: usize) -> Tally {
    let mut x = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1;
    let mut next = move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    let mut total = Tally::default();
    for _ in 0..games {
        let nodes = 4 + (next() % 9) as usize;
        let picks: Vec<u32> = (0..36).map(|_| next() as u32).collect();
        match game(nodes, &picks) {
            Ok(tally) => total.add(&tally),
            Err(_) => {
                total.tried += 1;
                total.bug += 1;
                *total.bugs.entry("forfeit".into()).or_default() += 1;
            }
        }
    }
    total
}
