//! Random games drawn through the router, shared by the property test and
//! the bench's tally.

use std::collections::BTreeMap;

use super::{route, Unroutable};
use crate::face::Move as FaceMove;
use crate::types::{GameState, Move, Player, Point};
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

/// Whether a route failed only because the position left no room.
pub(crate) fn cramped(why: &Unroutable) -> bool {
    matches!(why, Unroutable::Room)
}

/// Whether a failure may leave a position with nothing to draw: no room,
/// or a line the relaxation could not get clear of a node. The latter
/// still counts toward the failure share.
pub(crate) fn walled_in(why: &Unroutable) -> bool {
    use crate::validation::{Rejection, RejectionKind};
    cramped(why)
        || matches!(
            why,
            Unroutable::Rejected(Rejection {
                kind: RejectionKind::PathTooCloseToNode,
                ..
            })
        )
}

pub(crate) fn kind_name(why: &Unroutable) -> String {
    match why {
        Unroutable::Rejected(r) => format!("{:?}", r.kind),
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

/// Why a listed move failed, with the geometry around the offending node
/// when the validator rejected the line for passing one too closely: how
/// near the taut and the relaxed path came, and the room beyond the path.
pub(crate) fn diagnose(state: &GameState, mv: &FaceMove, why: &Unroutable) -> String {
    use super::{clearance, corridor, rules, Arena, Rules, NODE_ROOM, SUBDIVIDE};
    use crate::geometry::{closest_point_on_segment, resample_polyline};
    use crate::validation::{BoardLocation, Rejection, RejectionKind};
    let head = format!("{mv:?} -> {}", kind_name(why));
    let Unroutable::Rejected(Rejection {
        kind: RejectionKind::PathTooCloseToNode,
        at: BoardLocation::Node { id },
        ..
    }) = why
    else {
        return head;
    };
    let Rules {
        clearance: c,
        node_clearance,
        spacing,
        ..
    } = rules(state);
    let room = (NODE_ROOM * spacing).max(node_clearance + c);
    let need = corridor::Clearances {
        line: c,
        node: node_clearance,
        room,
    };
    let Ok(face) = state.face.corner_face(mv.a, mv.at_a) else {
        return head;
    };
    let Ok(arena) = Arena::build(state, face, SUBDIVIDE * c) else {
        return head;
    };
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    let nodes: Vec<Point> = state
        .nodes
        .iter()
        .filter(|n| n.id != mv.a && n.id != mv.b)
        .map(|n| n.position)
        .collect();
    let Ok(threaded) = corridor::shortest(state, &arena, mv, &nodes, need, false) else {
        return format!("{head}; no corridor");
    };
    let obstacles = clearance::Obstacles::new(
        arena.segments(),
        nodes.clone(),
        ends,
        c,
        node_clearance,
        room,
        spacing,
    );
    let pushed = clearance::push(threaded.clone(), &obstacles);
    let path = pushed.clone().unwrap_or_else(|_| threaded.clone());
    let target = state.nodes[*id].position;
    let nearest = |path: &[Point]| {
        resample_polyline(path, c / 2.0)
            .into_iter()
            .map(|q| (q, q.distance_to(&target)))
            .min_by(|x, y| x.1.total_cmp(&y.1))
            .unwrap()
    };
    let (q, d) = nearest(&path);
    let (_, dt) = nearest(&threaded);
    let away = (q.x - target.x, q.y - target.y);
    let beyond = |w: &Point| (w.x - q.x) * away.0 + (w.y - q.y) * away.1 > 0.0;
    let wall = arena
        .segments()
        .iter()
        .map(|(a, b)| closest_point_on_segment(a, b, &q))
        .filter(|(w, _)| beyond(w))
        .map(|(_, dw)| dw)
        .fold(f64::INFINITY, f64::min);
    let other = nodes
        .iter()
        .filter(|n| **n != target && beyond(n))
        .map(|n| n.distance_to(&q))
        .fold(f64::INFINITY, f64::min);
    format!(
        "{head}: node {id} at {d:.1} after the push ({dt:.1} taut, push {}), needs {node_clearance:.1}; \
         beyond the path a wall at {wall:.1} and a node at {other:.1}; {:.0} and {:.0} from the ends",
        if pushed.is_ok() { "ok" } else { "failed" },
        ends[0].distance_to(&q),
        ends[1].distance_to(&q)
    )
}

/// Every forfeit over `games` seeded games, each with its diagnoses, so the
/// remaining router failures can be read in one pass; each position is
/// also written to `$TMPDIR/forfeit_g<game>.json`.
#[cfg(test)]
pub fn survey(seed: u64, games: usize) -> String {
    let mut x = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) | 1;
    let mut next = move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    let mut out = Vec::new();
    for g in 0..games {
        let nodes = 4 + (next() % 9) as usize;
        let picks: Vec<u32> = (0..36).map(|_| next() as u32).collect();
        if let Err(e) = game(nodes, &picks) {
            let (summary, fixture) = e.split_once("\nin\n").unwrap_or((&e, ""));
            let path = std::env::temp_dir().join(format!("forfeit_g{g}.json"));
            std::fs::write(&path, fixture).ok();
            out.push(format!("game {g} ({nodes} nodes): {summary}"));
        }
    }
    format!(
        "{} forfeits in {games} games\n{}",
        out.len(),
        out.join("\n")
    )
}

/// Routes tried, those that failed for a reason other than room, and every
/// failure by kind.
#[derive(Debug, Default, Clone)]
pub struct Tally {
    pub tried: usize,
    pub failed: usize,
    pub kinds: BTreeMap<String, usize>,
}

impl Tally {
    pub fn add(&mut self, other: &Tally) {
        self.tried += other.tried;
        self.failed += other.failed;
        for (kind, n) in &other.kinds {
            *self.kinds.entry(kind.clone()).or_default() += n;
        }
    }
}

/// One random game on `nodes` nodes: each pick names a search move to try
/// first, the rest follow in order until one draws. Every drawn move has to
/// pass the AI validator, and a position where nothing draws has to have
/// failed on room alone; otherwise the position comes back as a fixture
/// with the reasons.
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
        let mut reasons = Vec::new();
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
                    if !cramped(&why) {
                        tally.failed += 1;
                    }
                    *tally.kinds.entry(kind_name(&why)).or_default() += 1;
                    reasons.push(why);
                }
            }
        }
        let Some(mov) = drawn else {
            if reasons.iter().all(walled_in) {
                break;
            }
            let diagnoses: Vec<String> = moves
                .iter()
                .zip(&reasons)
                .filter(|(_, why)| !walled_in(why))
                .map(|(mv, why)| diagnose(&state, mv, why))
                .collect();
            return Err(format!(
                "no listed move routes, {reasons:?}\n{}\nin\n{}",
                diagnoses.join("\n"),
                fixture_json(&state)
            ));
        };
        state.apply_move(mov).unwrap();
        state.current_player = Player::AI;
    }
    Ok(tally)
}

/// `games` games of 4 to 12 nodes and up to 36 picks each from a fixed
/// seed, tallied together; a game that fails is counted as tried once.
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
                total.failed += 1;
                *total.kinds.entry("forfeit".into()).or_default() += 1;
            }
        }
    }
    total
}
