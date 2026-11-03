use std::cell::RefCell;

use crate::abstract_graph::{self, AbstractState, Budget, Clock, WallClock, SEARCH_BUDGET_MS};
use crate::face::{Component, Move as FaceMove};
use crate::nimber::Nimbers;
use crate::route;
use crate::types::{GameState, Move, Player};

/// The AI's answer to a position.
#[derive(Debug, Clone)]
pub enum Reply {
    Move(Move),
    /// The model lists moves but none of them could be drawn; the game is
    /// still open.
    Undrawn,
    GameOver,
}

/// Class members tried for a listed move that cannot be drawn, before the
/// next ranked move is.
const CLASS_TRIES: usize = 6;
/// Wall clock for drawing, after the search has had its own.
const ROUTE_BUDGET_MS: f64 = 800.0;

/// Lists the search moves, ranks them once on the model, and draws the
/// best one that routes, walking the ranking on failure.
pub fn choose(state: &GameState) -> Reply {
    let candidates = state.face.search_moves();
    if candidates.is_empty() {
        return Reply::GameOver;
    }
    let order = rank(state, &candidates);
    match draw(state, &candidates, &order) {
        (Some(mov), _) => Reply::Move(mov),
        (None, _) => Reply::Undrawn,
    }
}

thread_local! {
    /// Settled regions, kept across turns and games.
    static NIMBERS: RefCell<Nimbers> = RefCell::new(Nimbers::new());
}

/// Indices into `candidates`, best first, from one search.
pub(crate) fn rank(state: &GameState, candidates: &[FaceMove]) -> Vec<usize> {
    if candidates.len() < 2 {
        return (0..candidates.len()).collect();
    }
    let root = AbstractState::new(state.face.clone(), state.current_player == Player::AI);
    let mut budget = Budget::new(WallClock::start(), SEARCH_BUDGET_MS);
    NIMBERS.with(|nimbers| {
        abstract_graph::choose(root, candidates, &mut budget, &mut nimbers.borrow_mut()).order
    })
}

/// Routes the candidates in ranked order until one draws, trying the other
/// members of a move's class before moving down the ranking. Once
/// `ROUTE_BUDGET_MS` has passed, every remaining candidate still gets one
/// attempt at the cheap clearance, so Undrawn means every listed move was
/// tried. Returns the drawn move and the number of ranked moves passed
/// over.
pub(crate) fn draw(
    state: &GameState,
    candidates: &[FaceMove],
    order: &[usize],
) -> (Option<Move>, usize) {
    let mut clock = WallClock::start();
    let mut failures = 0;
    for &idx in order {
        let listed = &candidates[idx];
        if clock.elapsed_ms() > ROUTE_BUDGET_MS {
            if let Ok(mov) = route::route_cheap(state, listed) {
                return (Some(mov), failures);
            }
            failures += 1;
            continue;
        }
        let tries = std::iter::once(listed.clone()).chain(class_members(state, listed));
        for attempt in tries {
            if let Ok(mov) = route::route(state, &attempt) {
                return (Some(mov), failures);
            }
            if clock.elapsed_ms() > ROUTE_BUDGET_MS {
                break;
            }
        }
        failures += 1;
    }
    (None, failures)
}

/// Other moves reaching the same position as `rep` up to relabelling the
/// isolated nodes of its face: the same move from another isolated node,
/// or enclosing other isolated nodes. At most `CLASS_TRIES`, in the order
/// the face lists its pieces.
pub(crate) fn class_members(state: &GameState, rep: &FaceMove) -> Vec<FaceMove> {
    class_members_where(state, rep, CLASS_TRIES, |_| true)
}

/// The members of `rep`'s class that `keep` accepts, at most `limit`.
pub(crate) fn class_members_where(
    state: &GameState,
    rep: &FaceMove,
    limit: usize,
    keep: impl Fn(&FaceMove) -> bool,
) -> Vec<FaceMove> {
    let model = &state.face;
    let Ok(face) = model.corner_face(rep.a, rep.at_a) else {
        return Vec::new();
    };
    let isolated: Vec<usize> = model
        .components(face)
        .iter()
        .filter_map(|c| match c {
            Component::Node(v) => Some(*v),
            Component::Cycle(_) => None,
        })
        .collect();
    let bare = |v: usize| model.degree(v) == 0;
    let ends_a: Vec<usize> = if bare(rep.a) {
        isolated.clone()
    } else {
        vec![rep.a]
    };
    let cycles: Vec<usize> = rep.enclosed.iter().copied().filter(|v| !bare(*v)).collect();
    let wanted = rep.enclosed.len() - cycles.len();
    let mut out = Vec::new();
    for &a in &ends_a {
        let ends_b: Vec<usize> = if rep.a == rep.b {
            vec![a]
        } else if bare(rep.b) {
            isolated.iter().copied().filter(|&v| v != a).collect()
        } else {
            vec![rep.b]
        };
        for &b in &ends_b {
            // The lowest other piece of a bare loop is held outside, as in
            // the enumeration, so only the rest are enclosable.
            let mut pool: Vec<usize> = isolated
                .iter()
                .copied()
                .filter(|&v| v != a && v != b)
                .collect();
            if a == b && bare(a) {
                let first_other = model
                    .components(face)
                    .iter()
                    .map(|&c| model.lowest_node(c))
                    .find(|&v| v != a);
                pool.retain(|&v| Some(v) != first_other);
                if first_other.is_some_and(|v| cycles.contains(&v)) {
                    continue;
                }
            }
            if pool.len() < wanted {
                continue;
            }
            for start in 0..=pool.len() - wanted {
                let mut enclosed = cycles.clone();
                enclosed.extend_from_slice(&pool[start..start + wanted]);
                enclosed.sort_unstable();
                let member = FaceMove {
                    a,
                    at_a: if bare(a) { None } else { rep.at_a },
                    b,
                    at_b: if bare(b) { None } else { rep.at_b },
                    enclosed,
                };
                if member != *rep && keep(&member) && !out.contains(&member) {
                    out.push(member);
                    if out.len() == limit {
                        return out;
                    }
                }
                if wanted == 0 {
                    break;
                }
            }
        }
    }
    out
}

/// Size of the worker's nimber table, its hit and miss counts, and the
/// lives and node count of the largest region it settled.
#[cfg(feature = "bench")]
pub fn nimber_stats() -> (usize, usize, usize, (usize, usize)) {
    NIMBERS.with(|nimbers| {
        let n = nimbers.borrow();
        (n.len(), n.hits(), n.misses(), n.largest())
    })
}

/// Positions charged when deepening to `depth` with no time limit.
#[cfg(feature = "bench")]
pub(crate) fn nodes_to_depth(state: &GameState, candidates: &[FaceMove], depth: usize) -> usize {
    use crate::abstract_graph::{best_candidate_to, Clock};
    struct Still;
    impl Clock for Still {
        fn elapsed_ms(&mut self) -> f64 {
            0.0
        }
    }
    let root = AbstractState::new(state.face.clone(), state.current_player == Player::AI);
    let mut budget = Budget::new(Still, f64::INFINITY);
    best_candidate_to(root, candidates, &mut budget, &mut Nimbers::new(), depth).spent
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

pub fn select_ai_move(state: &GameState) -> Reply {
    choose(state)
}
