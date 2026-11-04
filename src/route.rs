//! Draws a face-model move as a curve inside its face.
//!
//! The face is eroded first: every wall moves inward by the line clearance
//! and every other node is cut out to the node clearance, and the move is
//! routed through what is left (src/route/erode.rs). A path through the
//! eroded face keeps the clearances by construction, so the two ends being
//! disconnected there is exactly "no room". The corridor search picks the
//! homotopy class the move names, the funnel pulls the path taut, an
//! optional relaxation moves it off the walls where the room allows, and
//! the spline is fit and read back through the embedding.

use crate::ai::rank_node_placements;
use crate::curve;
use crate::embedding;
#[cfg(any(test, feature = "hash"))]
use crate::exact::Fnv;
use crate::face::{Component, Move as FaceMove, NodeId};
use crate::geometry::resample_polyline;
use crate::types::{
    ai_node_clearance, node_disc, scale, GameState, Move, Player, Point, AI_INTERSECTION_TOLERANCE,
    HUMAN_INTERSECTION_TOLERANCE, MIN_NODE_SPACING, MIN_NODE_SPACING_AI, MIN_PATH_CLEARANCE,
    MIN_PATH_CLEARANCE_AI,
};
use crate::validation::{validate_ai_move, validate_move};

mod clearance;
mod corridor;
mod erode;
mod small_loop;
#[cfg(any(test, feature = "bench"))]
pub mod sweep;
#[cfg(test)]
mod tests;

/// Why a legal move could not be drawn.
#[derive(Debug, Clone, PartialEq)]
pub enum Unroutable {
    /// The eroded face has no path between the move's ends at these
    /// clearances, or no circle for an empty loop.
    Room,
    /// No point on the path keeps the node spacing from every node.
    NoSpot,
    /// The path found could not be drawn as one curve: tangled, touching
    /// itself, or too short for its node.
    Shape(&'static str),
    /// The drawn curve reads back as a different move.
    Topology,
    /// A failure the router should never produce in a face it found a path
    /// through; the sweep counts these and holds them at zero.
    Bug(&'static str),
}

/// Room wanted around every node, in node spacings, where the face allows
/// it, so a later loop at that node still has somewhere to go.
const NODE_ROOM: f64 = 2.0;

/// Within this many clearances of an end node the legs of a loop may run
/// as close together as they like.
const END_FREE: f64 = 3.0;

pub(crate) use crate::exact::cross;

/// Clearances the validator holds the player to move to.
#[derive(Clone, Copy)]
struct Rules {
    clearance: f64,
    /// Nearest the line comes to the centre of a node it does not touch.
    node_clearance: f64,
    /// Within this distance of a node a crossing of that node's own spoke
    /// changes nothing: the validator forgives it and the embedding reads
    /// the corner from where the curve leaves the MIN_PATH_CLEARANCE circle.
    forgiven: f64,
    /// Node clearance to fall back to when the wanted one cannot be drawn.
    fallback: f64,
    spacing: f64,
}

fn rules(state: &GameState) -> Rules {
    let s = scale(state.board_size);
    match state.current_player {
        Player::AI => Rules {
            clearance: MIN_PATH_CLEARANCE_AI * s,
            node_clearance: ai_node_clearance(state.board_size),
            forgiven: (AI_INTERSECTION_TOLERANCE / 2.0).min(MIN_PATH_CLEARANCE) * s,
            fallback: MIN_PATH_CLEARANCE * s,
            spacing: MIN_NODE_SPACING_AI * s,
        },
        Player::Human => Rules {
            clearance: MIN_PATH_CLEARANCE * s,
            node_clearance: MIN_PATH_CLEARANCE * s,
            forgiven: (HUMAN_INTERSECTION_TOLERANCE / 2.0).min(MIN_PATH_CLEARANCE) * s,
            fallback: MIN_PATH_CLEARANCE * s,
            spacing: MIN_NODE_SPACING * s,
        },
    }
}

/// The curve for `mv`, drawn to the clearances of the player to move,
/// validated as that player and read back as `mv`.
pub fn route(state: &GameState, mv: &FaceMove) -> Result<Move, Unroutable> {
    let want = rules(state);
    let drawn = route_at(state, mv, &want);
    // The disc clearance is what the AI aims for; a line that cannot have
    // it is still legal at the clearance a human line gets.
    let worth_retrying = matches!(
        drawn,
        Err(Unroutable::Room | Unroutable::NoSpot | Unroutable::Shape(_))
    );
    if !worth_retrying || want.node_clearance <= want.fallback {
        return drawn;
    }
    route_cheap(state, mv)
}

/// `route` at the clearance a human line gets, one attempt.
pub fn route_cheap(state: &GameState, mv: &FaceMove) -> Result<Move, Unroutable> {
    let want = rules(state);
    route_at(
        state,
        mv,
        &Rules {
            node_clearance: want.fallback,
            ..want
        },
    )
}

fn route_at(state: &GameState, mv: &FaceMove, want: &Rules) -> Result<Move, Unroutable> {
    if mv.a == mv.b && mv.enclosed.is_empty() {
        return small_loop::route(state, mv, want.clearance, want.node_clearance);
    }
    let (arena, path) = plan(state, mv, want)?;
    // The relaxed path is the one wanted; the taut path already keeps
    // every clearance when the relaxation ends up somewhere it cannot be
    // used.
    match relax(state, mv, want, &arena, &path) {
        Some(relaxed) => finish(state, mv, relaxed).or_else(|_| finish(state, mv, path)),
        None => finish(state, mv, path),
    }
}

/// The wall clearance the face is eroded by: the fit may move the curve a
/// tolerance off the path, and the walls are flattenings a tolerance off
/// their curves.
fn eroded_by(state: &GameState, want: &Rules) -> f64 {
    want.clearance + curve::tolerance(state.board_size)
}

/// The taut path for `mv` through the eroded face, legs included, in the
/// homotopy class the move names.
fn plan(
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
) -> Result<(erode::Arena, Vec<Point>), Unroutable> {
    let c = eroded_by(state, want);
    let face = state
        .face
        .corner_face(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Bug("corner"))?;
    // The fit may move the curve a tolerance off the path at a disc too.
    let disc = want.node_clearance + curve::tolerance(state.board_size);
    let arena = erode::Arena::build(state, face, mv.a, mv.b, c, disc, want.forgiven)?;
    let entries = vec![arena.entries(state, mv.a, mv.at_a)?];
    let exits = vec![arena.entries(state, mv.b, mv.at_b)?];
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    let blocked = corridor::blocked(&arena, &entries, &exits, ends);
    let cuts = arena.cuts(state, mv, face, &blocked)?;
    let own = state
        .face
        .component_of(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Bug("corner"))?;
    let is_loop = mv.a == mv.b;
    let first = corridor::search(
        &arena,
        &cuts,
        &entries,
        &exits,
        corridor::Want::Any,
        is_loop,
    );
    // A chord's Room means the free triangles disconnect its ends, so a
    // path the search missed is a bug. A loop's Room means no closed walk
    // from the node crosses a cut, which only the parity search decides.
    if !is_loop
        && matches!(first, Err(Unroutable::Room))
        && corridor::reachable(&arena, &entries, &exits)
    {
        return Err(Unroutable::Bug("room with a path"));
    }
    let first = first?;
    let mut path = arena.tighten(&cuts, &corridor::draw(&first, &entries, &exits));
    if cuts.all != 0 {
        let wrong = wrong_sides(state, mv, own, &cuts, &path);
        if wrong != 0 {
            // Flipping every bit at once is the same move drawn round the
            // other side of its own piece, and a bare node has no sides.
            let want = corridor::Want::Parity(first.parity ^ wrong);
            let second = corridor::search(&arena, &cuts, &entries, &exits, want, is_loop)?;
            path = arena.tighten(&cuts, &corridor::draw(&second, &entries, &exits));
        }
    }
    let path = uncross(untwist_legs(path)).ok_or(Unroutable::Shape("tangled"))?;
    Ok((arena, path))
}

/// The path moved off the walls and nodes as far as the room allows.
fn relax(
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    arena: &erode::Arena,
    path: &[Point],
) -> Option<Vec<Point>> {
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    let nodes: Vec<Point> = state
        .nodes
        .iter()
        .filter(|n| n.id != mv.a && n.id != mv.b)
        .map(|n| n.position)
        .collect();
    // The relaxation settles a little short of its room, so the room sits a
    // line clearance above what the validator demands.
    let room = (NODE_ROOM * want.spacing).max(want.node_clearance + want.clearance);
    // The fit may move the curve a tolerance toward a node as well.
    let rooms = clearance::Rooms {
        clearance: eroded_by(state, want),
        node_clearance: want.node_clearance + curve::tolerance(state.board_size),
        node_room: room,
        end_room: want.spacing,
    };
    let obstacles = clearance::Obstacles::new(arena.walls(), nodes, ends, rooms, path);
    clearance::push(path.to_vec(), &obstacles)
}

/// Feeds the flattening and node of every routed search move of `state`
/// into `fnv`, in the model's move order.
#[cfg(any(test, feature = "hash"))]
pub fn hash_routes(fnv: &mut Fnv, state: &GameState) {
    for mv in state.face.search_moves() {
        match route(state, &mv) {
            Ok(drawn) => {
                fnv.points(&drawn.flatten(curve::tolerance(state.board_size)));
                fnv.points(&[drawn.new_node_pos]);
            }
            // A refusal is part of the output too: a platform that turns
            // one into a route, or the other way, has to show.
            Err(why) => fnv.feed(match why {
                Unroutable::Room => 1.0,
                Unroutable::NoSpot => 2.0,
                Unroutable::Shape(_) => 3.0,
                Unroutable::Topology => 4.0,
                Unroutable::Bug(_) => 5.0,
            }),
        }
    }
}

/// Bits of the pieces the path puts on the wrong side of the move.
fn wrong_sides(
    state: &GameState,
    mv: &FaceMove,
    own: Component,
    cuts: &erode::Cuts,
    path: &[Point],
) -> u64 {
    let got = embedding::sides(state, mv.a, mv.at_a, mv.b, mv.at_b, own, path);
    cuts.piece_bits
        .iter()
        .filter(|(comp, _)| {
            let rep = state.face.lowest_node(*comp);
            got.contains(&rep) != mv.enclosed.contains(&rep)
        })
        .fold(0, |acc, (_, bit)| acc | bit)
}

/// Fits the spline, places the node, validates, and checks the curve reads
/// back as the move. A loop at a bare node may read as the other side.
pub(crate) fn finish(
    state: &GameState,
    mv: &FaceMove,
    path: Vec<Point>,
) -> Result<Move, Unroutable> {
    let tolerance = curve::tolerance(state.board_size);
    let Rules {
        clearance, spacing, ..
    } = rules(state);
    // A point on a path of length L is within L/2 of one end, so a path
    // under two spacings has nowhere for its node.
    let length: f64 = path.windows(2).map(|w| w[0].distance_to(&w[1])).sum();
    if length < 2.0 * spacing {
        return Err(Unroutable::Shape("short"));
    }
    // The fit measures its error at the points alone, so a long straight
    // run gets points along it to hold the cubic down. The face is eroded
    // by one fit tolerance, so nothing else may move the path before the
    // fit, and Move::new refits the stored points at this tolerance.
    let path = densify(&path, clearance);
    let segments = curve::fit(&path, tolerance);
    let flat = curve::flatten(&segments, tolerance);
    let node = place_node(state, &segments, &flat, spacing).ok_or(Unroutable::NoSpot)?;
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    // Two passes the relaxation could not pull apart touch, and the fit
    // can cross where they touch: no room for both.
    if self_crossing(&flat) || self_touching(&flat, ends, clearance) {
        return Err(Unroutable::Shape("touching"));
    }
    let mov = Move::from_segments(mv.a, mv.b, path, segments, node, state.current_player);
    match state.current_player {
        Player::AI => validate_ai_move(state, &mov),
        Player::Human => validate_move(state, &mov),
    }
    .map_err(|r| Unroutable::Bug(kind_name(r.kind)))?;
    let read = embedding::embed(state, &mov).map_err(|r| Unroutable::Bug(kind_name(r.kind)))?;
    let mut want = mv.enclosed.clone();
    want.sort_unstable();
    let mut got = read.enclosed.clone();
    got.sort_unstable();
    let same_corners =
        read.a == mv.a && read.b == mv.b && read.at_a == mv.at_a && read.at_b == mv.at_b;
    let bare_loop = mv.a == mv.b && mv.at_a.is_none();
    let sides_match = got == want || (bare_loop && got == other_side(state, mv));
    if same_corners && sides_match {
        Ok(mov)
    } else {
        Err(Unroutable::Topology)
    }
}

fn kind_name(kind: crate::validation::RejectionKind) -> &'static str {
    use crate::validation::RejectionKind::*;
    match kind {
        PathTooCloseToNode => "PathTooCloseToNode",
        PathCrossesLine => "PathCrossesLine",
        NodeTooClose => "NodeTooClose",
        PathTooShort => "PathTooShort",
        PathAlongLine => "PathAlongLine",
        _ => "Rejected",
    }
}

/// A leg the taut path doubles back across is cut at the crossing, so the
/// path runs from the end node straight to where it was heading.
fn untwist_legs(mut path: Vec<Point>) -> Vec<Point> {
    let crossing = |a: Point, b: Point, c: Point, d: Point| {
        clearance::proper_crossing(a, b, c, d)
            .then(|| crate::geometry::segment_intersection(&a, &b, &c, &d))
            .flatten()
    };
    'again: loop {
        let n = path.len().saturating_sub(1);
        for j in 2..n {
            if let Some(x) = crossing(path[0], path[1], path[j], path[j + 1]) {
                let mut out = vec![path[0], x];
                out.extend_from_slice(&path[j + 1..]);
                path = out;
                continue 'again;
            }
        }
        for i in 0..n.saturating_sub(2) {
            if let Some(x) = crossing(path[i], path[i + 1], path[n - 1], path[n]) {
                let mut out = path[..=i].to_vec();
                out.push(x);
                out.push(path[n]);
                path = out;
                continue 'again;
            }
        }
        return path;
    }
}

/// Rounds of uncrossing before a path is given up as tangled.
const UNCROSS_ROUNDS: usize = 8;

/// The shortest path in a parity class can twist into a figure of eight.
/// Reversing the part between the two visits of a crossing joins the
/// lobes into one simple curve through the same free triangles; the
/// relaxation then pulls the two visits apart. None when the path stays
/// tangled.
fn uncross(mut path: Vec<Point>) -> Option<Vec<Point>> {
    for _ in 0..UNCROSS_ROUNDS {
        let n = path.len() - 1;
        let hit = (0..n).find_map(|i| {
            (i + 2..n).find_map(|j| {
                clearance::proper_crossing(path[i], path[i + 1], path[j], path[j + 1])
                    .then(|| {
                        crate::geometry::segment_intersection(
                            &path[i],
                            &path[i + 1],
                            &path[j],
                            &path[j + 1],
                        )
                    })
                    .flatten()
                    .map(|x| (i, j, x))
            })
        });
        let Some((i, j, x)) = hit else {
            return Some(path);
        };
        let mut out = path[..=i].to_vec();
        out.push(x);
        out.extend(path[i + 1..=j].iter().rev());
        out.push(x);
        out.extend_from_slice(&path[j + 1..]);
        path = out;
    }
    None
}

/// The path with points added along any segment longer than `step`, the
/// original points kept.
fn densify(path: &[Point], step: f64) -> Vec<Point> {
    let mut out = Vec::with_capacity(path.len());
    for w in path.windows(2) {
        let (p, q) = (w[0], w[1]);
        let pieces = (p.distance_to(&q) / step).ceil().max(1.0) as usize;
        for k in 0..pieces {
            let t = k as f64 / pieces as f64;
            out.push(Point::new(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t));
        }
    }
    if let Some(last) = path.last() {
        out.push(*last);
    }
    out
}

/// Whether two segments of the polyline that do not touch cross each other.
pub(crate) fn self_crossing(flat: &[Point]) -> bool {
    let n = flat.len().saturating_sub(1);
    (0..n).any(|i| {
        (i + 2..n).any(|j| clearance::proper_crossing(flat[i], flat[i + 1], flat[j], flat[j + 1]))
    })
}

/// Whether the curve, away from its ends along its length, comes within
/// `c` of an end node or of a part of itself well apart along it, either
/// of which the embedding could read as a different move. The legs of a
/// loop converge at their node, so the stretch nearest each end is left
/// alone.
fn self_touching(flat: &[Point], ends: [Point; 2], c: f64) -> bool {
    let dense = resample_polyline(flat, c / 2.0);
    let n = dense.len();
    let free = (2.0 * END_FREE) as usize;
    let mid = |i: usize| i >= free && i + free < n;
    dense.iter().enumerate().any(|(i, p)| {
        mid(i)
            && (ends.iter().any(|e| e.distance_to(p) < c)
                || dense
                    .iter()
                    .enumerate()
                    .skip(i + clearance::SELF_GAP)
                    .any(|(j, q)| mid(j) && p.distance_to(q) < c))
    })
}

/// The best-ranked spot on the curve a clear disc from every node, else
/// the best at the node spacing, in each case a human spacing in from the
/// board edge; none when the curve offers no such spot. Spots are measured
/// where the move will snap them, on the curve itself.
fn place_node(
    state: &GameState,
    segments: &[curve::Cubic],
    flat: &[Point],
    spacing: f64,
) -> Option<Point> {
    let s = scale(state.board_size);
    let dense = resample_polyline(flat, MIN_PATH_CLEARANCE_AI * s / 2.0);
    let inset = MIN_NODE_SPACING * s;
    let far = state.board_size as f64 - inset;
    let ranked: Vec<Point> = rank_node_placements(&dense, state)
        .into_iter()
        .map(|q| curve::nearest(segments, &q).map_or(q, |hit| hit.point))
        .collect();
    let spot = |apart: f64| {
        ranked.iter().copied().find(|q| {
            (inset..=far).contains(&q.x)
                && (inset..=far).contains(&q.y)
                && state
                    .nodes
                    .iter()
                    .all(|n| n.position.distance_to(q) >= apart)
        })
    };
    // The validator's test is a strict less-than on a spot snapped twice.
    spot(spacing.max(2.0 * node_disc(state.board_size))).or_else(|| spot(spacing + 1e-6))
}

/// The pieces a bare node's loop would enclose from its other side.
// A bare loop's two sides are the same position on the sphere, so either
// reading of which pieces it encloses is the listed move.
fn other_side(state: &GameState, mv: &FaceMove) -> Vec<NodeId> {
    let face = state.face.corner_face(mv.a, mv.at_a).expect("own corner");
    let mut out: Vec<NodeId> = state
        .face
        .components(face)
        .iter()
        .filter(|&&c| c != Component::Node(mv.a))
        .map(|&c| state.face.lowest_node(c))
        .filter(|v| !mv.enclosed.contains(v))
        .collect();
    out.sort_unstable();
    out
}
