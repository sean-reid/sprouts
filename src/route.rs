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
use crate::face::{Component, Move as FaceMove, NodeId};
use crate::geometry::{decimate_polyline, resample_polyline};
use crate::types::{
    ai_node_clearance, node_disc, scale, GameState, Move, Player, Point, MIN_NODE_SPACING,
    MIN_NODE_SPACING_AI, MIN_PATH_CLEARANCE, MIN_PATH_CLEARANCE_AI,
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
    /// The eroded face has no path for the move at these clearances, or no
    /// spot on it for the new node.
    Room,
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

/// Screen-side test: positive when `q` is on the right of the heading
/// `o` to `p`, where y grows downward.
pub(crate) fn cross(o: Point, p: Point, q: Point) -> f64 {
    (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x)
}

/// Clearances the validator holds the player to move to.
#[derive(Clone, Copy)]
struct Rules {
    clearance: f64,
    /// Nearest the line comes to the centre of a node it does not touch.
    node_clearance: f64,
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
            fallback: MIN_PATH_CLEARANCE * s,
            spacing: MIN_NODE_SPACING_AI * s,
        },
        Player::Human => Rules {
            clearance: MIN_PATH_CLEARANCE * s,
            node_clearance: MIN_PATH_CLEARANCE * s,
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
    if !matches!(drawn, Err(Unroutable::Room)) || want.node_clearance <= want.fallback {
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
    let arena = erode::Arena::build(state, face, mv.a, mv.b, c, want.node_clearance)?;
    let entries = arena.entries(state, mv.a, mv.at_a, c);
    let exits = arena.entries(state, mv.b, mv.at_b, c);
    if entries.is_empty() || exits.is_empty() {
        return Err(Unroutable::Room);
    }
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
    )?;
    let mut path = corridor::draw(&first, &entries, &exits);
    if cuts.all != 0 {
        let wrong = wrong_sides(state, mv, own, &cuts, &path);
        if wrong != 0 {
            // Flipping every bit at once is the same move drawn round the
            // other side of its own piece, and a bare node has no sides.
            let want = corridor::Want::Parity(first.parity ^ wrong);
            let second = corridor::search(&arena, &cuts, &entries, &exits, want, is_loop)?;
            path = corridor::draw(&second, &entries, &exits);
        }
    }
    let path = uncross(untwist_legs(path)).ok_or(Unroutable::Room)?;
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
    let obstacles = clearance::Obstacles::new(
        arena.walls(),
        nodes,
        ends,
        eroded_by(state, want),
        want.node_clearance,
        room,
        want.spacing,
    );
    clearance::push(path.to_vec(), &obstacles)
}

/// Writes the arena and the taut path as an SVG under the directory in
/// SPROUTS_ARENA_SVG, when set.
#[cfg(test)]
fn dump_arena(
    out: &mut String,
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    arena: &erode::Arena,
    path: &[Point],
) {
    use std::fmt::Write;
    let Ok(dir) = std::env::var("SPROUTS_ARENA_SVG") else {
        return;
    };
    let file = format!(
        "{dir}/arena-{}-{}-{}-{}.svg",
        mv.a,
        mv.b,
        mv.at_a.map_or(0, |h| h + 1),
        want.node_clearance as u32
    );
    let mut svg = arena.svg(state.board_size as f64);
    let points: Vec<String> = path
        .iter()
        .map(|p| format!("{:.2},{:.2}", p.x, p.y))
        .collect();
    svg = svg.replace(
        "</svg>",
        &format!(
            "<polyline points=\"{}\" fill=\"none\" stroke=\"red\" stroke-width=\"1\"/></svg>",
            points.join(" ")
        ),
    );
    std::fs::write(&file, svg).ok();
    let _ = writeln!(out, "  arena written to {file}");
}

/// How a route of `mv` goes, step by step, for working a failure down by
/// hand.
#[cfg(test)]
pub(crate) fn explain(state: &GameState, mv: &FaceMove) -> String {
    use std::fmt::Write;
    let mut out = String::new();
    let base = rules(state);
    for want in [
        base,
        Rules {
            node_clearance: base.fallback,
            ..base
        },
    ] {
        let _ = writeln!(out, "node clearance {}", want.node_clearance);
        let (arena, path) = match plan(state, mv, &want) {
            Ok(planned) => planned,
            Err(e) => {
                let _ = writeln!(out, "  no plan: {e:?}");
                let c = eroded_by(state, &want);
                let face = state.face.corner_face(mv.a, mv.at_a).unwrap();
                if let Ok(arena) =
                    erode::Arena::build(state, face, mv.a, mv.b, c, want.node_clearance)
                {
                    let entries = arena.entries(state, mv.a, mv.at_a, c);
                    let exits = arena.entries(state, mv.b, mv.at_b, c);
                    let _ = writeln!(
                        out,
                        "  {} entries {:?}, {} exits {:?}",
                        entries.len(),
                        entries.iter().map(|e| e.at).collect::<Vec<_>>(),
                        exits.len(),
                        exits.iter().map(|e| e.at).collect::<Vec<_>>()
                    );
                    let _ = writeln!(out, "  {}", arena.describe());
                    dump_arena(&mut out, state, mv, &want, &arena, &[]);
                }
                continue;
            }
        };
        dump_arena(&mut out, state, mv, &want, &arena, &path);
        let _ = writeln!(out, "  {}", arena.describe());
        let _ = writeln!(
            out,
            "  taut path of {} points: {:?}",
            path.len(),
            &path[..path.len().min(4)]
        );
        let relaxed = relax(state, mv, &want, &arena, &path);
        let _ = writeln!(
            out,
            "  relaxation {}",
            if relaxed.is_some() {
                "held"
            } else {
                "crossed a wall"
            }
        );
        for (name, candidate) in [("relaxed", relaxed), ("taut", Some(path))] {
            let Some(candidate) = candidate else { continue };
            let _ = writeln!(
                out,
                "  {name}: {:?}",
                finish(state, mv, candidate.clone()).map(|_| ())
            );
            let tolerance = curve::tolerance(state.board_size);
            let dense = densify(&decimate_polyline(&candidate, tolerance), want.clearance);
            let segments = curve::fit(&dense, tolerance);
            let flat = curve::flatten(&segments, tolerance);
            if let Some(node) = place_node(state, &flat, want.spacing) {
                let mov =
                    Move::from_segments(mv.a, mv.b, dense, segments, node, state.current_player);
                let _ = writeln!(
                    out,
                    "    validation {:?}",
                    validate_ai_move(state, &mov).map_err(|r| (r.kind, r.at))
                );
                for w in flat.windows(2) {
                    for line in &state.lines {
                        for l in line.flat.windows(2) {
                            if clearance::proper_crossing(w[0], w[1], l[0], l[1]) {
                                let _ = writeln!(
                                    out,
                                    "    curve {:?}-{:?} crosses line {} at {:?}-{:?}",
                                    w[0], w[1], line.id, l[0], l[1]
                                );
                            }
                        }
                    }
                }
                let _ = writeln!(
                    out,
                    "    path end {:?}",
                    &candidate[candidate.len().saturating_sub(3)..]
                );
                let _ = writeln!(
                    out,
                    "    embedding {:?}",
                    embedding::embed(state, &mov).map_err(|r| (r.kind, r.at))
                );
                let radius = MIN_PATH_CLEARANCE * scale(state.board_size);
                for (name, v) in [("a", mv.a), ("b", mv.b)] {
                    let centre = state.nodes[v].position;
                    let outward: Vec<Point> = if name == "a" {
                        flat.clone()
                    } else {
                        flat.iter().rev().copied().collect()
                    };
                    let spokes: Vec<(usize, f64)> = state
                        .face
                        .rotation(v)
                        .iter()
                        .map(|&h| {
                            (
                                h,
                                embedding::leaving_angle(
                                    centre,
                                    &embedding::half_edge_path(state, h),
                                    radius,
                                ),
                            )
                        })
                        .collect();
                    let _ = writeln!(
                        out,
                        "    leaves {name} at {:.3}, spokes {spokes:?}",
                        embedding::leaving_angle(centre, &outward, radius)
                    );
                }
            }
        }
    }
    out
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
        return Err(Unroutable::Room);
    }
    // Wiggles under the fit tolerance are relaxation noise, not shape;
    // dropping them first gives the fit a few long, even cubics. The fit
    // measures its error at the points alone, so a long straight run gets
    // points along it to hold the cubic down.
    let path = densify(&decimate_polyline(&path, tolerance), clearance);
    let segments = curve::fit(&path, tolerance);
    let flat = curve::flatten(&segments, tolerance);
    let node = place_node(state, &flat, spacing).ok_or(Unroutable::Room)?;
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    // Two passes the relaxation could not pull apart touch, and the fit
    // can cross where they touch: no room for both.
    if self_crossing(&flat) || self_touching(&flat, ends, clearance) {
        return Err(Unroutable::Room);
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
/// board edge; none when the curve offers no such spot.
fn place_node(state: &GameState, flat: &[Point], spacing: f64) -> Option<Point> {
    let s = scale(state.board_size);
    let dense = resample_polyline(flat, MIN_PATH_CLEARANCE_AI * s / 2.0);
    let inset = MIN_NODE_SPACING * s;
    let far = state.board_size as f64 - inset;
    let ranked = rank_node_placements(&dense, state);
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
    spot(spacing.max(2.0 * node_disc(state.board_size))).or_else(|| spot(spacing))
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
