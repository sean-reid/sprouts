//! A self-loop that encloses nothing: a small circle off the node, in the
//! direction with the most room.

use std::f64::consts::{PI, TAU};

use super::{finish, Unroutable};
use crate::embedding::{half_edge_path, leaving_angle};
use crate::face::Move as FaceMove;
use crate::geometry::closest_point_on_segment;
use crate::types::{node_disc, scale, GameState, Move, Point, MIN_PATH_CLEARANCE};

/// Circle radii to try as multiples of the smallest, largest first. The
/// smallest spans three disc radii, so the loop shows beside the node and
/// its new node sits a clear disc away from the original.
const RADII: [f64; 3] = [1.5, 1.25, 1.0];
const LOOP_DISCS: f64 = 1.5;
const HEADINGS: usize = 36;
const SAMPLES: usize = 32;
/// The circle's centre keeps this far, in radians, from the half-plane an
/// existing spoke would need to cross it.
const SPOKE_MARGIN: f64 = PI / 9.0;
/// How many of the best-scoring circles get fitted and validated.
const TRIES: usize = 6;

pub(super) fn route(
    state: &GameState,
    mv: &FaceMove,
    c: f64,
    node_clearance: f64,
) -> Result<Move, Unroutable> {
    let least = LOOP_DISCS * node_disc(state.board_size);
    let a = state.nodes[mv.a].position;
    let lookup = MIN_PATH_CLEARANCE * scale(state.board_size);
    let spokes: Vec<f64> = state
        .face
        .rotation(mv.a)
        .iter()
        .map(|&h| leaving_angle(a, &half_edge_path(state, h), lookup))
        .collect();
    let edge = state.board_size as f64;
    let rim = [
        Point::new(0.0, 0.0),
        Point::new(0.0, edge),
        Point::new(edge, edge),
        Point::new(edge, 0.0),
        Point::new(0.0, 0.0),
    ];
    // Each wall segment, flagged when it belongs to one of the node's own
    // spokes, whose first stretch starts inside any circle through the node.
    let mut segments: Vec<(Point, Point, bool)> =
        rim.windows(2).map(|w| (w[0], w[1], false)).collect();
    for line in &state.lines {
        let own = line.from_node == mv.a || line.to_node == mv.a;
        segments.extend(line.flat.windows(2).map(|w| (w[0], w[1], own)));
    }
    let nodes: Vec<Point> = state
        .nodes
        .iter()
        .filter(|n| n.id != mv.a)
        .map(|n| n.position)
        .collect();

    let mut candidates: Vec<(f64, f64, f64)> = Vec::new();
    for radius in RADII.map(|k| k * least) {
        for k in 0..HEADINGS {
            let heading = TAU * k as f64 / HEADINGS as f64;
            let clear_of_spokes = spokes.iter().all(|&s| {
                let gap = (heading - s).rem_euclid(TAU);
                gap.min(TAU - gap) > PI / 2.0 + SPOKE_MARGIN
            });
            if !clear_of_spokes {
                continue;
            }
            let centre = Point::new(a.x + radius * heading.cos(), a.y - radius * heading.sin());
            let score = score(centre, radius, a, c, node_clearance, &segments, &nodes);
            if score >= 0.0 {
                candidates.push((score, radius, heading));
            }
        }
    }
    candidates.sort_by(|x, y| y.0.total_cmp(&x.0));
    let mut why = Unroutable::Room;
    for (_, radius, heading) in candidates.into_iter().take(TRIES) {
        match finish(state, mv, circle(a, radius, heading)) {
            Ok(mov) => return Ok(mov),
            Err(e) => why = e,
        }
    }
    Err(why)
}

/// Room around the circle: the least clearance of any point on it, minus
/// what that point needs, and very negative when the disc holds a node or
/// a line runs through it.
fn score(
    centre: Point,
    radius: f64,
    a: Point,
    c: f64,
    node_clearance: f64,
    segments: &[(Point, Point, bool)],
    nodes: &[Point],
) -> f64 {
    let mut room = f64::INFINITY;
    for n in nodes {
        let d = n.distance_to(&centre);
        if d < radius + node_clearance {
            return f64::NEG_INFINITY;
        }
        room = room.min(d - radius - node_clearance);
    }
    for (p, q, own) in segments {
        let (at, d) = closest_point_on_segment(p, q, &centre);
        if *own && at.distance_to(&a) < 2.0 * c {
            continue;
        }
        if d < radius {
            return f64::NEG_INFINITY;
        }
        room = room.min(d - radius - c);
    }
    room
}

/// The circle through `a` with its centre `radius` away along `heading`,
/// counterclockwise on screen, closed at `a`.
fn circle(a: Point, radius: f64, heading: f64) -> Vec<Point> {
    let centre = Point::new(a.x + radius * heading.cos(), a.y - radius * heading.sin());
    let from = heading + PI;
    let mut out: Vec<Point> = (0..SAMPLES)
        .map(|k| {
            let t = from + TAU * k as f64 / SAMPLES as f64;
            Point::new(centre.x + radius * t.cos(), centre.y - radius * t.sin())
        })
        .collect();
    out[0] = a;
    out.push(a);
    out
}
