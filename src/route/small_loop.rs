//! A self-loop that encloses nothing: a small circle off the node, in the
//! direction with the most room.

use super::{finish, Unroutable};
use crate::embedding::{half_edge_path, leaving_point};
use crate::exact::unit_circle;
use crate::face::Move as FaceMove;
use crate::geometry::closest_point_on_segment;
use crate::types::{node_disc, scale, GameState, Move, Point, MIN_PATH_CLEARANCE};

/// Circle radii to try as multiples of the smallest, largest first. The
/// smallest spans three disc radii, so the loop shows beside the node and
/// its new node sits a clear disc away from the original.
const RADII: [f64; 3] = [1.5, 1.25, 1.0];
const LOOP_DISCS: f64 = 1.5;
/// Headings tried round the node, and points along each circle.
const STEPS: usize = 32;
/// Unit circle points, four per step: a circle's points sit a quarter
/// step past the heading grid, so none has a mirror image across the
/// circle's axis through the node and the fit never splits on a tie.
const TURN: usize = 4 * STEPS;
/// Cosine of 110 degrees: the circle's centre keeps that far from the
/// half-plane an existing spoke would need to cross it.
const SPOKE_COS: f64 = -0.342_020_143_325_668_7;
/// How many of the best-scoring circles get fitted and validated.
const TRIES: usize = 6;

pub(super) fn route(
    state: &GameState,
    mv: &FaceMove,
    c: f64,
    node_clearance: f64,
) -> Result<Move, Unroutable> {
    let turn = unit_circle(TURN);
    let least = LOOP_DISCS * node_disc(state.board_size);
    let a = state.nodes[mv.a].position;
    let lookup = MIN_PATH_CLEARANCE * scale(state.board_size);
    let spokes: Vec<Point> = state
        .face
        .rotation(mv.a)
        .iter()
        .filter_map(|&h| leaving_point(a, &half_edge_path(state, h), lookup))
        .map(|p| Point::new(p.x - a.x, p.y - a.y))
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

    let mut candidates: Vec<(f64, f64, usize)> = Vec::new();
    for radius in RADII.map(|k| k * least) {
        for heading in 0..STEPS {
            let u = outward(&turn, 4 * heading);
            let clear_of_spokes = spokes
                .iter()
                .all(|s| u.x * s.x + u.y * s.y < SPOKE_COS * crate::exact::length(s.x, s.y));
            if !clear_of_spokes {
                continue;
            }
            let centre = Point::new(a.x + radius * u.x, a.y + radius * u.y);
            let score = score(centre, radius, a, c, node_clearance, &segments, &nodes);
            if score >= 0.0 {
                candidates.push((score, radius, heading));
            }
        }
    }
    candidates.sort_by(|x, y| y.0.total_cmp(&x.0));
    let mut why = Unroutable::Room;
    for (_, radius, heading) in candidates.into_iter().take(TRIES) {
        match finish(state, mv, circle(a, radius, heading, &turn)) {
            Ok(mov) => return Ok(mov),
            Err(e) => why = e,
        }
    }
    Err(why)
}

/// Unit vector `k` of the turn on screen, where y grows downward, so the
/// turn runs counterclockwise as seen.
fn outward(turn: &[Point], k: usize) -> Point {
    let u = turn[k % TURN];
    Point::new(u.x, -u.y)
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
/// counterclockwise on screen, closed at `a`: `a`, then `STEPS` points
/// starting a quarter step past the point opposite the centre.
fn circle(a: Point, radius: f64, heading: usize, turn: &[Point]) -> Vec<Point> {
    let u = outward(turn, 4 * heading);
    let centre = Point::new(a.x + radius * u.x, a.y + radius * u.y);
    let mut out = Vec::with_capacity(STEPS + 2);
    out.push(a);
    for k in 0..STEPS {
        let u = outward(turn, 4 * heading + TURN / 2 + 4 * k + 1);
        out.push(Point::new(centre.x + radius * u.x, centre.y + radius * u.y));
    }
    out.push(a);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::{distance, Fnv};

    /// The circles round a grid point at the smallest loop radius, as a Mac
    /// records them; the sqrt-only build gives the same bits everywhere.
    const CIRCLES_HASH: u64 = 0x8597_4a1b_04a8_4232;

    #[test]
    fn a_loop_circle_lies_on_its_circle_and_has_no_mirror_points() {
        let turn = unit_circle(TURN);
        let a = Point::new(713.0, 778.0);
        let radius = LOOP_DISCS * node_disc(1000);
        let mut fnv = Fnv::default();
        for heading in 0..STEPS {
            let pts = circle(a, radius, heading, &turn);
            assert_eq!(pts.len(), STEPS + 2);
            assert_eq!(pts[0], a);
            assert_eq!(pts[STEPS + 1], a);
            let u = outward(&turn, 4 * heading);
            let centre = Point::new(a.x + radius * u.x, a.y + radius * u.y);
            // The mirror image of p across the axis through a and the
            // centre.
            let mirror = |p: Point| {
                let (dx, dy) = (p.x - centre.x, p.y - centre.y);
                let along = dx * u.x + dy * u.y;
                let across = dx * u.y - dy * u.x;
                Point::new(
                    centre.x + along * u.x - across * u.y,
                    centre.y + along * u.y + across * u.x,
                )
            };
            for p in &pts[1..=STEPS] {
                assert!(
                    (distance(*p, centre) - radius).abs() < 1e-9,
                    "{heading}: {p:?}"
                );
                let m = mirror(*p);
                let nearest = pts[1..=STEPS]
                    .iter()
                    .map(|q| distance(*q, m))
                    .fold(f64::INFINITY, f64::min);
                // A quarter step off the mirror grid: at least half a
                // chord of the circle away from every other point.
                assert!(
                    nearest > radius * 0.09,
                    "{heading}: {p:?} mirrors onto {m:?}"
                );
            }
            fnv.points(&pts);
        }
        assert_eq!(fnv.0, CIRCLES_HASH, "circles hash {}", fnv.hex());
    }
}
