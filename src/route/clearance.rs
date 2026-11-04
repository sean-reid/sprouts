//! Pushes a path off the face boundary and away from nodes, as far as the
//! room around it allows.

use crate::geometry::{closest_point_on_segment, segments_intersect};
use crate::types::Point;

const ROUNDS: usize = 40;
/// A moved segment that comes within one grid step of a wall vertex or a
/// node touches it and counts as crossing there. Distance from the walls
/// themselves is the hard step's job, and finish refuses a fitted curve
/// nearer any line than the player's clearance.
const GRAZE: f64 = crate::exact::GRID;
/// Share of the shortfall a point moves per round, so two walls closer than
/// twice the clearance settle the point between them instead of bouncing.
const STEP: f64 = 0.35;
/// Longest move of one point in one round, in clearances.
const STRIDE: f64 = 0.35;
const SMOOTHING: f64 = 0.4;
/// Rounds of smoothing alone once the pushes have settled.
const POLISH: usize = 40;
/// Clearance to a boundary segment tapers to nothing at the end nodes, where
/// the path has to pass between the spokes already there.
pub(super) const TAPER: f64 = 0.6;
/// Room wanted from a line, in clearances, where the face allows it.
const LINE_ROOM: f64 = 4.0;
/// Beyond its room a point still drifts away from the nearest line, in
/// small steps, until it is this many clearances off, so a line wrapping
/// another swings wide where there is space instead of tracing it.
const COMFORT: f64 = 8.0;
/// Drift per round toward the comfortable distance, in clearances.
const DRIFT: f64 = 0.1;
/// Room wanted between two parts of the path itself, in clearances, so the
/// legs of a loop pushed by the same obstacle do not fall onto each other.
const SELF_ROOM: f64 = 2.0;
/// Path points this many places apart or more count as separate parts:
/// six clearances of path at the sampling below, which is more than the
/// arc around any single corner.
pub(super) const SELF_GAP: usize = 12;
/// The middle of the path, as a share of its points either side of the
/// centre, keeps the new node's spacing from both end nodes so the node
/// has somewhere to go on a short line.
const MIDDLE: f64 = 0.25;
/// Path points about this fraction of the clearance apart.
pub(super) const SPACING: f64 = 0.5;

/// The distances a relaxed path keeps.
#[derive(Clone, Copy)]
pub(super) struct Rooms {
    pub clearance: f64,
    /// Distance the validator demands from every node other than the ends.
    pub node_clearance: f64,
    /// Room wanted from every node other than the ends.
    pub node_room: f64,
    /// Room the middle of the path wants from the end nodes.
    pub end_room: f64,
}

pub(super) struct Obstacles<'a> {
    segments: &'a [(Point, Point)],
    nodes: Vec<Point>,
    ends: [Point; 2],
    clearance: f64,
    node_clearance: f64,
    node_room: f64,
    end_room: f64,
    /// Unit normal to the chord between the ends on the side the middle
    /// bows out to.
    bow: (f64, f64),
}

/// A feature a point is short of its room from: where it is, how far, how
/// far it has to be.
struct Short {
    at: Point,
    distance: f64,
    required: f64,
}

impl<'a> Obstacles<'a> {
    pub(super) fn new(
        segments: &'a [(Point, Point)],
        nodes: Vec<Point>,
        ends: [Point; 2],
        rooms: Rooms,
        path: &[Point],
    ) -> Self {
        let mut obstacles = Self {
            segments,
            nodes,
            ends,
            clearance: rooms.clearance,
            node_clearance: rooms.node_clearance,
            node_room: rooms.node_room,
            end_room: rooms.end_room,
            bow: normal(ends[0], ends[1]),
        };
        obstacles.bow = obstacles.bow_side(path);
        obstacles
    }

    /// The side of the chord between the ends the middle of the path bows
    /// out to: the side the taut path already leans to by a clearance, else
    /// the side with more room at the chord's midpoint. A hair's lean
    /// decides nothing, so a board a rounding error off bows the same way.
    fn bow_side(&self, path: &[Point]) -> (f64, f64) {
        let [a, b] = self.ends;
        let n = normal(a, b);
        let lean: f64 = path
            .iter()
            .map(|p| (p.x - a.x) * n.0 + (p.y - a.y) * n.1)
            .sum::<f64>()
            / path.len().max(1) as f64;
        if lean.abs() > self.clearance {
            return if lean > 0.0 { n } else { (-n.0, -n.1) };
        }
        let mid = Point::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
        let probe = self.end_room / 2.0;
        let room = |side: (f64, f64)| {
            let q = Point::new(mid.x + side.0 * probe, mid.y + side.1 * probe);
            let walls = self
                .nearest_line(q)
                .map_or(f64::INFINITY, |(_, distance)| distance);
            self.nodes
                .iter()
                .map(|v| v.distance_to(&q))
                .fold(walls, f64::min)
        };
        if room(n) >= room((-n.0, -n.1)) {
            n
        } else {
            (-n.0, -n.1)
        }
    }

    /// Where point `i` should step to clear what the validator demands:
    /// the line clearance from walls and the node clearance from nodes,
    /// with no soft room pulling the other way.
    fn hard_step(&self, path: &[Point], i: usize, stride: f64) -> Option<Point> {
        let p = path[i];
        let (mut dx, mut dy) = (0.0, 0.0);
        let mut push = |at: Point, distance: f64, required: f64| {
            if distance < required {
                let away = direction(at, p).unwrap_or_else(|| normal(path[i - 1], path[i + 1]));
                let amount = (required - distance) * STEP;
                dx += away.0 * amount;
                dy += away.1 * amount;
            }
        };
        for (a, b) in self.segments {
            let (at, distance) = closest_point_on_segment(a, b, &p);
            push(at, distance, self.clearance);
        }
        let to_end = self
            .ends
            .iter()
            .map(|e| e.distance_to(&p))
            .fold(f64::INFINITY, f64::min);
        if to_end >= self.node_clearance {
            for n in &self.nodes {
                push(*n, n.distance_to(&p), self.node_clearance);
            }
        }
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-9 {
            return None;
        }
        let scale = stride.min(len) / len;
        Some(Point::new(p.x + dx * scale, p.y + dy * scale))
    }

    /// Every feature point `i` is short of its room from. Only violations
    /// matter, so anything outside its room's square is skipped before a
    /// distance is computed.
    fn shortfalls(&self, path: &[Point], i: usize) -> Vec<Short> {
        let p = path[i];
        let to_end = self
            .ends
            .iter()
            .map(|e| e.distance_to(&p))
            .fold(f64::INFINITY, f64::min);
        let segment_need = (LINE_ROOM * self.clearance).min(TAPER * to_end);
        let self_need = (SELF_ROOM * self.clearance).min(TAPER * to_end);
        let mut out = Vec::new();
        let mut consider = |at: Point, distance: f64, required: f64| {
            if distance < required {
                out.push(Short {
                    at,
                    distance,
                    required,
                });
            }
        };
        let within = |q: Point, need: f64| (q.x - p.x).abs() < need && (q.y - p.y).abs() < need;
        for (a, b) in self.segments {
            let near_box = p.x > a.x.min(b.x) - segment_need
                && p.x < a.x.max(b.x) + segment_need
                && p.y > a.y.min(b.y) - segment_need
                && p.y < a.y.max(b.y) + segment_need;
            if near_box {
                let (at, distance) = closest_point_on_segment(a, b, &p);
                consider(at, distance, segment_need);
            }
        }
        for n in &self.nodes {
            if within(*n, self.node_room) {
                consider(*n, n.distance_to(&p), self.node_room);
            }
        }
        for (j, q) in path.iter().enumerate() {
            if i.abs_diff(j) >= SELF_GAP && within(*q, self_need) {
                consider(*q, q.distance_to(&p), self_need);
            }
        }
        let centre = path.len() as f64 / 2.0;
        if (i as f64 - centre).abs() <= MIDDLE * path.len() as f64 {
            // The push is square to the chord between the ends, so a short
            // straight line bows out instead of stretching along itself.
            let from = Point::new(p.x - self.bow.0, p.y - self.bow.1);
            for e in &self.ends {
                consider(from, e.distance_to(&p), self.end_room);
            }
        }
        out
    }

    /// The worst shortfall at point `i`, zero when it is clear.
    fn shortfall(&self, path: &[Point], i: usize) -> f64 {
        self.shortfalls(path, i)
            .iter()
            .map(|s| s.required - s.distance)
            .fold(0.0, f64::max)
    }

    /// Where point `i` should step to ease every shortfall at once, each
    /// feature pushing it away by a share of what it is short, the whole
    /// step held to a stride.
    fn step(&self, path: &[Point], i: usize, stride: f64) -> Option<Point> {
        let shorts = self.shortfalls(path, i);
        if shorts.is_empty() {
            return None;
        }
        let (mut dx, mut dy) = (0.0, 0.0);
        for short in &shorts {
            let away =
                direction(short.at, path[i]).unwrap_or_else(|| normal(path[i - 1], path[i + 1]));
            let amount = (short.required - short.distance) * STEP;
            dx += away.0 * amount;
            dy += away.1 * amount;
        }
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-9 {
            return None;
        }
        let scale = stride.min(len) / len;
        Some(Point::new(path[i].x + dx * scale, path[i].y + dy * scale))
    }

    /// Shortfall point `i` would have at `target`.
    fn shortfall_at(&self, path: &mut [Point], i: usize, target: Point) -> f64 {
        let was = path[i];
        path[i] = target;
        let after = self.shortfall(path, i);
        path[i] = was;
        after
    }

    /// The nearest boundary segment to `p` and how far it is.
    fn nearest_line(&self, p: Point) -> Option<(Point, f64)> {
        self.segments
            .iter()
            .map(|(a, b)| closest_point_on_segment(a, b, &p))
            .min_by(|x, y| x.1.total_cmp(&y.1))
    }

    /// Whether the segment from `a` to `b` crosses the boundary or a part of
    /// the path other than the segments around point `i`. Passing through a
    /// wall vertex or a node counts, since a crossing there touches no
    /// segment properly, except the vertices on an end of the path for the
    /// segment meeting that end: the end's own spokes start there.
    fn crosses(&self, path: &[Point], i: usize, a: Point, b: Point) -> bool {
        if self
            .segments
            .iter()
            .any(|(p, q)| proper_crossing(a, b, *p, *q))
        {
            return true;
        }
        let leaving = (i == 1).then(|| path[0]);
        let arriving = (i + 2 == path.len()).then(|| path[path.len() - 1]);
        let own_end = |v: &Point| {
            leaving
                .into_iter()
                .chain(arriving)
                .any(|e| e.distance_to(v) <= GRAZE)
        };
        let corners = self
            .segments
            .iter()
            .map(|(p, _)| p)
            .chain(self.nodes.iter());
        if corners
            .into_iter()
            .any(|v| !own_end(v) && closest_point_on_segment(&a, &b, v).1 <= GRAZE)
        {
            return true;
        }
        // Only the two segments that move are left out; their neighbours
        // share an end with them, which the crossing test already allows.
        path.windows(2).enumerate().any(|(j, w)| {
            let moving = j + 1 == i || j == i;
            !moving && proper_crossing(a, b, w[0], w[1])
        })
    }
}

/// Resamples the path and nudges every point away from the boundary and
/// the nodes until each has its room or the walls allow no more.
pub(super) fn push(threaded: Vec<Point>, obstacles: &Obstacles) -> Option<Vec<Point>> {
    let mut path = resample(&threaded, SPACING * obstacles.clearance);
    let stride = STRIDE * obstacles.clearance;
    for _ in 0..ROUNDS {
        let mut moved = false;
        // Every target comes from the same picture of the path, so points
        // on facing legs step apart together instead of chasing each other.
        let targets: Vec<Option<Point>> = (0..path.len())
            .map(|i| {
                if i == 0 || i == path.len() - 1 {
                    return None;
                }
                obstacles.step(&path, i, stride)
            })
            .collect();
        for (i, target) in targets.into_iter().enumerate() {
            if let Some(target) = target {
                if try_move(&mut path, i, target, obstacles) {
                    moved = true;
                }
            }
        }
        smooth(&mut path, obstacles);
        if !moved {
            break;
        }
    }
    for _ in 0..POLISH {
        smooth(&mut path, obstacles);
    }
    // The soft rooms balance where they cannot all be met, which can leave
    // a point under a hard clearance that the room beyond it allows; a last
    // pass moves only those points.
    for _ in 0..ROUNDS {
        let targets: Vec<Option<Point>> = (0..path.len())
            .map(|i| {
                if i == 0 || i == path.len() - 1 {
                    return None;
                }
                obstacles.hard_step(&path, i, stride)
            })
            .collect();
        let mut moved = false;
        for (i, target) in targets.into_iter().enumerate() {
            if let Some(target) = target {
                if try_move(&mut path, i, target, obstacles) {
                    moved = true;
                }
            }
        }
        if !moved {
            break;
        }
    }
    let boundary = |w: &[Point]| {
        obstacles
            .segments
            .iter()
            .any(|(p, q)| proper_crossing(w[0], w[1], *p, *q))
    };
    if path.windows(2).any(boundary) {
        return None;
    }
    Some(path)
}

/// Moves each point part way to the midpoint of its neighbours where that
/// costs it no room it already has, then a small step away from the
/// nearest line while it is closer than the comfortable distance.
fn smooth(path: &mut [Point], obstacles: &Obstacles) {
    for i in 1..path.len() - 1 {
        if let Some((at, distance)) = obstacles.nearest_line(path[i]) {
            // Near the end nodes the drift tapers like the room does, so the
            // legs still meet their node instead of being bent off its spokes.
            let to_end = obstacles
                .ends
                .iter()
                .map(|e| e.distance_to(&path[i]))
                .fold(f64::INFINITY, f64::min);
            let comfort = (COMFORT * obstacles.clearance).min(TAPER * to_end);
            if distance < comfort {
                if let Some(away) = direction(at, path[i]) {
                    let step = DRIFT * obstacles.clearance;
                    let target = Point::new(path[i].x + away.0 * step, path[i].y + away.1 * step);
                    if obstacles.shortfall_at(path, i, target) <= obstacles.shortfall(path, i) {
                        try_move(path, i, target, obstacles);
                    }
                }
            }
        }
        let mid = Point::new(
            (path[i - 1].x + path[i + 1].x) / 2.0,
            (path[i - 1].y + path[i + 1].y) / 2.0,
        );
        let target = Point::new(
            path[i].x + (mid.x - path[i].x) * SMOOTHING,
            path[i].y + (mid.y - path[i].y) * SMOOTHING,
        );
        if obstacles.shortfall_at(path, i, target) <= obstacles.shortfall(path, i) {
            try_move(path, i, target, obstacles);
        }
    }
}

/// Whether two segments cross away from a shared endpoint.
pub(super) fn proper_crossing(a: Point, b: Point, p: Point, q: Point) -> bool {
    if a.x.min(b.x) > p.x.max(q.x)
        || p.x.min(q.x) > a.x.max(b.x)
        || a.y.min(b.y) > p.y.max(q.y)
        || p.y.min(q.y) > a.y.max(b.y)
    {
        return false;
    }
    let touch = |x: Point, y: Point| x.distance_to(&y) < 1e-9;
    if touch(a, p) || touch(a, q) || touch(b, p) || touch(b, q) {
        return false;
    }
    segments_intersect(&a, &b, &p, &q)
}

/// Unit vector from `from` to `to`, `None` when they coincide.
fn direction(from: Point, to: Point) -> Option<(f64, f64)> {
    let (dx, dy) = (to.x - from.x, to.y - from.y);
    let len = (dx * dx + dy * dy).sqrt();
    (len > 1e-9).then(|| (dx / len, dy / len))
}

/// A unit normal to the chord from `p` to `n`.
fn normal(p: Point, n: Point) -> (f64, f64) {
    direction(p, n).map_or((0.0, 1.0), |(dx, dy)| (dy, -dx))
}

/// Puts the point at `target` unless either adjacent segment would then
/// cross the boundary.
fn try_move(path: &mut [Point], i: usize, target: Point, obstacles: &Obstacles) -> bool {
    if !target.x.is_finite() || !target.y.is_finite() {
        return false;
    }
    if obstacles.crosses(path, i, path[i - 1], target)
        || obstacles.crosses(path, i, target, path[i + 1])
    {
        return false;
    }
    path[i] = target;
    true
}

/// Points about `spacing` apart along the path, the ends kept exactly.
pub(super) fn resample(path: &[Point], spacing: f64) -> Vec<Point> {
    let mut out = vec![path[0]];
    for pair in path.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let pieces = (p.distance_to(&q) / spacing).ceil().max(1.0) as usize;
        for k in 1..=pieces {
            let t = k as f64 / pieces as f64;
            out.push(Point::new(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t));
        }
    }
    out
}
