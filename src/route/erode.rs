//! The face eroded by the clearances, as a constrained triangulation.
//!
//! Every wall of the face is offset to its inside by the line clearance
//! and every node but the move's two ends gets a disc of the node
//! clearance; those curves, the walls and the board rim all go in as
//! constraints, so no triangle straddles a clearance boundary and the
//! centroid of one triangle classifies every triangle it can reach without
//! crossing a constraint. A path through the free triangles keeps every
//! clearance by construction, and the free triangles disconnecting the two
//! ends is what "no room" means.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use spade::handles::{FixedFaceHandle, FixedVertexHandle, InnerTag};
use spade::{ConstrainedDelaunayTriangulation, Point2, PositionInTriangulation, Triangulation};

use super::cross;
use super::Unroutable;
use crate::curve;
use crate::embedding::half_edge_path;
use crate::face::{Component, FaceId, HalfEdgeId, Move as FaceMove, NodeId};
use crate::geometry::{closest_point_on_polyline, closest_point_on_segment};
use crate::types::{GameState, Point};

type Cdt = ConstrainedDelaunayTriangulation<Point2<f64>>;

/// Longest sweep, in radians, of one flat piece of an arc.
const ARC_STEP: f64 = 0.4;
/// Sides of the polygon drawn round a node for its clearance disc.
const DISC_SIDES: usize = 24;

/// Rim subdivision, in clearances, so cuts reach the rim quickly.
const RIM_STEP: f64 = 8.0;
/// How far from an end node, in clearances, its own spokes carry only the
/// flattening margin instead of the line clearance, so a path can run out
/// of a corner narrower than two clearances.
const ENTRY_RING: f64 = 20.0;

/// Where a path leaves an end node: the node and the free triangles at
/// its vertex inside the move's corner.
#[derive(Clone, Debug)]
pub(super) struct Entry {
    pub at: Point,
    pub tris: Vec<usize>,
    /// Which end of the move this is, as an index into the arena's ends.
    pub end: usize,
}

/// An end node of the move and the radii within which its own spokes
/// carry less than the line clearance.
struct End {
    at: Point,
    /// Within this radius a crossing of the node's own spokes changes
    /// nothing, so the part of each spoke inside it carries no clearance.
    inner: f64,
    /// Out to this radius the part of each spoke beyond the inner radius
    /// carries only the fit margin.
    outer: f64,
    /// Radius the middle of the path keeps from the node: the line
    /// clearance, or the inner radius where that is larger.
    guard: f64,
}

/// A spoke of one or both end nodes, with the ends that own it: a line
/// from a node to the new node on it is a spoke of both.
struct OwnSpoke {
    path: Vec<Point>,
    /// The runs of the path beyond the inner radius of every owner, where
    /// the validator would refuse a crossing; none when the owners cover it.
    far: Vec<Vec<Point>>,
    owners: Vec<usize>,
}

/// One cut per boundary piece whose side the move names (and one for the
/// move's own piece when a loop's orientation is free): the edges of a
/// vertex chain from the piece to the rim, each carrying the piece's bit.
pub(super) struct Cuts {
    /// Bit mask of every undirected edge, by edge index.
    pub bits: Vec<u64>,
    /// Every bit in use.
    pub all: u64,
    /// Bit of the pieces whose side the move names, in component order.
    pub piece_bits: Vec<(Component, u64)>,
    /// Bit tracking which way round the move's own piece a loop runs.
    pub own_bit: u64,
}

/// A fence's run through one triangle: the two edges whose midpoints its
/// segment joins, and the vertex they share, which the segment cuts off
/// from the rest of the triangle.
#[derive(Clone, Copy)]
pub(super) struct Segment {
    pub edges: [usize; 2],
    pub shared: FixedVertexHandle,
}

/// An edge crossed by a fence, with its two vertices.
type Crossing = (usize, [FixedVertexHandle; 2]);

/// The fence segment in each triangle, by face index, where one runs.
pub(super) struct Fences {
    pub segments: Vec<Option<Segment>>,
}

impl Fences {
    /// No fences at all.
    pub(super) fn none() -> Self {
        Self {
            segments: Vec::new(),
        }
    }

    /// The fence running through triangle `t`, if any.
    pub(super) fn at(&self, t: usize) -> Option<Segment> {
        self.segments.get(t).copied().flatten()
    }
}

pub(super) struct Arena {
    cdt: Cdt,
    /// Whether each triangle, by face index, keeps every clearance.
    free: Vec<bool>,
    /// Bit per end, by face index: the triangle lies within that end's
    /// guard radius, where only a path starting or finishing there may go.
    guard: Vec<u8>,
    /// Handle of each triangle, by face index.
    handles: Vec<Option<FixedFaceHandle<InnerTag>>>,
    /// Whether each undirected edge, by index, lies on a wall or the rim.
    wall_edge: Vec<bool>,
    walls: Walls,
    /// The walls that are no end node's spoke.
    other_walls: Walls,
    /// Centres of every node the path keeps the node clearance from.
    nodes: Vec<Point>,
    node_clearance: f64,
    ends: Vec<End>,
    spokes: Vec<OwnSpoke>,
    /// The line clearance plus the fit tolerance.
    clearance: f64,
    /// Two fit tolerances, the least the path may come to the part of an
    /// end node's own spoke beyond the forgiven radius.
    margin: f64,
    rim: Vec<FixedVertexHandle>,
}

/// How far off a wall the midpoint of an edge on it may compute.
const ON_WALL: f64 = 1e-6;
/// Cells per side of the wall index.
const GRID: usize = 32;

/// The wall segments of the face and the rim, bucketed on a grid for the
/// distance queries the erosion makes by the thousand.
struct Walls {
    segments: Vec<(Point, Point)>,
    cells: Vec<Vec<usize>>,
    cell: f64,
}

impl Walls {
    fn new(segments: Vec<(Point, Point)>, board: f64) -> Self {
        let cell = board / GRID as f64;
        let mut cells = vec![Vec::new(); GRID * GRID];
        let at = |x: f64| ((x / cell).floor().max(0.0) as usize).min(GRID - 1);
        for (i, (a, b)) in segments.iter().enumerate() {
            for cy in at(a.y.min(b.y))..=at(a.y.max(b.y)) {
                for cx in at(a.x.min(b.x))..=at(a.x.max(b.x)) {
                    cells[cy * GRID + cx].push(i);
                }
            }
        }
        Self {
            segments,
            cells,
            cell,
        }
    }

    /// Distance from `p` to the nearest wall, exact when under `within`
    /// and at least `within` otherwise.
    fn distance(&self, p: Point, within: f64) -> f64 {
        let at = |x: f64| ((x / self.cell).floor().max(0.0) as usize).min(GRID - 1);
        let mut best = f64::INFINITY;
        for cy in at(p.y - within)..=at(p.y + within) {
            for cx in at(p.x - within)..=at(p.x + within) {
                for &i in &self.cells[cy * GRID + cx] {
                    let (a, b) = &self.segments[i];
                    best = best.min(closest_point_on_segment(a, b, &p).1);
                }
            }
        }
        best.max(if best < within { best } else { within })
    }
}

/// Two points closer than this are one vertex of the arrangement.
const SAME_POINT: f64 = 1e-7;

/// The constraint segments of an erosion, split where they cross or meet
/// before they reach the triangulation, so no constraint has to be split
/// by the triangulation itself; its splitting mistakes a cluster of
/// nearly coincident vertices for one and stops. A constraint that still
/// cannot be added is dropped, which at worst leaves a sliver of a
/// clearance curve unmarked.
struct Arrangement {
    segments: Vec<(Point, Point, bool)>,
    /// Points that must be vertices on their own: bare end nodes.
    points: Vec<Point>,
    /// Endpoint bits of every segment, lower endpoint first, so a leaf
    /// spoke walked out and back goes in once.
    seen: BTreeSet<[u64; 4]>,
    cells: Vec<Vec<usize>>,
    cell: f64,
}

impl Arrangement {
    fn new(board: f64) -> Self {
        Self {
            segments: Vec::new(),
            points: Vec::new(),
            seen: BTreeSet::new(),
            cells: vec![Vec::new(); GRID * GRID],
            cell: board / GRID as f64,
        }
    }

    fn cell_at(&self, x: f64) -> usize {
        ((x / self.cell).floor().max(0.0) as usize).min(GRID - 1)
    }

    fn add_closed(&mut self, pts: &[Point], wall: bool) {
        for i in 0..pts.len() {
            let (p, q) = (
                crate::exact::snap_to_grid(pts[i]),
                crate::exact::snap_to_grid(pts[(i + 1) % pts.len()]),
            );
            if p.distance_to(&q) < SAME_POINT {
                continue;
            }
            let mut key = [p.x.to_bits(), p.y.to_bits(), q.x.to_bits(), q.y.to_bits()];
            if (q.x, q.y) < (p.x, p.y) {
                key.rotate_left(2);
            }
            if !self.seen.insert(key) {
                continue;
            }
            let index = self.segments.len();
            self.segments.push((p, q, wall));
            for cy in self.cell_at(p.y.min(q.y))..=self.cell_at(p.y.max(q.y)) {
                for cx in self.cell_at(p.x.min(q.x))..=self.cell_at(p.x.max(q.x)) {
                    self.cells[cy * GRID + cx].push(index);
                }
            }
        }
    }

    fn walls(&self) -> Vec<(Point, Point)> {
        self.segments
            .iter()
            .filter(|s| s.2)
            .map(|s| (s.0, s.1))
            .collect()
    }

    /// Where every segment is cut: its crossings with others and the
    /// endpoints of others lying on it, as parameters along it.
    fn cuts(&self) -> Vec<Vec<(f64, Point)>> {
        let mut cuts = vec![Vec::new(); self.segments.len()];
        let along = |(p, q): (Point, Point), x: Point| {
            let (dx, dy) = (q.x - p.x, q.y - p.y);
            ((x.x - p.x) * dx + (x.y - p.y) * dy) / (dx * dx + dy * dy)
        };
        for cell in &self.cells {
            for (k, &i) in cell.iter().enumerate() {
                for &j in &cell[k + 1..] {
                    let (a, b, _) = self.segments[i];
                    let (c, d, _) = self.segments[j];
                    // An endpoint on the other segment cuts it at that very
                    // point, and a crossing cuts both at one shared point,
                    // so the pieces meet at one vertex.
                    for (s, from, to) in [(i, c, d), (j, a, b)] {
                        let (p, q, _) = self.segments[s];
                        for x in [from, to] {
                            let (_, dist) = closest_point_on_segment(&p, &q, &x);
                            if dist < SAME_POINT {
                                let t = along((p, q), x);
                                if t > 0.0 && t < 1.0 {
                                    cuts[s].push((t, x));
                                }
                            }
                        }
                    }
                    if let Some(x) = crate::geometry::segment_intersection(&a, &b, &c, &d) {
                        // A crossing at an endpoint is the endpoint's own cut
                        // above, placed exactly; the computed one would sit a
                        // rounding error off it.
                        let at_end = [a, b, c, d].iter().any(|e| e.distance_to(&x) < SAME_POINT);
                        let (ta, tb) = (along((a, b), x), along((c, d), x));
                        if !at_end && ta > 0.0 && ta < 1.0 && tb > 0.0 && tb < 1.0 {
                            cuts[i].push((ta, x));
                            cuts[j].push((tb, x));
                        }
                    }
                }
            }
        }
        cuts
    }

    /// The triangulation with every piece of every segment as a
    /// constraint, and which undirected edges lie on walls.
    fn triangulate(self) -> Result<(Cdt, Vec<bool>), Unroutable> {
        let cuts = self.cuts();
        let mut cdt = Cdt::new();
        let mut snap = Snap::default();
        let bug = |_| Unroutable::Bug("triangulation");
        let mut pieces: Vec<(FixedVertexHandle, FixedVertexHandle, bool)> = Vec::new();
        for (s, &(p, q, wall)) in self.segments.iter().enumerate() {
            let len = p.distance_to(&q);
            // Cuts at the ends are the ends, exactly, so no vertex lands a
            // rounding error off an endpoint another segment shares.
            let mut ts: Vec<(f64, Point)> = cuts[s]
                .iter()
                .copied()
                .filter(|(t, _)| t * len > SAME_POINT && (1.0 - t) * len > SAME_POINT)
                .collect();
            ts.sort_by(|x, y| x.0.total_cmp(&y.0));
            ts.dedup_by(|x, y| (x.0 - y.0).abs() * len < SAME_POINT);
            let mut handles = Vec::with_capacity(ts.len() + 2);
            handles.push(cdt.insert(point2(snap.at(p))).map_err(bug)?);
            for (_, x) in ts {
                handles.push(cdt.insert(point2(snap.at(x))).map_err(bug)?);
            }
            handles.push(cdt.insert(point2(snap.at(q))).map_err(bug)?);
            for w in handles.windows(2) {
                if w[0] != w[1] {
                    pieces.push((w[0], w[1], wall));
                }
            }
        }
        for p in &self.points {
            cdt.insert(point2(snap.at(*p))).map_err(bug)?;
        }
        for &(u, v, _) in &pieces {
            if cdt.can_add_constraint(u, v) {
                cdt.add_constraint(u, v);
            } else {
                // A crossing the arrangement missed, at an angle too shallow
                // for its intersection test; the triangulation splits it.
                cdt.add_constraint_and_split(u, v, |p| p);
            }
        }
        // An uncovered wall piece would let the free region through.
        for &(u, v, wall) in &pieces {
            let covered = spade::LineIntersectionIterator::new_from_handles(&cdt, u, v).all(
                |hit| match hit {
                    spade::Intersection::EdgeOverlap(e) => e.is_constraint_edge(),
                    spade::Intersection::VertexIntersection(_) => true,
                    spade::Intersection::EdgeIntersection(_) => false,
                },
            );
            if !covered {
                return Err(Unroutable::Bug(if wall {
                    "wall gap"
                } else {
                    "clearance gap"
                }));
            }
        }
        // Wall pieces may have been split further, so the wall edges are
        // read off the geometry.
        let walls = Walls::new(self.walls(), GRID as f64 * self.cell);
        let mut wall_edge = vec![false; cdt.num_undirected_edges()];
        for e in cdt.undirected_edges() {
            if !e.is_constraint_edge() {
                continue;
            }
            let [p, q] = e.positions().map(point);
            let mid = Point::new((p.x + q.x) / 2.0, (p.y + q.y) / 2.0);
            if walls.distance(mid, ON_WALL) < ON_WALL {
                wall_edge[e.fix().index()] = true;
            }
        }
        Ok((cdt, wall_edge))
    }
}

/// Points already in the triangulation; one computed a rounding error off
/// another becomes that vertex, since the splitting cannot tell them apart.
#[derive(Default)]
struct Snap {
    cells: BTreeMap<(i64, i64), Vec<Point>>,
}

impl Snap {
    fn at(&mut self, p: Point) -> Point {
        let (cx, cy) = (
            (p.x / SAME_POINT).floor() as i64,
            (p.y / SAME_POINT).floor() as i64,
        );
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(near) = self.cells.get(&(cx + dx, cy + dy)) {
                    if let Some(q) = near.iter().find(|q| q.distance_to(&p) < SAME_POINT) {
                        return *q;
                    }
                }
            }
        }
        self.cells.entry((cx, cy)).or_default().push(p);
        p
    }
}

pub(super) fn point(p: Point2<f64>) -> Point {
    Point::new(p.x, p.y)
}

fn point2(p: Point) -> Point2<f64> {
    Point2::new(p.x, p.y)
}

/// Largest displacement of a clearance curve vertex, in board units.
const JITTER: f64 = 1e-3;

/// The `k`th vertex of a clearance curve moved a hair by `k` alone: a line
/// drawn a clearance from a wall offsets back onto it otherwise.
fn jitter(p: Point, k: usize) -> Point {
    let t = k as f64;
    Point::new(
        p.x + JITTER * (t * 1.618_033_988_749_895).sin(),
        p.y + JITTER * (t * 2.414_213_562_373_095).cos(),
    )
}

fn unit(from: Point, to: Point) -> Option<(f64, f64)> {
    let (dx, dy) = (to.x - from.x, to.y - from.y);
    let len = (dx * dx + dy * dy).sqrt();
    (len > 1e-9).then(|| (dx / len, dy / len))
}

/// Screen-left normal of a heading, where y grows downward.
fn left_of((dx, dy): (f64, f64)) -> (f64, f64) {
    (dy, -dx)
}

/// Drops repeated points and, when closed, a last point equal to the first.
fn tidy(mut pts: Vec<Point>) -> Vec<Point> {
    pts.dedup_by(|p, q| p.distance_to(q) < 1e-9);
    while pts.len() > 1 && pts[0].distance_to(pts.last().unwrap()) < 1e-9 {
        pts.pop();
    }
    pts
}

/// The closed polygon of a boundary walk, offset by `c` to its left. The
/// walk has its face on the left, so a left turn is an inside corner, met
/// with a mitre, and a right turn a corner the region wraps, met with an
/// arc round the vertex.
fn offset_closed(poly: &[Point], c: f64) -> Vec<Point> {
    let n = poly.len();
    let mut out = Vec::with_capacity(n * 2);
    for i in 0..n {
        let prev = poly[(i + n - 1) % n];
        let v = poly[i];
        let next = poly[(i + 1) % n];
        let (Some(d1), Some(d2)) = (unit(prev, v), unit(v, next)) else {
            continue;
        };
        let n1 = left_of(d1);
        let n2 = left_of(d2);
        let turning = d1.0 * d2.1 - d1.1 * d2.0;
        // The walk doubles back at a leaf, and the region wraps the tip.
        let doubles_back = d1.0 * d2.0 + d1.1 * d2.1 < -0.999;
        if turning < -1e-9 && !doubles_back {
            // Left turn: the offset lines meet at the mitre.
            let (mx, my) = (n1.0 + n2.0, n1.1 + n2.1);
            let len = (mx * mx + my * my).sqrt();
            if len < 1e-9 {
                out.push(Point::new(v.x + n1.0 * c, v.y + n1.1 * c));
                continue;
            }
            let (mx, my) = (mx / len, my / len);
            // The mitre is exact however sharp the corner, so the two offset
            // lines meet and the cell beyond them is wholly clear.
            let along = c / (mx * n1.0 + my * n1.1);
            out.push(Point::new(v.x + mx * along, v.y + my * along));
        } else if turning > 1e-9 || doubles_back {
            let from = n1.1.atan2(n1.0);
            let to = n2.1.atan2(n2.0);
            let sweep = (to - from).rem_euclid(std::f64::consts::TAU);
            let steps = (sweep / ARC_STEP).ceil().max(1.0) as usize;
            let radius = c / (sweep / steps as f64 / 2.0).cos();
            for k in 0..=steps {
                let t = from + sweep * k as f64 / steps as f64;
                out.push(Point::new(v.x + radius * t.cos(), v.y + radius * t.sin()));
            }
        } else {
            out.push(Point::new(v.x + n1.0 * c, v.y + n1.1 * c));
        }
    }
    tidy(out)
}

/// A polygon round `centre` that contains the disc of `radius`.
fn disc(centre: Point, radius: f64) -> Vec<Point> {
    let step = std::f64::consts::TAU / DISC_SIDES as f64;
    ring(centre, radius / (step / 2.0).cos())
}

/// A polygon round `centre` with its vertices on the circle of `radius`,
/// so a test against the radius reads its vertices as on it.
fn ring(centre: Point, radius: f64) -> Vec<Point> {
    let step = std::f64::consts::TAU / DISC_SIDES as f64;
    (0..DISC_SIDES)
        .map(|k| {
            let t = step * k as f64;
            jitter(
                Point::new(centre.x + radius * t.cos(), centre.y + radius * t.sin()),
                k,
            )
        })
        .collect()
}

/// Points along the closed polygon through `corners`, at most `step` apart.
fn subdivide_closed(corners: &[Point], step: f64) -> Vec<Point> {
    let mut out = Vec::new();
    let n = corners.len();
    for i in 0..n {
        let (p, q) = (corners[i], corners[(i + 1) % n]);
        let pieces = (p.distance_to(&q) / step).ceil().max(1.0) as usize;
        for k in 0..pieces {
            let t = k as f64 / pieces as f64;
            out.push(Point::new(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t));
        }
    }
    out
}

/// The path of half-edge `h` with the node positions as its ends; a drawn
/// stroke ends within the validator's tolerance of its node.
fn spoke_path(state: &GameState, h: HalfEdgeId) -> Vec<Point> {
    let model = &state.face;
    let mut path = half_edge_path(state, h);
    if let Some(first) = path.first_mut() {
        *first = state.nodes[model.origin(h)].position;
    }
    if let Some(last) = path.last_mut() {
        *last = state.nodes[model.head(h)].position;
    }
    path
}

/// The runs of `path` outside `radius` of every point of `ends`, each cut
/// exactly where the path crosses that radius, so a path that comes back
/// inside and leaves again gives two runs. A straight spoke between the
/// two ends has both its vertices inside, so the cuts are found on the
/// segments, not at the vertices.
pub(super) fn beyond(path: &[Point], ends: &[Point], radius: f64) -> Vec<Vec<Point>> {
    let Some(&last) = path.last() else {
        return Vec::new();
    };
    let mut pts: Vec<Point> = Vec::new();
    for w in path.windows(2) {
        let (p, q) = (w[0], w[1]);
        pts.push(p);
        let mut ts: Vec<f64> = ends
            .iter()
            .flat_map(|e| radius_crossings(p, q, *e, radius))
            .filter(|t| *t > 0.0 && *t < 1.0)
            .collect();
        ts.sort_by(f64::total_cmp);
        for t in ts {
            pts.push(crate::exact::snap_to_grid(Point::new(
                p.x + (q.x - p.x) * t,
                p.y + (q.y - p.y) * t,
            )));
        }
    }
    pts.push(last);
    // Between two crossings a segment is inside or outside throughout,
    // so its midpoint says which.
    let outside = |i: usize| {
        let (p, q) = (pts[i], pts[i + 1]);
        let mid = Point::new((p.x + q.x) / 2.0, (p.y + q.y) / 2.0);
        ends.iter().all(|e| e.distance_to(&mid) >= radius)
    };
    let mut runs: Vec<Vec<Point>> = Vec::new();
    let mut open = false;
    for i in 0..pts.len().saturating_sub(1) {
        if !outside(i) {
            open = false;
            continue;
        }
        if !open {
            runs.push(vec![pts[i]]);
            open = true;
        }
        runs.last_mut().expect("an open run").push(pts[i + 1]);
    }
    runs
}

/// Where the segment from `p` to `q` crosses the circle of `radius` round
/// `centre`, as parameters along it.
fn radius_crossings(p: Point, q: Point, centre: Point, radius: f64) -> Vec<f64> {
    let (dx, dy) = (q.x - p.x, q.y - p.y);
    let (fx, fy) = (p.x - centre.x, p.y - centre.y);
    let a = dx * dx + dy * dy;
    let b = 2.0 * (fx * dx + fy * dy);
    let c = fx * fx + fy * fy - radius * radius;
    let disc = b * b - 4.0 * a * c;
    if a == 0.0 || disc <= 0.0 {
        return Vec::new();
    }
    let root = disc.sqrt();
    vec![(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]
}

/// Whether the direction `d` lies inside the corner swept from spoke
/// direction `from` to spoke direction `to` the way the model orders a
/// node's spokes, by cross products alone. The model measures its turns
/// with y up, so its sweep runs clockwise on screen, where y grows down.
fn in_corner(from: Point, to: Point, d: Point) -> bool {
    let o = Point::new(0.0, 0.0);
    let turn = cross(o, to, from);
    let after_from = cross(o, d, from) > 0.0;
    let before_to = cross(o, to, d) > 0.0;
    if turn > 0.0 {
        after_from && before_to
    } else {
        after_from || before_to
    }
}

/// The closed polyline of a cycle component, every half-edge from origin
/// to head with the node positions at the joins.
fn cycle_polygon(state: &GameState, start: HalfEdgeId) -> Vec<Point> {
    let mut out = Vec::new();
    for h in state.face.cycle(start) {
        let mut path = spoke_path(state, h);
        path.pop();
        out.extend(path);
    }
    tidy(out)
}

impl Arena {
    /// Erodes `face` for a move between `a` and `b`: walls offset by `c`,
    /// every other node cut out to `node_clearance`, and each end node's
    /// own spokes carrying no clearance on their part within `forgiven` of
    /// it and only the fit margin out to the entry ring, so a chord can
    /// leave a corner as narrow as the margin lets it at the forgiven
    /// radius.
    pub(super) fn build(
        state: &GameState,
        face: FaceId,
        a: NodeId,
        b: NodeId,
        c: f64,
        node_clearance: f64,
        forgiven: f64,
    ) -> Result<Self, Unroutable> {
        let edge = state.board_size as f64;
        let corners = [
            Point::new(0.0, 0.0),
            Point::new(edge, 0.0),
            Point::new(edge, edge),
            Point::new(0.0, edge),
        ];
        let mut arrangement = Arrangement::new(edge);
        let model = &state.face;
        let components = model.components(face);
        // The fit and the flattening each move the drawn line a tolerance
        // off the path, and the validator reads line flattenings exactly.
        let margin = 2.0 * curve::tolerance(state.board_size);
        let inset: Vec<Point> = [
            Point::new(c, c),
            Point::new(edge - c, c),
            Point::new(edge - c, edge - c),
            Point::new(c, edge - c),
        ]
        .into_iter()
        .enumerate()
        .map(|(k, p)| jitter(p, k))
        .collect();
        arrangement.add_closed(&inset, false);
        let end_nodes: Vec<NodeId> = if a == b { vec![a] } else { vec![a, b] };
        let touches_end = |start: HalfEdgeId| {
            model
                .cycle(start)
                .into_iter()
                .any(|h| end_nodes.contains(&model.origin(h)))
        };
        let mut polygons = Vec::new();
        for comp in components {
            if let Component::Cycle(start) = *comp {
                let poly = cycle_polygon(state, start);
                if poly.len() >= 2 {
                    for (k, offset) in [c, margin].into_iter().enumerate() {
                        if k == 1 && !touches_end(start) {
                            continue;
                        }
                        let curve: Vec<Point> = offset_closed(&poly, offset)
                            .into_iter()
                            .enumerate()
                            .map(|(k, p)| jitter(p, k))
                            .collect();
                        arrangement.add_closed(&curve, false);
                    }
                    polygons.push(poly);
                }
            }
        }
        let nodes: Vec<Point> = state
            .nodes
            .iter()
            .filter(|n| n.id != a && n.id != b)
            .map(|n| n.position)
            .collect();
        for n in &nodes {
            arrangement.add_closed(&disc(*n, node_clearance), false);
        }
        let ends: Vec<End> = end_nodes
            .iter()
            .map(|&v| End {
                at: state.nodes[v].position,
                inner: forgiven,
                outer: ENTRY_RING * c,
                guard: c.max(forgiven),
            })
            .collect();
        let mut spokes: Vec<OwnSpoke> = Vec::new();
        for (k, &v) in end_nodes.iter().enumerate() {
            for &h in model.rotation(v) {
                let path = spoke_path(state, h);
                let back: Vec<Point> = path.iter().rev().copied().collect();
                // A line from one end to the other is the same spoke seen
                // from both, walked the other way.
                match spokes.iter_mut().find(|s| s.path == path || s.path == back) {
                    Some(shared) => {
                        if !shared.owners.contains(&k) {
                            shared.owners.push(k);
                        }
                    }
                    None => spokes.push(OwnSpoke {
                        path,
                        far: Vec::new(),
                        owners: vec![k],
                    }),
                }
            }
            if model.rotation(v).is_empty() {
                arrangement.points.push(state.nodes[v].position);
            }
        }
        for spoke in &mut spokes {
            let owners: Vec<Point> = spoke.owners.iter().map(|&k| ends[k].at).collect();
            spoke.far = beyond(&spoke.path, &owners, forgiven);
            // The margin band round a far run ends in a half disc at each
            // cut, and the triangles need its edge to stop at.
            for run in &spoke.far {
                for (cut, whole) in [
                    (run.first(), spoke.path.first()),
                    (run.last(), spoke.path.last()),
                ] {
                    if let Some(&cut) = cut {
                        if Some(&cut) != whole {
                            arrangement.add_closed(&disc(cut, margin), false);
                        }
                    }
                }
            }
        }
        for end in &ends {
            arrangement.add_closed(&ring(end.at, end.inner), false);
            if end.guard > end.inner {
                arrangement.add_closed(&ring(end.at, end.guard), false);
            }
            arrangement.add_closed(&ring(end.at, end.outer), false);
        }
        let rim_pts = subdivide_closed(&corners, RIM_STEP * c);
        arrangement.add_closed(&rim_pts, true);
        for poly in &polygons {
            arrangement.add_closed(poly, true);
        }
        let all_walls = arrangement.walls();
        let on_spoke = |(u, v): &(Point, Point)| {
            let mid = Point::new((u.x + v.x) / 2.0, (u.y + v.y) / 2.0);
            spokes
                .iter()
                .any(|spoke| closest_point_on_polyline(&spoke.path, &mid).1 < SAME_POINT * 1e3)
        };
        let other: Vec<(Point, Point)> =
            all_walls.iter().copied().filter(|w| !on_spoke(w)).collect();
        let walls = Walls::new(all_walls, edge);
        let other_walls = Walls::new(other, edge);
        let (cdt, wall_edge) = arrangement.triangulate()?;
        let rim = cdt.convex_hull().map(|e| e.from().fix()).collect();

        let mut arena = Self {
            cdt,
            free: Vec::new(),
            guard: Vec::new(),
            handles: Vec::new(),
            wall_edge,
            walls,
            other_walls,
            nodes,
            node_clearance,
            ends,
            spokes,
            clearance: c,
            margin,
            rim,
        };
        arena.classify();
        Ok(arena)
    }

    /// Whether `p` keeps every clearance: the line clearance from walls that
    /// are no end node's spoke, the node clearance from every other node,
    /// and from each end node's own spokes the least that any owning end
    /// asks at `p`: the fit margin from the spoke's far runs within its
    /// entry ring, the clearance from the whole spoke beyond.
    fn clear_at(&self, p: Point, slack: f64) -> bool {
        let c = self.clearance;
        if self.other_walls.distance(p, c) < c - slack {
            return false;
        }
        if self
            .nodes
            .iter()
            .any(|n| n.distance_to(&p) < self.node_clearance - slack)
        {
            return false;
        }
        self.spokes.iter().all(|spoke| {
            let entering = spoke.owners.iter().any(|&k| {
                let end = &self.ends[k];
                end.at.distance_to(&p) < end.outer + slack
            });
            if entering {
                spoke
                    .far
                    .iter()
                    .all(|run| closest_point_on_polyline(run, &p).1 >= self.margin - slack)
            } else {
                closest_point_on_polyline(&spoke.path, &p).1 >= c - slack
            }
        })
    }

    /// Marks the free triangles: one whose centroid and three vertices all
    /// keep the clearances. The vertices sit on the clearance curves a
    /// jitter off them, so they get that much slack; a triangle reaching
    /// past a curve has a vertex well inside and fails.
    fn classify(&mut self) {
        let n = self.cdt.num_all_faces();
        let mut free = vec![false; n];
        let mut guard = vec![0u8; n];
        let mut handles = vec![None; n];
        for t in self.cdt.inner_faces() {
            let fixed = t.fix();
            handles[fixed.index()] = Some(fixed);
            let centroid = self.centroid(fixed);
            let clear = self.clear_at(centroid, 1e-7)
                && t.positions()
                    .into_iter()
                    .all(|p| self.clear_at(point(p), 2.0 * JITTER));
            free[fixed.index()] = clear;
            for (k, end) in self.ends.iter().enumerate() {
                if end.at.distance_to(&centroid) < end.guard {
                    guard[fixed.index()] |= 1 << k;
                }
            }
        }
        self.free = free;
        self.guard = guard;
        self.handles = handles;
    }

    fn centroid(&self, t: FixedFaceHandle<InnerTag>) -> Point {
        let [p, q, r] = self.cdt.face(t).positions();
        Point::new((p.x + q.x + r.x) / 3.0, (p.y + q.y + r.y) / 3.0)
    }

    /// Whether the triangle holding `p` is free; a point on an edge or a
    /// vertex is free only if every triangle there is.
    pub(super) fn free_at(&self, p: Point) -> bool {
        let inner =
            |f: spade::handles::FaceHandle<'_, spade::handles::PossiblyOuterTag, _, _, _, _>| {
                f.as_inner().is_some_and(|f| self.free[f.fix().index()])
            };
        match self.cdt.locate(point2(p)) {
            PositionInTriangulation::OnFace(f) => self.free[f.index()],
            PositionInTriangulation::OnEdge(e) => {
                let e = self.cdt.directed_edge(e);
                inner(e.face()) && inner(e.rev().face())
            }
            PositionInTriangulation::OnVertex(v) => {
                self.cdt.vertex(v).out_edges().all(|e| inner(e.face()))
            }
            _ => false,
        }
    }

    /// Whether `p` lies in the triangulation at all.
    #[cfg(test)]
    pub(super) fn contains(&self, p: Point) -> bool {
        !matches!(
            self.cdt.locate(point2(p)),
            PositionInTriangulation::OutsideOfConvexHull(_)
                | PositionInTriangulation::NoTriangulation
        )
    }

    /// Triangles by face index, free or not.
    pub(super) fn triangle_count(&self) -> usize {
        self.free.len()
    }

    /// Bit per end: whether triangle `t` lies within that end's guard.
    pub(super) fn guard(&self, t: usize) -> u8 {
        self.guard[t]
    }

    pub(super) fn walls(&self) -> &[(Point, Point)] {
        &self.walls.segments
    }

    /// Every vertex of the triangulation, in insertion order.
    #[cfg(test)]
    pub(super) fn vertex_points(&self) -> Vec<Point> {
        self.cdt.vertices().map(|v| point(v.position())).collect()
    }

    pub(super) fn vertex_point(&self, v: FixedVertexHandle) -> Point {
        point(self.cdt.vertex(v).position())
    }

    /// Vertices of a triangle, counterclockwise on screen, so each edge in
    /// turn has the triangle on its screen-left. spade keeps its faces
    /// counterclockwise with y up, which is clockwise on screen.
    pub(super) fn ccw_vertices(&self, t: usize) -> [FixedVertexHandle; 3] {
        let [a, b, c] = self
            .cdt
            .face(self.handles[t].expect("a triangle"))
            .vertices()
            .map(|v| v.fix());
        [a, c, b]
    }

    /// The free triangle across the edge `p`, `q` from triangle `t`, when
    /// the edge is not a wall.
    pub(super) fn across(
        &self,
        t: usize,
        p: FixedVertexHandle,
        q: FixedVertexHandle,
    ) -> Option<(usize, usize)> {
        let e = self.cdt.get_edge_from_neighbors(p, q)?;
        let index = e.as_undirected().fix().index();
        if self.wall_edge[index] {
            return None;
        }
        let other = [e.face(), e.rev().face()]
            .into_iter()
            .filter_map(|f| f.as_inner().map(|f| f.fix().index()))
            .find(|&f| f != t)?;
        self.free[other].then_some((other, index))
    }

    /// Free triangles with `v` as a vertex.
    fn free_around(&self, v: FixedVertexHandle) -> Vec<usize> {
        let mut out: Vec<usize> = self
            .cdt
            .vertex(v)
            .out_edges()
            .filter_map(|e| e.face().as_inner())
            .map(|f| f.fix().index())
            .filter(|&t| self.free[t])
            .collect();
        out.sort_unstable();
        out
    }

    /// Where a path leaves node `v` through the corner that follows spoke
    /// `at`: the free triangles at the node's own vertex between that
    /// spoke and the next one round the node, every one of them for a bare
    /// node. Room when there is none.
    pub(super) fn entries(
        &self,
        state: &GameState,
        v: NodeId,
        at: Option<HalfEdgeId>,
    ) -> Result<Entry, Unroutable> {
        let a = state.nodes[v].position;
        let vertex = self.vertex_at(a).ok_or(Unroutable::Bug("end vertex"))?;
        let mut tris = self.free_around(vertex);
        if let Some(h) = at {
            let rotation = state.face.rotation(v);
            let k = rotation.iter().position(|&x| x == h).expect("own spoke");
            let next = rotation[(k + 1) % rotation.len()];
            if next != h {
                let heading = |h: HalfEdgeId| {
                    let path = spoke_path(state, h);
                    Point::new(path[1].x - a.x, path[1].y - a.y)
                };
                let (from, to) = (heading(h), heading(next));
                tris.retain(|&t| {
                    let q = self.centroid(self.handles[t].expect("a triangle"));
                    in_corner(from, to, Point::new(q.x - a.x, q.y - a.y))
                });
            }
        }
        if tris.is_empty() {
            return Err(Unroutable::Room);
        }
        let end = self
            .ends
            .iter()
            .position(|e| e.at == a)
            .ok_or(Unroutable::Bug("end"))?;
        Ok(Entry { at: a, tris, end })
    }

    /// The cut parity of the straight segment from `p` to `q` when it runs
    /// through free triangles across non-wall edges only, else None. A
    /// segment through a vertex is refused as unclear; one along an edge
    /// between two free triangles crosses nothing, as the funnel's own
    /// segments between wrap vertices do. When `from` or `to` is an end,
    /// the first triangle the segment enters, or the last it leaves, has
    /// to be one of that end's corner triangles, or the segment would
    /// reach the node from the wrong side of one of its own lines. The
    /// segment stays out of each end's guard except at that end.
    pub(super) fn segment_parity(
        &self,
        cuts: &Cuts,
        p: Point,
        q: Point,
        from: Option<&Entry>,
        to: Option<&Entry>,
    ) -> Option<u64> {
        let mut parity = 0;
        let mut first: Option<[usize; 2]> = None;
        let mut last: Option<[usize; 2]> = None;
        // A shortcut may run inside an end's guard only at that end.
        let allowed: u8 = [from, to]
            .into_iter()
            .flatten()
            .fold(0, |bits, e| bits | 1 << e.end);
        for hit in spade::LineIntersectionIterator::new(&self.cdt, point2(p), point2(q)) {
            let (edge, crossing) = match hit {
                spade::Intersection::EdgeIntersection(e) => (e, true),
                spade::Intersection::EdgeOverlap(e) => (e, false),
                spade::Intersection::VertexIntersection(v) => {
                    let at = point(v.position());
                    if at.distance_to(&p) > SAME_POINT && at.distance_to(&q) > SAME_POINT {
                        return None;
                    }
                    continue;
                }
            };
            let index = edge.as_undirected().fix().index();
            if self.wall_edge[index] {
                return None;
            }
            let faces =
                [edge.face(), edge.rev().face()].map(|f| f.as_inner().map(|f| f.fix().index()));
            let [Some(a), Some(b)] = faces else {
                return None;
            };
            if !self.free[a] || !self.free[b] {
                return None;
            }
            if self.guard[a] & !allowed != 0 || self.guard[b] & !allowed != 0 {
                return None;
            }
            first.get_or_insert([a, b]);
            last = Some([a, b]);
            if crossing {
                parity ^= cuts.bits[index];
            }
        }
        for (end, faces, at) in [(from, first, p), (to, last, q)] {
            let Some(end) = end else { continue };
            let vertex = self.vertex_at(at)?;
            let touching = faces?
                .into_iter()
                .filter(|&t| self.ccw_vertices(t).contains(&vertex))
                .collect::<Vec<_>>();
            if touching.is_empty() || !touching.iter().any(|t| end.tris.contains(t)) {
                return None;
            }
        }
        Some(parity)
    }

    /// The path with every detour the corridor forced on it cut short: any
    /// two points joined by a straight segment through free triangles that
    /// crosses the cuts with the same parity as the path between them. A
    /// shortcut from the entry node or to the exit node has to leave or
    /// arrive through that end's corner.
    pub(super) fn tighten(
        &self,
        cuts: &Cuts,
        path: &[Point],
        entry: &Entry,
        exit: &Entry,
    ) -> Vec<Point> {
        let mut out = vec![path[0]];
        let mut i = 0;
        while i + 1 < path.len() {
            let mut next = i + 1;
            let from = (i == 0).then_some(entry);
            let last = |j: usize| (j + 1 == path.len()).then_some(exit);
            let mut along = self.segment_parity(cuts, path[i], path[i + 1], from, last(i + 1));
            for j in i + 2..path.len() {
                along = match (
                    along,
                    self.segment_parity(cuts, path[j - 1], path[j], None, last(j)),
                ) {
                    (Some(a), Some(b)) => Some(a ^ b),
                    _ => None,
                };
                let Some(along) = along else { break };
                if self.segment_parity(cuts, path[i], path[j], from, last(j)) == Some(along) {
                    next = j;
                }
            }
            out.push(path[next]);
            i = next;
        }
        out
    }

    /// A vertex on the piece, to start its cut from: a node on a cycle,
    /// the nearest vertex to an isolated node.
    fn piece_vertex(&self, state: &GameState, comp: Component) -> Option<FixedVertexHandle> {
        let v = match comp {
            Component::Node(v) => v,
            Component::Cycle(start) => state.face.origin(start),
        };
        let p = state.nodes[v].position;
        match self.cdt.locate(point2(p)) {
            PositionInTriangulation::OnVertex(h) => Some(h),
            PositionInTriangulation::OnFace(f) => self
                .cdt
                .face(f)
                .vertices()
                .iter()
                .map(|x| x.fix())
                .min_by_key(|x| x.index()),
            PositionInTriangulation::OnEdge(e) => Some(self.cdt.directed_edge(e).from().fix()),
            _ => None,
        }
    }

    /// Lays a cut from each piece whose side matters to the rim, avoiding
    /// `blocked` vertices where it can, and one from the move's own piece
    /// when that piece is a cycle: a path round the other side of it turns
    /// the closed walk the other way, which swaps every piece's side.
    pub(super) fn cuts(
        &self,
        state: &GameState,
        mv: &FaceMove,
        face: FaceId,
        blocked: &[FixedVertexHandle],
    ) -> Result<Cuts, Unroutable> {
        let model = &state.face;
        let components = model.components(face);
        let own = model
            .component_of(mv.a, mv.at_a)
            .map_err(|_| Unroutable::Bug("corner"))?;
        let other = model
            .component_of(mv.b, mv.at_b)
            .map_err(|_| Unroutable::Bug("corner"))?;
        let mut cuts = Cuts {
            bits: vec![0; self.cdt.num_undirected_edges()],
            all: 0,
            piece_bits: Vec::new(),
            own_bit: 0,
        };
        if own != other {
            return Ok(cuts);
        }
        let adjacency = self.adjacency();
        // No cut runs through an end's guard, so no parity flips inside
        // the fan or band a path starts or finishes in.
        let mut blocked_flags = vec![false; self.cdt.num_vertices()];
        for v in blocked {
            blocked_flags[v.index()] = true;
        }
        for v in self.cdt.vertices() {
            let p = point(v.position());
            if self
                .ends
                .iter()
                .any(|e| e.at.distance_to(&p) <= e.guard + SAME_POINT)
            {
                blocked_flags[v.fix().index()] = true;
            }
        }
        let mut on_rim = vec![false; self.cdt.num_vertices()];
        for v in &self.rim {
            on_rim[v.index()] = true;
        }
        let mut sources: Vec<(Option<Component>, FixedVertexHandle)> = components
            .iter()
            .filter(|&&c| c != own)
            .map(|&c| (Some(c), self.piece_vertex(state, c)))
            .map(|(c, v)| v.map(|v| (c, v)).ok_or(Unroutable::Bug("piece vertex")))
            .collect::<Result<_, _>>()?;
        if let Component::Cycle(start) = own {
            // The own cut starts at the vertex of the piece farthest from
            // both ends and off every blocked vertex, so the second search
            // cannot satisfy its parity inside an entry fan. The piece is a
            // cycle of flattened lines, so it has vertices off both ends.
            let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
            let room = |p: Point| {
                ends.iter()
                    .map(|e| e.distance_to(&p))
                    .fold(f64::INFINITY, f64::min)
            };
            let mut candidates: Vec<(usize, Point)> = cycle_polygon(state, start)
                .into_iter()
                .enumerate()
                .collect();
            candidates.sort_by(|(i, p), (j, q)| room(*q).total_cmp(&room(*p)).then(i.cmp(j)));
            let v = candidates
                .iter()
                .filter(|(_, p)| room(*p) > SAME_POINT)
                .find_map(|(_, p)| self.vertex_at(*p))
                .filter(|v| !blocked_flags[v.index()])
                .ok_or(Unroutable::Bug("own vertex"))?;
            sources.push((None, v));
        }
        if sources.len() > 64 {
            return Err(Unroutable::Bug("too many pieces"));
        }
        for (bit, (comp, start)) in sources.into_iter().enumerate() {
            let bit = 1u64 << bit;
            let chain = self
                .chain(&adjacency, start, &on_rim, &blocked_flags)
                .or_else(|| {
                    self.chain(
                        &adjacency,
                        start,
                        &on_rim,
                        &vec![false; blocked_flags.len()],
                    )
                })
                .ok_or(Unroutable::Bug("cut"))?;
            for (u, v) in chain {
                let e = self
                    .cdt
                    .get_edge_from_neighbors(u, v)
                    .ok_or(Unroutable::Bug("cut edge"))?;
                cuts.bits[e.as_undirected().fix().index()] |= bit;
            }
            cuts.all |= bit;
            match comp {
                Some(comp) => cuts.piece_bits.push((comp, bit)),
                None => cuts.own_bit = bit,
            }
        }
        Ok(cuts)
    }

    /// Labels every triangle that is not free with its blob, the triangles
    /// it reaches through other unfree ones. A blob is one obstacle to a
    /// path, whatever pieces sit in it.
    fn blobs(&self) -> Vec<usize> {
        let n = self.free.len();
        let mut label = vec![usize::MAX; n];
        let mut next = 0;
        for start in 0..n {
            if self.handles[start].is_none() || self.free[start] || label[start] != usize::MAX {
                continue;
            }
            label[start] = next;
            let mut stack = vec![start];
            while let Some(t) = stack.pop() {
                for e in self
                    .cdt
                    .face(self.handles[t].expect("a triangle"))
                    .adjacent_edges()
                {
                    let Some(f) = e.rev().face().as_inner() else {
                        continue;
                    };
                    let u = f.fix().index();
                    if !self.free[u] && label[u] == usize::MAX {
                        label[u] = next;
                        stack.push(u);
                    }
                }
            }
            next += 1;
        }
        label
    }

    /// The blob a piece's wall sits in.
    fn piece_blob(&self, state: &GameState, comp: Component, blobs: &[usize]) -> Option<usize> {
        let v = self.piece_vertex(state, comp)?;
        self.cdt
            .vertex(v)
            .out_edges()
            .filter_map(|e| e.face().as_inner())
            .map(|f| blobs[f.fix().index()])
            .find(|&b| b != usize::MAX)
    }

    /// The triangle across the edge `p`, `q` of triangle `t`, free or not,
    /// with the edge's index.
    fn neighbour(
        &self,
        t: usize,
        p: FixedVertexHandle,
        q: FixedVertexHandle,
    ) -> Option<(usize, usize)> {
        let e = self.cdt.get_edge_from_neighbors(p, q)?;
        let index = e.as_undirected().fix().index();
        [e.face(), e.rev().face()]
            .into_iter()
            .filter_map(|f| f.as_inner().map(|f| f.fix().index()))
            .find(|&f| f != t)
            .map(|f| (f, index))
    }

    /// Fences a parity search may not cross: one tree of polylines
    /// joining the pieces the move encloses and one joining the pieces it
    /// does not, each polyline running through free triangles off the
    /// ends' guards, from the midpoint of one edge to the midpoint of the
    /// next, so it touches no vertex and glues to nothing but the blobs it
    /// joins. Kept from passing between two pieces of one side, a path
    /// can reach only a few parities, so the search stays small; where a
    /// fence closes the passage a path needs, the search runs again
    /// without them.
    pub(super) fn fences(&self, state: &GameState, mv: &FaceMove, face: FaceId) -> Fences {
        let model = &state.face;
        let own = model.component_of(mv.a, mv.at_a).ok();
        let blobs = self.blobs();
        let mut sides: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
        for &comp in model.components(face) {
            if Some(comp) == own {
                continue;
            }
            let Some(blob) = self.piece_blob(state, comp, &blobs) else {
                continue;
            };
            let side = &mut sides[usize::from(!mv.enclosed.contains(&model.lowest_node(comp)))];
            if !side.contains(&blob) {
                side.push(blob);
            }
        }
        let mut fences = Fences {
            segments: vec![None; self.free.len()],
        };
        let mut on_fence = vec![false; self.cdt.num_undirected_edges()];
        for members in &sides {
            if members.len() >= 2 {
                self.fence_tree(members, &blobs, &mut fences.segments, &mut on_fence);
            }
        }
        fences
    }

    /// Joins the blobs `members` with polylines through free triangles no
    /// fence runs through yet, fewest triangles first, each new polyline
    /// starting at a joined blob's edge or at a midpoint already on the
    /// tree and ending at an edge of a blob not yet joined.
    fn fence_tree(
        &self,
        members: &[usize],
        blobs: &[usize],
        segments: &mut [Option<Segment>],
        on_fence: &mut [bool],
    ) {
        let mut joined: Vec<usize> = vec![members[0]];
        while joined.len() < members.len() {
            let anchored = on_fence.to_vec();
            let reached = self.grow_fence(
                |_, edge, other| {
                    anchored[edge] || (!self.free[other] && joined.contains(&blobs[other]))
                },
                |other| {
                    !self.free[other]
                        && members.contains(&blobs[other])
                        && !joined.contains(&blobs[other])
                },
                segments,
                on_fence,
            );
            let Some(other) = reached else {
                return;
            };
            joined.push(blobs[other]);
        }
    }

    /// One fence polyline through free triangles off the guards that host
    /// none yet, fewest triangles first, from a triangle `anchor` accepts
    /// by one of its edges (the edge the polyline starts at) to an edge
    /// across which `target` accepts the triangle. Returns that triangle.
    fn grow_fence(
        &self,
        anchor: impl Fn(usize, usize, usize) -> bool,
        target: impl Fn(usize) -> bool,
        segments: &mut [Option<Segment>],
        on_fence: &mut [bool],
    ) -> Option<usize> {
        let n = self.free.len();
        let edges = |t: usize| {
            let v = self.ccw_vertices(t);
            (0..3).filter_map(move |k| {
                let (p, q) = (v[k], v[(k + 1) % 3]);
                self.neighbour(t, p, q)
                    .map(|(other, edge)| (edge, other, p, q))
            })
        };
        let hostable = |t: usize, segments: &[Option<Segment>]| {
            self.free[t] && self.guard[t] == 0 && segments[t].is_none()
        };
        let mut parent: Vec<Option<usize>> = vec![None; n];
        let mut entry: Vec<Option<Crossing>> = vec![None; n];
        let mut seen = vec![false; n];
        let mut queue = VecDeque::new();
        for t in (0..n).filter(|&t| self.handles[t].is_some() && hostable(t, segments)) {
            if let Some((edge, _, p, q)) =
                edges(t).find(|&(edge, other, _, _)| anchor(t, edge, other))
            {
                seen[t] = true;
                entry[t] = Some((edge, [p, q]));
                queue.push_back(t);
            }
        }
        let mut found = None;
        'search: while let Some(t) = queue.pop_front() {
            for (edge, other, p, q) in edges(t) {
                if entry[t].is_some_and(|(e, _)| e == edge) {
                    continue;
                }
                if target(other) {
                    found = Some((t, (edge, [p, q]), other));
                    break 'search;
                }
                if seen[other] || !hostable(other, segments) {
                    continue;
                }
                seen[other] = true;
                parent[other] = Some(t);
                entry[other] = Some((edge, [p, q]));
                queue.push_back(other);
            }
        }
        let (mut cur, mut exit, reached) = found?;
        loop {
            let came = entry[cur].expect("an entry edge");
            let shared = came.1.into_iter().find(|v| exit.1.contains(v));
            segments[cur] = Some(Segment {
                edges: [came.0, exit.0],
                shared: shared.expect("edges of one triangle"),
            });
            on_fence[came.0] = true;
            on_fence[exit.0] = true;
            match parent[cur] {
                Some(prev) => {
                    exit = came;
                    cur = prev;
                }
                None => return Some(reached),
            }
        }
    }

    /// The midpoint of every edge, by edge index.
    #[cfg(test)]
    pub(super) fn edge_midpoints(&self) -> Vec<Point> {
        let mut out = vec![Point::new(0.0, 0.0); self.cdt.num_undirected_edges()];
        for e in self.cdt.undirected_edges() {
            let [p, q] = e.positions().map(point);
            out[e.fix().index()] = Point::new((p.x + q.x) / 2.0, (p.y + q.y) / 2.0);
        }
        out
    }

    /// One fence from the triangle holding `from` to the one holding `to`,
    /// for tests that want a wall of their own placing.
    #[cfg(test)]
    pub(super) fn fence_between(&self, from: Point, to: Point) -> Fences {
        let at = |p: Point| match self.cdt.locate(point2(p)) {
            PositionInTriangulation::OnFace(f) => f.index(),
            _ => panic!("{p:?} is not inside a triangle"),
        };
        let (start, end) = (at(from), at(to));
        let mut fences = Fences {
            segments: vec![None; self.free.len()],
        };
        let mut on_fence = vec![false; self.cdt.num_undirected_edges()];
        let reached = self.grow_fence(
            |t, _, _| t == start,
            |other| other == end,
            &mut fences.segments,
            &mut on_fence,
        );
        assert_eq!(reached, Some(end), "no fence runs from {from:?} to {to:?}");
        fences
    }

    fn adjacency(&self) -> Vec<Vec<FixedVertexHandle>> {
        let mut adjacency = vec![Vec::new(); self.cdt.num_vertices()];
        for e in self.cdt.undirected_edges() {
            let [p, q] = e.vertices().map(|v| v.fix());
            adjacency[p.index()].push(q);
            adjacency[q.index()].push(p);
        }
        for list in &mut adjacency {
            list.sort_unstable_by_key(|v| v.index());
        }
        adjacency
    }

    /// Shortest vertex chain from `start` to the rim through no blocked
    /// vertex, as the edges walked.
    fn chain(
        &self,
        adjacency: &[Vec<FixedVertexHandle>],
        start: FixedVertexHandle,
        on_rim: &[bool],
        blocked: &[bool],
    ) -> Option<Vec<(FixedVertexHandle, FixedVertexHandle)>> {
        let mut parent: Vec<Option<FixedVertexHandle>> = vec![None; adjacency.len()];
        let mut seen = vec![false; adjacency.len()];
        let mut queue = VecDeque::new();
        seen[start.index()] = true;
        queue.push_back(start);
        while let Some(v) = queue.pop_front() {
            if on_rim[v.index()] {
                let mut edges = Vec::new();
                let mut cur = v;
                while let Some(prev) = parent[cur.index()] {
                    edges.push((prev, cur));
                    cur = prev;
                }
                edges.reverse();
                return Some(edges);
            }
            for &w in &adjacency[v.index()] {
                if seen[w.index()] || blocked[w.index()] && !on_rim[w.index()] {
                    continue;
                }
                seen[w.index()] = true;
                parent[w.index()] = Some(v);
                queue.push_back(w);
            }
        }
        None
    }

    /// The vertex at `p`, if there is one.
    pub(super) fn vertex_at(&self, p: Point) -> Option<FixedVertexHandle> {
        match self.cdt.locate(point2(p)) {
            PositionInTriangulation::OnVertex(v) => Some(v),
            _ => None,
        }
    }
}
