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

use super::clearance::proper_crossing;
use super::Unroutable;
use crate::curve;
use crate::embedding::{half_edge_path, leaving_angle, turn};
use crate::face::{Component, FaceId, HalfEdgeId, Move as FaceMove, NodeId};
use crate::geometry::{closest_point_on_polyline, closest_point_on_segment};
use crate::types::{scale, GameState, Point, AI_INTERSECTION_TOLERANCE, MIN_PATH_CLEARANCE};

type Cdt = ConstrainedDelaunayTriangulation<Point2<f64>>;

/// Longest sweep, in radians, of one flat piece of an arc.
const ARC_STEP: f64 = 0.4;
/// Sides of the polygon drawn round a node for its clearance disc.
const DISC_SIDES: usize = 24;
/// Longest mitre at a sharp inside corner, in clearances; the wedge past
/// it is thinner than two clearances and holds no free triangle.
const MITRE_LIMIT: f64 = 4.0;
/// Rim subdivision, in clearances, so cuts reach the rim quickly.
const RIM_STEP: f64 = 8.0;
/// How far from an end node, in clearances, its leg may reach into the
/// free region.
const LEG_REACH: f64 = 20.0;
/// Entries kept per end, nearest first.
const LEGS: usize = 4;
/// Least angle, in radians, between a leg and either spoke of its corner,
/// so the embedding reads the leg into the same corner.
const LEG_MARGIN: f64 = 0.2;
/// How far past the radius the embedding reads a heading at a leg has to
/// end, in board units, so the reading lands on the leg's own knot and
/// not on some flattening vertex far along the curve.
const LEG_PAST: f64 = 0.5;

/// Where a path enters the free region from an end node: the point, the
/// free triangles around it, and the leg from the node when the point is
/// not the node itself.
#[derive(Clone, Debug)]
pub(super) struct Entry {
    pub at: Point,
    pub tris: Vec<usize>,
    pub leg: Option<(Point, Point)>,
}

/// One cut per boundary piece whose side the move names (and one for the
/// move's own piece when a loop's orientation is free): the edges of a
/// vertex chain from the piece to the rim, each carrying the piece's bit.
pub(super) struct Cuts {
    /// Bit mask of every undirected edge, by edge index.
    pub bits: Vec<u64>,
    /// The edges as segments, for counting the legs across them.
    pub edges: Vec<(Point, Point, u64)>,
    /// Every bit in use.
    pub all: u64,
    /// Bit of the pieces whose side the move names, in component order.
    pub piece_bits: Vec<(Component, u64)>,
    /// Bit tracking which way round the move's own piece a loop runs.
    pub own_bit: u64,
}

pub(super) struct Arena {
    cdt: Cdt,
    /// Whether each triangle, by face index, keeps every clearance.
    free: Vec<bool>,
    /// Handle of each triangle, by face index.
    handles: Vec<Option<FixedFaceHandle<InnerTag>>>,
    /// Whether each undirected edge, by index, lies on a wall or the rim.
    wall_edge: Vec<bool>,
    walls: Walls,
    /// Centres of every node the path keeps the node clearance from.
    nodes: Vec<Point>,
    node_clearance: f64,
    /// The move's end nodes, which only the legs may come near.
    ends: Vec<Point>,
    end_clearance: f64,
    /// The wall clearance the face was eroded by.
    #[cfg(test)]
    clearance: f64,
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
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
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
        for &(u, v, _) in &pieces {
            if cdt.can_add_constraint(u, v) {
                cdt.add_constraint(u, v);
            } else {
                // A crossing the arrangement missed, at an angle too shallow
                // for its intersection test; the triangulation splits it.
                cdt.add_constraint_and_split(u, v, |p| p);
            }
        }
        // A wall piece the triangulation left uncovered would let the free
        // region leak through it.
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

/// Points already in the triangulation, so a point computed a rounding
/// error off one of them becomes that vertex and not a second one beside
/// it, which the constraint splitting cannot tell apart.
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

/// The `k`th vertex of a clearance curve moved a hair by a function of
/// `k` alone, so a board a rounding error off erodes to the same arena. A
/// drawn line runs exactly a clearance from the walls it hugged, so the
/// next offset of it would land on those walls and break the splitting.
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
            let along = (c / (mx * n1.0 + my * n1.1)).min(MITRE_LIMIT * c);
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
    let r = radius / (step / 2.0).cos();
    (0..DISC_SIDES)
        .map(|k| {
            let t = step * k as f64;
            jitter(
                Point::new(centre.x + r * t.cos(), centre.y + r * t.sin()),
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

/// The path of half-edge `h` from its origin node's position to its head
/// node's. A drawn stroke ends within the validator's tolerance of its
/// node, not on it, so the nodes stand in for the path's ends and every
/// spoke of a node meets at one point.
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
    /// every other node cut out to `node_clearance`.
    pub(super) fn build(
        state: &GameState,
        face: FaceId,
        a: NodeId,
        b: NodeId,
        c: f64,
        node_clearance: f64,
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
        let mut polygons = Vec::new();
        for comp in components {
            if let Component::Cycle(start) = *comp {
                let poly = cycle_polygon(state, start);
                if poly.len() >= 2 {
                    let offset: Vec<Point> = offset_closed(&poly, c)
                        .into_iter()
                        .enumerate()
                        .map(|(k, p)| jitter(p, k))
                        .collect();
                    arrangement.add_closed(&offset, false);
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
        // The validator forgives a crossing of an end node's spoke close to
        // that node, so the path keeps the forgiven distance from its own
        // ends except along its legs, or the fitted curve could cross
        // there and the embedding read a different move.
        let end_clearance = MIN_PATH_CLEARANCE * scale(state.board_size) + LEG_PAST;
        let mut ends = vec![state.nodes[a].position];
        if b != a {
            ends.push(state.nodes[b].position);
        }
        for e in &ends {
            arrangement.add_closed(&disc(*e, end_clearance), false);
        }
        let rim_pts = subdivide_closed(&corners, RIM_STEP * c);
        arrangement.add_closed(&rim_pts, true);
        for poly in &polygons {
            arrangement.add_closed(poly, true);
        }
        let walls = Walls::new(arrangement.walls(), edge);
        let (cdt, wall_edge) = arrangement.triangulate()?;
        let rim = cdt.convex_hull().map(|e| e.from().fix()).collect();

        let mut arena = Self {
            cdt,
            free: Vec::new(),
            handles: Vec::new(),
            wall_edge,
            walls,
            nodes,
            node_clearance,
            ends,
            end_clearance,
            #[cfg(test)]
            clearance: c,
            rim,
        };
        arena.classify(c);
        Ok(arena)
    }

    /// Marks the free triangles: every region bounded by constraints is
    /// wholly clear or wholly not, so one centroid decides each.
    fn classify(&mut self, c: f64) {
        let n = self.cdt.num_all_faces();
        let mut free = vec![false; n];
        let mut seen = vec![false; n];
        let mut handles = vec![None; n];
        for t in self.cdt.inner_faces() {
            let t = t.fix();
            handles[t.index()] = Some(t);
            if seen[t.index()] {
                continue;
            }
            let mut region = Vec::new();
            let mut stack = vec![t];
            seen[t.index()] = true;
            while let Some(f) = stack.pop() {
                region.push(f);
                for e in self.cdt.face(f).adjacent_edges() {
                    if e.is_constraint_edge() {
                        continue;
                    }
                    if let Some(next) = e.rev().face().as_inner() {
                        let next = next.fix();
                        if !seen[next.index()] {
                            seen[next.index()] = true;
                            stack.push(next);
                        }
                    }
                }
            }
            let probe = self.centroid(t);
            let clear = self.walls.distance(probe, c) >= c - 1e-7
                && self
                    .nodes
                    .iter()
                    .all(|n| n.distance_to(&probe) >= self.node_clearance - 1e-7)
                && self
                    .ends
                    .iter()
                    .all(|e| e.distance_to(&probe) >= self.end_clearance - 1e-7);
            if clear {
                for f in region {
                    free[f.index()] = true;
                }
            }
        }
        self.free = free;
        self.handles = handles;
    }

    fn centroid(&self, t: FixedFaceHandle<InnerTag>) -> Point {
        let [p, q, r] = self.cdt.face(t).positions();
        Point::new((p.x + q.x + r.x) / 3.0, (p.y + q.y + r.y) / 3.0)
    }

    pub(super) fn walls(&self) -> &[(Point, Point)] {
        &self.walls.segments
    }

    /// The arena as an SVG: free triangles filled, constraints grey, walls
    /// black, for diagnosis.
    #[cfg(test)]
    pub(super) fn svg(&self, board: f64) -> String {
        let mut out = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1000\" height=\"1000\" viewBox=\"0 0 {board} {board}\"><rect width=\"100%\" height=\"100%\" fill=\"white\"/>"
        );
        for t in self.cdt.inner_faces() {
            if self.free[t.fix().index()] {
                let [p, q, r] = t.positions();
                let wrong = self.walls.distance(self.centroid(t.fix()), self.clearance)
                    < self.clearance - 1e-7;
                let fill = if wrong { "#f66" } else { "#cfc" };
                out.push_str(&format!(
                    "<polygon points=\"{:.2},{:.2} {:.2},{:.2} {:.2},{:.2}\" fill=\"{fill}\" stroke=\"#9c9\" stroke-width=\"0.3\"/>",
                    p.x, p.y, q.x, q.y, r.x, r.y
                ));
            }
        }
        for e in self.cdt.undirected_edges() {
            if !e.is_constraint_edge() {
                continue;
            }
            let [p, q] = e.positions();
            let colour = if self.wall_edge[e.fix().index()] {
                "black"
            } else {
                "#999"
            };
            out.push_str(&format!(
                "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"{colour}\" stroke-width=\"0.8\"/>",
                p.x, p.y, q.x, q.y
            ));
        }
        out.push_str("</svg>");
        out
    }

    /// Free and total triangle counts, and any free triangle touching a
    /// wall, for diagnosis.
    #[cfg(test)]
    pub(super) fn describe(&self) -> String {
        let mut touching = Vec::new();
        for t in self.cdt.inner_faces() {
            if !self.free[t.fix().index()] {
                continue;
            }
            for p in t.positions().map(point) {
                if self.walls.distance(p, 1.0) < 1e-6 {
                    let c = self.centroid(t.fix());
                    touching.push(format!(
                        "{p:?} (centroid {c:?} at {:.2} from a wall)",
                        self.walls.distance(c, 100.0)
                    ));
                }
            }
        }
        format!(
            "{} of {} triangles free, {} walls, {} nodes kept clear; free triangles on wall vertices: {touching:?}",
            self.free.iter().filter(|f| **f).count(),
            self.cdt.num_inner_faces(),
            self.walls.segments.len(),
            self.nodes.len()
        )
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

    /// The free triangle holding `p`, or the free triangles round the
    /// vertex or edge it sits on.
    fn free_at(&self, p: Point) -> Vec<usize> {
        match self.cdt.locate(point2(p)) {
            PositionInTriangulation::OnFace(f) => {
                let t = f.index();
                if self.free[t] {
                    vec![t]
                } else {
                    Vec::new()
                }
            }
            PositionInTriangulation::OnVertex(v) => self.free_around(v),
            PositionInTriangulation::OnEdge(e) => {
                let e = self.cdt.directed_edge(e);
                let mut out: Vec<usize> = [e.face(), e.rev().face()]
                    .into_iter()
                    .filter_map(|f| f.as_inner().map(|f| f.fix().index()))
                    .filter(|&t| self.free[t])
                    .collect();
                out.sort_unstable();
                out
            }
            _ => Vec::new(),
        }
    }

    /// Whether the segment from `a` to `p` crosses no wall, keeps the node
    /// clearance from every node it must, ends two fit tolerances from
    /// every wall, and past the stretch where the validator forgives a
    /// crossing at the node keeps two fit tolerances from every wall but
    /// the node's own spokes, so the fitted curve cannot cross one. The
    /// spokes diverge from the leg's start, and a straight leg's fit stays
    /// straight, so they need no margin along the leg.
    fn leg_clear(
        &self,
        a: Point,
        p: Point,
        spokes: &[Vec<Point>],
        forgiven: f64,
        tolerance: f64,
    ) -> bool {
        if self.walls.distance(p, 2.0 * tolerance) < 2.0 * tolerance {
            return false;
        }
        if self
            .walls
            .segments
            .iter()
            .any(|(u, v)| proper_crossing(a, p, *u, *v))
        {
            return false;
        }
        if self
            .nodes
            .iter()
            .any(|n| closest_point_on_segment(&a, &p, n).1 < self.node_clearance)
        {
            return false;
        }
        let len = a.distance_to(&p);
        if len <= forgiven {
            return true;
        }
        let t = forgiven / len;
        let from = Point::new(a.x + (p.x - a.x) * t, a.y + (p.y - a.y) * t);
        let on_spoke = |u: &Point, v: &Point| {
            let mid = Point::new((u.x + v.x) / 2.0, (u.y + v.y) / 2.0);
            spokes
                .iter()
                .any(|spoke| closest_point_on_polyline(spoke, &mid).1 < SAME_POINT * 1e3)
        };
        self.walls.segments.iter().all(|(u, v)| {
            on_spoke(u, v)
                || ([from, p]
                    .iter()
                    .all(|q| closest_point_on_segment(u, v, q).1 >= 2.0 * tolerance)
                    && closest_point_on_segment(&from, &p, u).1 >= 2.0 * tolerance
                    && closest_point_on_segment(&from, &p, v).1 >= 2.0 * tolerance)
        })
    }

    /// Where a path may enter the free region from node `v` through the
    /// corner that follows spoke `at`: the node itself when it sits in the
    /// free region, else the free vertices a straight leg reaches without
    /// crossing a wall, heading out between the corner's spokes, the ones
    /// costing the least detour towards `target` first.
    pub(super) fn entries(
        &self,
        state: &GameState,
        v: NodeId,
        at: Option<HalfEdgeId>,
        c: f64,
        target: Point,
    ) -> Vec<Entry> {
        let a = state.nodes[v].position;
        if at.is_none() {
            let tris = self.free_at(a);
            if !tris.is_empty() {
                return vec![Entry {
                    at: a,
                    tris,
                    leg: None,
                }];
            }
        }
        let radius = MIN_PATH_CLEARANCE * scale(state.board_size);
        let spokes: Vec<Vec<Point>> = state
            .face
            .rotation(v)
            .iter()
            .map(|&h| spoke_path(state, h))
            .collect();
        // A narrow corner gets a narrower margin; the leg is straight, so
        // the embedding reads its heading exactly.
        let wedge = at.map(|h| {
            let rotation = state.face.rotation(v);
            let k = rotation.iter().position(|&x| x == h).expect("own spoke");
            let next = rotation[(k + 1) % rotation.len()];
            let angle = |h| leaving_angle(a, &spoke_path(state, h), radius);
            let from = angle(h);
            let span = if next == h {
                std::f64::consts::TAU
            } else {
                turn(from, angle(next))
            };
            (from, span, LEG_MARGIN.min(span / 4.0))
        });
        let reach = LEG_REACH * c;
        // Half the distance from a node within which the validator lets a
        // line cross a spoke of that node.
        let forgiven = AI_INTERSECTION_TOLERANCE * scale(state.board_size) / 2.0;
        let tolerance = curve::tolerance(state.board_size);
        let mut candidates: Vec<(f64, FixedVertexHandle)> = self
            .cdt
            .vertices()
            .filter_map(|vertex| {
                let p = point(vertex.position());
                let d = p.distance_to(&a);
                (d > radius + LEG_PAST && d <= reach)
                    .then_some((d + p.distance_to(&target), vertex.fix()))
            })
            .collect();
        candidates.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.index().cmp(&y.1.index())));
        let mut out = Vec::new();
        for (_, handle) in candidates {
            let p = self.vertex_point(handle);
            let tris = self.free_around(handle);
            if tris.is_empty() {
                continue;
            }
            if let Some((from, span, margin)) = wedge {
                let heading = (a.y - p.y).atan2(p.x - a.x);
                let into = turn(from, heading);
                if into < margin || into > span - margin {
                    continue;
                }
            }
            if !self.leg_clear(a, p, &spokes, forgiven, tolerance) {
                continue;
            }
            out.push(Entry {
                at: p,
                tris,
                leg: Some((a, p)),
            });
            if out.len() == LEGS {
                break;
            }
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
            edges: Vec::new(),
            all: 0,
            piece_bits: Vec::new(),
            own_bit: 0,
        };
        if own != other {
            return Ok(cuts);
        }
        let adjacency = self.adjacency();
        let mut blocked_flags = vec![false; self.cdt.num_vertices()];
        for v in blocked {
            blocked_flags[v.index()] = true;
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
            // The own cut starts at the node of the piece farthest from
            // both ends, so neither leg can land on it.
            let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
            let room = |v: NodeId| {
                let p = state.nodes[v].position;
                ends.iter()
                    .map(|e| e.distance_to(&p))
                    .fold(f64::INFINITY, f64::min)
            };
            let probe = model
                .cycle(start)
                .into_iter()
                .map(|h| model.origin(h))
                .max_by(|&x, &y| room(x).total_cmp(&room(y)).then(y.cmp(&x)))
                .ok_or(Unroutable::Bug("own piece"))?;
            let v = match self.cdt.locate(point2(state.nodes[probe].position)) {
                PositionInTriangulation::OnVertex(v) => v,
                _ => return Err(Unroutable::Bug("own vertex")),
            };
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
                cuts.edges
                    .push((self.vertex_point(u), self.vertex_point(v), bit));
            }
            cuts.all |= bit;
            match comp {
                Some(comp) => cuts.piece_bits.push((comp, bit)),
                None => cuts.own_bit = bit,
            }
        }
        Ok(cuts)
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

/// Bits of the cuts a leg crosses.
pub(super) fn leg_bits(cuts: &Cuts, leg: Option<(Point, Point)>) -> u64 {
    let Some((a, p)) = leg else {
        return 0;
    };
    cuts.edges
        .iter()
        .filter(|(u, v, _)| proper_crossing(a, p, *u, *v))
        .fold(0, |acc, (_, _, bit)| acc ^ bit)
}
