//! Draws a face-model move as a curve inside its face.
//!
//! The face is triangulated with the flattenings of its boundary as
//! constraints. A corridor of triangles joins the two corners, chosen so
//! that every other boundary piece ends up on the side the move names, the
//! funnel algorithm pulls the shortest path through it, and the path is then
//! pushed off the boundary to the AI's clearance before the spline is fit.

use std::collections::hash_map::Entry;
use std::collections::HashMap;

use spade::handles::{FixedFaceHandle, FixedVertexHandle, InnerTag};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use crate::ai::rank_node_placements;
use crate::curve;
use crate::embedding::{self, half_edge_path};
use crate::face::{Component, FaceId, HalfEdgeId, Move as FaceMove, NodeId};
use crate::geometry::{decimate_polyline, resample_polyline};
use crate::types::{
    ai_node_clearance, node_disc, scale, GameState, Move, Player, Point, MIN_NODE_SPACING,
    MIN_NODE_SPACING_AI, MIN_PATH_CLEARANCE, MIN_PATH_CLEARANCE_AI,
};
use crate::validation::{validate_ai_move, validate_move, Rejection, RejectionKind};

mod clearance;
mod corridor;
mod small_loop;
#[cfg(any(test, feature = "bench"))]
pub mod sweep;
#[cfg(test)]
mod tests;

type Cdt = ConstrainedDelaunayTriangulation<Point2<f64>>;

/// Why a legal move could not be drawn.
#[derive(Debug, Clone, PartialEq)]
pub enum Unroutable {
    /// The face geometry has no corner matching an endpoint of the move.
    Corner,
    /// No corridor through the face puts the other pieces on the named sides.
    Corridor,
    /// No circle for a loop round nothing fits beside the node.
    Room,
    /// The pushed path still crosses the boundary.
    Crossing,
    /// The drawn curve reads back as a different move.
    Topology,
    Rejected(Rejection),
}

/// Boundary segments longer than this many clearances are subdivided so
/// long straight edges still offer portals along their length.
const SUBDIVIDE: f64 = 3.0;

/// Room wanted around every node, in node spacings, where the face allows
/// it, so a later loop at that node still has somewhere to go.
const NODE_ROOM: f64 = 2.0;

/// Within this many clearances of an end node the legs of a loop may run
/// as close together as they like.
const END_FREE: f64 = 3.0;

/// How far a constraint edge's midpoint may sit off a run it belongs to.
const ON_RUN: f64 = 1e-6;

/// Key of a boundary segment that belongs to the board edge, not a line.
const BOARD: HalfEdgeId = usize::MAX;

/// The triangulated face with its boundary indexed by half-edge.
pub(crate) struct Arena {
    cdt: Cdt,
    /// Triangles inside the face, in a fixed order.
    tris: Vec<FixedFaceHandle<InnerTag>>,
    tri_index: HashMap<FixedFaceHandle<InnerTag>, usize>,
    /// Directed boundary segment, by vertex index, to the half-edge whose
    /// face lies on its screen-left.
    keys: HashMap<(usize, usize), HalfEdgeId>,
    node_vertex: HashMap<NodeId, FixedVertexHandle>,
    /// Vertices of each boundary piece, in the face's component order.
    piece_vertices: Vec<Vec<FixedVertexHandle>>,
    /// Boundary segments of the face, the board edge included.
    segments: Vec<(Point, Point)>,
}

/// A wedge of face triangles around a node between two of its spokes.
pub(crate) struct Corner {
    pub tris: Vec<usize>,
    /// The spoke the wedge follows counterclockwise, `None` at a bare node.
    pub at: Option<HalfEdgeId>,
}

pub(crate) fn point(p: Point2<f64>) -> Point {
    Point::new(p.x, p.y)
}

fn point2(p: Point) -> Point2<f64> {
    Point2::new(p.x, p.y)
}

/// Screen-side test: positive when `q` is on the right of the heading
/// `o` to `p`, where y grows downward.
pub(crate) fn cross(o: Point, p: Point, q: Point) -> f64 {
    (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x)
}

/// Splits long segments. The points of a segment are computed from its
/// lower end whichever way it is walked, so the two half-edges of a line
/// land on the same vertices.
fn subdivide(path: &[Point], step: f64) -> Vec<Point> {
    let mut out = Vec::with_capacity(path.len());
    for pair in path.windows(2) {
        let (p, q) = (pair[0], pair[1]);
        let pieces = (p.distance_to(&q) / step).ceil().max(1.0) as usize;
        let forward = (p.x, p.y) <= (q.x, q.y);
        let (lo, hi) = if forward { (p, q) } else { (q, p) };
        out.push(p);
        for k in 1..pieces {
            let k = if forward { k } else { pieces - k };
            let t = k as f64 / pieces as f64;
            out.push(Point::new(
                lo.x + (hi.x - lo.x) * t,
                lo.y + (hi.y - lo.y) * t,
            ));
        }
    }
    if let Some(last) = path.last() {
        out.push(*last);
    }
    out
}

impl Arena {
    pub(crate) fn build(state: &GameState, face: FaceId, step: f64) -> Result<Self, Unroutable> {
        let model = &state.face;
        let components = model.components(face);
        let mut cdt = Cdt::new();
        let insert =
            |cdt: &mut Cdt, p: Point| cdt.insert(point2(p)).map_err(|_| Unroutable::Corner);
        let mut node_vertex = HashMap::new();
        for comp in components {
            let nodes: Vec<NodeId> = match *comp {
                Component::Node(v) => vec![v],
                Component::Cycle(start) => model
                    .cycle(start)
                    .into_iter()
                    .flat_map(|h| [model.origin(h), model.head(h)])
                    .collect(),
            };
            for v in nodes {
                if let Entry::Vacant(slot) = node_vertex.entry(v) {
                    slot.insert(insert(&mut cdt, state.nodes[v].position)?);
                }
            }
        }

        // Constraints go in first; a later one can split an earlier edge, so
        // the keys are read off the finished triangulation.
        let constrain = |cdt: &mut Cdt, from: FixedVertexHandle, to: FixedVertexHandle| {
            if from != to {
                cdt.add_constraint_and_split(from, to, |p| p);
            }
        };
        let edge = state.board_size as f64;
        let corners = [
            Point::new(0.0, 0.0),
            Point::new(0.0, edge),
            Point::new(edge, edge),
            Point::new(edge, 0.0),
            Point::new(0.0, 0.0),
        ];
        let mut rim = Vec::new();
        for p in subdivide(&corners, step) {
            rim.push(insert(&mut cdt, p)?);
        }
        for pair in rim.windows(2) {
            constrain(&mut cdt, pair[0], pair[1]);
        }

        let mut piece_vertices = Vec::with_capacity(components.len());
        let mut runs: Vec<(Vec<FixedVertexHandle>, HalfEdgeId)> = vec![(rim.clone(), BOARD)];
        for comp in components {
            let mut vertices = Vec::new();
            match *comp {
                Component::Node(v) => vertices.push(node_vertex[&v]),
                Component::Cycle(start) => {
                    for h in model.cycle(start) {
                        let mut path = half_edge_path(state, h);
                        path[0] = state.nodes[model.origin(h)].position;
                        let n = path.len() - 1;
                        path[n] = state.nodes[model.head(h)].position;
                        let mut handles = Vec::new();
                        for p in subdivide(&path, step) {
                            handles.push(insert(&mut cdt, p)?);
                        }
                        for pair in handles.windows(2) {
                            constrain(&mut cdt, pair[0], pair[1]);
                        }
                        vertices.extend(handles.iter().copied());
                        runs.push((handles, h));
                    }
                }
            }
            piece_vertices.push(vertices);
        }

        // Every constraint edge of the finished triangulation lies on one
        // of the runs just inserted, split or not; it is keyed to the run
        // whose polyline it sits on, in that run's direction.
        let runs: Vec<(Vec<Point>, HalfEdgeId)> = runs
            .into_iter()
            .map(|(handles, h)| {
                let pts = handles
                    .iter()
                    .map(|&v| point(cdt.vertex(v).position()))
                    .collect();
                (pts, h)
            })
            .collect();
        let mut keys = HashMap::new();
        for edge in cdt.undirected_edges() {
            if !edge.is_constraint_edge() {
                continue;
            }
            let [p, q] = edge.vertices().map(|v| v.fix());
            let (pp, qq) = (
                point(cdt.vertex(p).position()),
                point(cdt.vertex(q).position()),
            );
            let mid = Point::new((pp.x + qq.x) / 2.0, (pp.y + qq.y) / 2.0);
            for (pts, h) in &runs {
                let Some(seg) = pts.windows(2).find(|w| {
                    crate::geometry::closest_point_on_segment(&w[0], &w[1], &mid).1 < ON_RUN
                }) else {
                    continue;
                };
                let forward = (qq.x - pp.x) * (seg[1].x - seg[0].x)
                    + (qq.y - pp.y) * (seg[1].y - seg[0].y)
                    >= 0.0;
                let key = if forward {
                    (p.index(), q.index())
                } else {
                    (q.index(), p.index())
                };
                keys.insert(key, *h);
            }
        }

        let mut arena = Self {
            cdt,
            tris: Vec::new(),
            tri_index: HashMap::new(),
            keys,
            node_vertex,
            piece_vertices,
            segments: Vec::new(),
        };
        arena.fill(components)?;
        Ok(arena)
    }

    /// Marks the triangles of the face: the whole board when the face has no
    /// cycle, else everything reachable from the left of the longest
    /// boundary segment, which no sliver from a grazing line can be, without
    /// crossing a constraint.
    fn fill(&mut self, components: &[Component]) -> Result<(), Unroutable> {
        let has_cycle = components.iter().any(|c| matches!(c, Component::Cycle(_)));
        let mut stack: Vec<FixedFaceHandle<InnerTag>> = if has_cycle {
            let length = |&(from, to): &(usize, usize)| {
                let p = self.vertex_point(FixedVertexHandle::from_index(from));
                let q = self.vertex_point(FixedVertexHandle::from_index(to));
                p.distance_to(&q)
            };
            let (&(from, to), _) = self
                .keys
                .iter()
                .filter(|(_, &h)| h != BOARD)
                .max_by(|x, y| length(x.0).total_cmp(&length(y.0)).then(x.0.cmp(y.0)))
                .ok_or(Unroutable::Corner)?;
            let from = FixedVertexHandle::from_index(from);
            let to = FixedVertexHandle::from_index(to);
            let e = self
                .cdt
                .get_edge_from_neighbors(from, to)
                .ok_or(Unroutable::Corner)?;
            let [p, q] = e.positions().map(point);
            let left_of = |w: Option<Point2<f64>>| w.is_some_and(|w| cross(p, q, point(w)) < 0.0);
            let seed = if left_of(e.opposite_position()) {
                e.face().as_inner()
            } else if left_of(e.rev().opposite_position()) {
                e.rev().face().as_inner()
            } else {
                None
            };
            vec![seed.map(|f| f.fix()).ok_or(Unroutable::Corner)?]
        } else {
            self.cdt.inner_faces().map(|f| f.fix()).collect()
        };
        while let Some(t) = stack.pop() {
            if self.tri_index.contains_key(&t) {
                continue;
            }
            self.tri_index.insert(t, self.tris.len());
            self.tris.push(t);
            for e in self.cdt.face(t).adjacent_edges() {
                if e.is_constraint_edge() {
                    self.segments
                        .push((point(e.from().position()), point(e.to().position())));
                    continue;
                }
                if let Some(next) = e.rev().face().as_inner() {
                    stack.push(next.fix());
                }
            }
        }
        Ok(())
    }

    pub(crate) fn tri_count(&self) -> usize {
        self.tris.len()
    }

    pub(crate) fn segments(&self) -> &[(Point, Point)] {
        &self.segments
    }

    pub(crate) fn vertex_point(&self, v: FixedVertexHandle) -> Point {
        point(self.cdt.vertex(v).position())
    }

    /// Vertices of a triangle, counterclockwise on screen, so each edge in
    /// turn has the triangle on its screen-left. spade keeps its faces
    /// counterclockwise with y up, which is clockwise on screen, so the
    /// order is reversed; this holds for slivers an area sign would not.
    pub(crate) fn ccw_vertices(&self, t: usize) -> [FixedVertexHandle; 3] {
        let [a, b, c] = self.cdt.face(self.tris[t]).vertices().map(|v| v.fix());
        [a, c, b]
    }

    pub(crate) fn is_constraint(&self, p: FixedVertexHandle, q: FixedVertexHandle) -> bool {
        self.cdt
            .get_edge_from_neighbors(p, q)
            .is_some_and(|e| e.is_constraint_edge())
    }

    /// The face triangle across the edge `p`, `q` from triangle `t`.
    pub(crate) fn across(
        &self,
        t: usize,
        p: FixedVertexHandle,
        q: FixedVertexHandle,
    ) -> Option<usize> {
        let e = self.cdt.get_edge_from_neighbors(p, q)?;
        if e.is_constraint_edge() {
            return None;
        }
        let here = self.tris[t];
        let other = [e.face(), e.rev().face()]
            .into_iter()
            .filter_map(|f| f.as_inner().map(|f| f.fix()))
            .find(|f| *f != here)?;
        self.tri_index.get(&other).copied()
    }

    /// Every face triangle with `v` as a vertex.
    fn incident(&self, v: FixedVertexHandle) -> Vec<usize> {
        self.cdt
            .vertex(v)
            .out_edges()
            .filter_map(|e| e.face().as_inner())
            .filter_map(|f| self.tri_index.get(&f.fix()).copied())
            .collect()
    }

    /// The wedges of face triangles around `v`, one per corner of the face
    /// at that node.
    pub(crate) fn corners(&self, v: FixedVertexHandle) -> Vec<Corner> {
        let incident = self.incident(v);
        let mut group: Vec<usize> = (0..incident.len()).collect();
        fn find(g: &mut [usize], x: usize) -> usize {
            if g[x] != x {
                g[x] = find(g, g[x]);
            }
            g[x]
        }
        for (i, &t) in incident.iter().enumerate() {
            let verts = self.ccw_vertices(t);
            for k in 0..3 {
                let (p, q) = (verts[k], verts[(k + 1) % 3]);
                if p != v && q != v {
                    continue;
                }
                if let Some(other) = self.across(t, p, q) {
                    if let Some(j) = incident.iter().position(|&x| x == other) {
                        let (a, b) = (find(&mut group, i), find(&mut group, j));
                        group[a] = b;
                    }
                }
            }
        }
        let mut corners: Vec<(usize, Corner)> = Vec::new();
        for (i, &t) in incident.iter().enumerate() {
            let root = find(&mut group, i);
            let at = self.outgoing_key(t, v);
            match corners.iter_mut().find(|(r, _)| *r == root) {
                Some((_, corner)) => {
                    corner.tris.push(t);
                    if at.is_some() {
                        corner.at = at;
                    }
                }
                None => corners.push((root, Corner { tris: vec![t], at })),
            }
        }
        corners.into_iter().map(|(_, c)| c).collect()
    }

    /// The half-edge leaving `v` along a constraint edge of triangle `t`.
    fn outgoing_key(&self, t: usize, v: FixedVertexHandle) -> Option<HalfEdgeId> {
        let verts = self.ccw_vertices(t);
        (0..3).find_map(|k| {
            let (p, q) = (verts[k], verts[(k + 1) % 3]);
            if p != v {
                return None;
            }
            self.keys
                .get(&(p.index(), q.index()))
                .copied()
                .filter(|&h| h != BOARD)
        })
    }

    /// The corner of node `v` that follows spoke `at`.
    pub(crate) fn corner(&self, v: NodeId, at: Option<HalfEdgeId>) -> Result<Corner, Unroutable> {
        let handle = *self.node_vertex.get(&v).ok_or(Unroutable::Corner)?;
        self.corners(handle)
            .into_iter()
            .find(|c| c.at == at)
            .ok_or(Unroutable::Corner)
    }

    pub(crate) fn node_handle(&self, v: NodeId) -> Option<FixedVertexHandle> {
        self.node_vertex.get(&v).copied()
    }
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
    // it is still legal at the clearance a human line gets. Only a failure
    // about clearance can change at the lower one.
    let about_clearance = matches!(
        drawn,
        Err(Unroutable::Room)
            | Err(Unroutable::Rejected(Rejection {
                kind: RejectionKind::PathTooCloseToNode,
                ..
            }))
    );
    if !about_clearance || want.node_clearance <= want.fallback {
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
    let Rules {
        clearance: c,
        node_clearance,
        spacing,
        ..
    } = *want;
    if mv.a == mv.b && mv.enclosed.is_empty() {
        return small_loop::route(state, mv, c, node_clearance);
    }
    let face = state
        .face
        .corner_face(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Corner)?;
    let arena = Arena::build(state, face, SUBDIVIDE * c)?;
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    let nodes: Vec<Point> = state
        .nodes
        .iter()
        .filter(|n| n.id != mv.a && n.id != mv.b)
        .map(|n| n.position)
        .collect();
    // The relaxation settles a little short of its room, so the room sits a
    // line clearance above what the validator demands.
    let room = (NODE_ROOM * spacing).max(node_clearance + c);
    let need = corridor::Clearances {
        line: c,
        node: node_clearance,
        room,
    };
    let threaded = corridor::shortest(state, &arena, mv, &nodes, need, false)?;
    let obstacles = clearance::Obstacles::new(
        arena.segments(),
        nodes.clone(),
        ends,
        c,
        node_clearance,
        room,
        spacing,
    );
    // The relaxed path is the one wanted; the threaded path is already
    // legal when the relaxation ends up somewhere it cannot be used.
    let pushed = clearance::push(threaded.clone(), &obstacles);
    let drawn = match pushed {
        Ok(path) => finish(state, mv, path).or_else(|_| finish(state, mv, threaded)),
        Err(_) => finish(state, mv, threaded),
    };
    match drawn {
        Err(Unroutable::Topology) => {
            let threaded = corridor::shortest(state, &arena, mv, &nodes, need, true)?;
            match clearance::push(threaded.clone(), &obstacles) {
                Ok(path) => finish(state, mv, path).or_else(|_| finish(state, mv, threaded)),
                Err(_) => finish(state, mv, threaded),
            }
        }
        other => other,
    }
}

/// Fits the spline, places the node, validates, and checks the curve reads
/// back as the move. A loop at a bare node may read as the other side.
pub(crate) fn finish(
    state: &GameState,
    mv: &FaceMove,
    path: Vec<Point>,
) -> Result<Move, Unroutable> {
    let tolerance = curve::tolerance(state.board_size);
    // Wiggles under the fit tolerance are relaxation noise, not shape;
    // dropping them first gives the fit a few long, even cubics.
    let Rules {
        clearance, spacing, ..
    } = rules(state);
    // A point on a path of length L is within L/2 of one end, so a path
    // under two spacings has nowhere for its node.
    let length: f64 = path.windows(2).map(|w| w[0].distance_to(&w[1])).sum();
    if length < 2.0 * spacing {
        return Err(Unroutable::Room);
    }
    let path = decimate_polyline(&path, tolerance);
    let segments = curve::fit(&path, tolerance);
    let flat = curve::flatten(&segments, tolerance);
    let node = place_node(state, &flat, spacing).ok_or(Unroutable::Room)?;
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    if self_crossing(&flat) || self_touching(&flat, ends, clearance) {
        return Err(Unroutable::Crossing);
    }
    let mov = Move::from_segments(mv.a, mv.b, path, segments, node, state.current_player);
    match state.current_player {
        Player::AI => validate_ai_move(state, &mov),
        Player::Human => validate_move(state, &mov),
    }
    .map_err(Unroutable::Rejected)?;
    let read = embedding::embed(state, &mov).map_err(Unroutable::Rejected)?;
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

/// Legs offset from a taut path can cross where they meet at a corner,
/// making a loop no longer than `max_len` along the path; the loop is cut
/// out at the crossing.
pub(super) fn untwist(mut path: Vec<Point>, max_len: f64) -> Vec<Point> {
    'again: loop {
        let n = path.len().saturating_sub(1);
        for i in 0..n {
            let mut len = 0.0;
            for j in i + 2..n {
                len += path[j - 1].distance_to(&path[j]);
                if len > max_len {
                    break;
                }
                if let Some(x) = crossing_point(path[i], path[i + 1], path[j], path[j + 1]) {
                    let mut out = path[..=i].to_vec();
                    out.push(x);
                    out.extend_from_slice(&path[j + 1..]);
                    path = out;
                    continue 'again;
                }
            }
        }
        return path;
    }
}

/// Where segment `a b` properly crosses segment `c d`.
fn crossing_point(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
    if !clearance::proper_crossing(a, b, c, d) {
        return None;
    }
    let (r, s) = ((b.x - a.x, b.y - a.y), (d.x - c.x, d.y - c.y));
    let denom = r.0 * s.1 - r.1 * s.0;
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = ((c.x - a.x) * s.1 - (c.y - a.y) * s.0) / denom;
    Some(Point::new(a.x + t * r.0, a.y + t * r.1))
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
