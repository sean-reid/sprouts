//! The triangle corridor for a move and the taut path through it.
//!
//! Which side of the new line each other boundary piece lands on is a
//! homotopy class. A cut joins each piece to the move's own piece through
//! interior edges; a path's side for that piece flips every time it crosses
//! the cut, so the corridor search carries a parity bit per cut and stops
//! only on the parity vector the move asks for.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use spade::handles::FixedVertexHandle;

use super::{cross, Arena, Unroutable, END_FREE};
use crate::face::{Component, HalfEdgeId, Move as FaceMove};
use crate::types::{GameState, Point};

/// Portals narrower than twice the clearance cost this many extra units
/// per unit they fall short, so a wider corridor wins when one exists.
const SQUEEZE_WEIGHT: f64 = 4.0;
/// How far from either end node, in clearances, a cut prefers to attach.
const ATTACH_ROOM: f64 = 3.0;
/// Extra cost, in clearances, for each portal out of an end node crossed
/// on the way.
const WRAP_COST: f64 = 10.0;
/// Longest loop, in clearances along the path, cut out where offset legs
/// cross each other at a corner.
const TWIST: f64 = 4.0;
/// States one corridor search may hold before it gives the move up. A face
/// with many islands has a parity mask per subset of them, and a search
/// for a mask no corridor reaches would otherwise visit them all.
const SEARCH_STATES: usize = 400_000;

/// How the cuts attach for a move.
pub(super) struct Cuts {
    /// Parity bits of every interior edge, keyed by the vertex pair in
    /// either order.
    pub bits: HashMap<(usize, usize), u32>,
    pub required: u32,
    /// Every bit in use.
    pub all: u32,
}

/// The distances a drawn line keeps: from walls, from node centres, and
/// the room it is relaxed toward around nodes.
#[derive(Clone, Copy)]
pub(super) struct Clearances {
    pub line: f64,
    pub node: f64,
    pub room: f64,
}

/// The path from `mv.a` to `mv.b` through the face, with the other pieces
/// on the sides the move names, offset from the boundary and ready for the
/// clearance push.
pub(super) fn shortest(
    state: &GameState,
    arena: &Arena,
    mv: &FaceMove,
    nodes: &[Point],
    need: Clearances,
    mirrored: bool,
) -> Result<Vec<Point>, Unroutable> {
    let Clearances {
        line: clearance,
        node: node_clearance,
        room: node_room,
    } = need;
    let start = arena.corner(mv.a, mv.at_a)?;
    let goal = arena.corner(mv.b, mv.at_b)?;
    let mut cuts = cuts(state, arena, mv, clearance)?;
    // When a drawn path read back as the other side of every piece, the
    // cuts attached where the arc is a matter of angle; asking for the
    // opposite parity of each gives the mirror image.
    if mirrored {
        cuts.required ^= cuts.all;
    }
    let a = state.nodes[mv.a].position;
    let b = state.nodes[mv.b].position;
    let query = Query {
        arena,
        start: &start.tris,
        goal: &goal.tris,
        cuts: &cuts,
        ends: [arena.node_handle(mv.a), arena.node_handle(mv.b)],
        a,
        b,
        clearance,
    };
    let draw = |portals: &[(Point, Point)]| {
        let path = thread(a, portals, b, nodes, arena.segments(), clearance, node_room);
        super::untwist(path, TWIST * clearance)
    };
    // Crossing a portal out of an end node is only charged for at first; a
    // corridor whose drawn path then crosses itself is searched again with
    // the portals of the ends it wrapped barred.
    let mut barred = [false, false];
    loop {
        let (portals, wraps) = search(&query, barred, false)?;
        if crowded(&portals, [a, b], nodes, clearance, node_clearance) {
            return Err(Unroutable::Room);
        }
        let path = draw(&portals);
        if !super::self_crossing(&path) {
            return Ok(path);
        }
        let retry = [wraps[0] > 0 && !barred[0], wraps[1] > 0 && !barred[1]];
        if retry == [false, false] {
            break;
        }
        barred = [barred[0] || retry[0], barred[1] || retry[1]];
    }
    // A path that still crosses itself came through some triangle twice;
    // the last resort keeps to a corridor that visits each triangle once.
    let (portals, _) = search(&query, barred, true)?;
    Ok(draw(&portals))
}

/// Whether the corridor passes through some portal more often than legs
/// clear of each end and two clearances apart fit there, so no line can be
/// drawn in the move's class.
fn crowded(
    portals: &[(Point, Point)],
    ends: [Point; 2],
    nodes: &[Point],
    clearance: f64,
    node_clearance: f64,
) -> bool {
    // Crossing near `q` keeps the node clearance from every node, so the
    // room needed beside `q` grows as a node comes within that distance.
    let need = |q: &Point| {
        nodes
            .iter()
            .map(|n| node_clearance - n.distance_to(q))
            .fold(clearance, f64::max)
    };
    let mut passes: HashMap<[u64; 4], (f64, usize, bool)> = HashMap::new();
    for (l, r) in portals {
        let (a, b) = if (l.x, l.y) <= (r.x, r.y) {
            (l, r)
        } else {
            (r, l)
        };
        let key = [a.x.to_bits(), a.y.to_bits(), b.x.to_bits(), b.y.to_bits()];
        let at_end = ends.iter().any(|e| {
            e.distance_to(a) < END_FREE * clearance || e.distance_to(b) < END_FREE * clearance
        });
        let fits = a.distance_to(b) - need(a) - need(b);
        // The spokes meeting at an end node make narrow portals the line
        // passes once on its way out; a node crowding them is another matter.
        let spared = at_end && need(a) + need(b) <= 2.0 * clearance;
        let slot = passes.entry(key).or_insert((fits, 0, spared));
        slot.1 += 1;
    }
    passes.values().any(|&(fits, count, spared)| {
        (count > 1 || !spared) && fits < 2.0 * (count as f64 - 1.0) * clearance
    })
}

fn pair(p: FixedVertexHandle, q: FixedVertexHandle) -> (usize, usize) {
    let (p, q) = (p.index(), q.index());
    (p.min(q), p.max(q))
}

/// Lays the cuts and works out the parity vector the move needs.
pub(super) fn cuts(
    state: &GameState,
    arena: &Arena,
    mv: &FaceMove,
    clearance: f64,
) -> Result<Cuts, Unroutable> {
    let model = &state.face;
    let face = model
        .corner_face(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Corner)?;
    let components = model.components(face);
    let own = model
        .component_of(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Corner)?;
    let other = model
        .component_of(mv.b, mv.at_b)
        .map_err(|_| Unroutable::Corner)?;
    let mut cuts = Cuts {
        bits: HashMap::new(),
        required: 0,
        all: 0,
    };
    if own != other {
        return Ok(cuts);
    }
    let own_index = components
        .iter()
        .position(|&c| c == own)
        .ok_or(Unroutable::Corner)?;
    let ends: HashSet<FixedVertexHandle> = [mv.a, mv.b]
        .into_iter()
        .filter_map(|v| arena.node_handle(v))
        .collect();
    let adjacency = interior_adjacency(arena);

    // A bare node has no boundary to attach to, so another piece anchors
    // the cuts and the parities say which pieces the loop separates from it.
    let anchor = if own == Component::Node(mv.a) {
        (0..components.len()).find(|&i| i != own_index)
    } else {
        None
    };
    let anchor_index = anchor.unwrap_or(own_index);
    let anchor_enclosed =
        anchor.is_some_and(|i| mv.enclosed.contains(&model.lowest_node(components[i])));
    // A cut attaches well away from the end nodes: next to one, which arc
    // of the boundary it lands on is a matter of angle, not region. When a
    // tiny piece offers nothing that far, its farthest vertex has to do.
    let end_points = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    let room = |v: &FixedVertexHandle| {
        let q = arena.vertex_point(*v);
        end_points
            .iter()
            .map(|e| e.distance_to(&q))
            .fold(f64::INFINITY, f64::min)
    };
    let own_vertices: Vec<FixedVertexHandle> = arena.piece_vertices[anchor_index]
        .iter()
        .copied()
        .filter(|v| !ends.contains(v))
        .collect();
    let mut targets: HashSet<FixedVertexHandle> = own_vertices
        .iter()
        .copied()
        .filter(|v| room(v) >= ATTACH_ROOM * clearance)
        .collect();
    if targets.is_empty() {
        targets.extend(
            own_vertices
                .iter()
                .copied()
                .max_by(|x, y| room(x).total_cmp(&room(y))),
        );
    }
    // A cut ends where it first meets the piece, else it attaches on the
    // side it happens to leave from.
    let mut blocked: HashSet<FixedVertexHandle> = ends.clone();
    blocked.extend(own_vertices.iter().filter(|v| !targets.contains(v)));
    let anywhere: HashSet<FixedVertexHandle> = own_vertices.iter().copied().collect();
    let arc_left = if mv.a == mv.b {
        None
    } else {
        Some(left_arc(state, mv))
    };

    let mut bit = 0;
    for (i, comp) in components.iter().enumerate() {
        if i == own_index || i == anchor_index {
            continue;
        }
        let trail = trail(&adjacency, &arena.piece_vertices[i], &targets, &blocked)
            .or_else(|| trail(&adjacency, &arena.piece_vertices[i], &anywhere, &ends))
            .ok_or(Unroutable::Corridor)?;
        let enclosed = mv.enclosed.contains(&model.lowest_node(*comp));
        let flip = match &arc_left {
            Some(arc) => {
                let (last, attach) = trail[trail.len() - 1];
                attaches_on(arena, last, attach, arc)?
            }
            None => anchor_enclosed,
        };
        if enclosed != flip {
            cuts.required |= 1 << bit;
        }
        cuts.all |= 1 << bit;
        for (p, q) in trail {
            *cuts.bits.entry(pair(p, q)).or_default() |= 1 << bit;
        }
        bit += 1;
        if bit == 32 {
            return Err(Unroutable::Corridor);
        }
    }
    Ok(cuts)
}

/// Half-edges of the boundary walk from `b` back to `a` that, with the new
/// line, bound the region on the line's left.
fn left_arc(state: &GameState, mv: &FaceMove) -> HashSet<HalfEdgeId> {
    let model = &state.face;
    let (Some(at_a), Some(at_b)) = (mv.at_a, mv.at_b) else {
        return HashSet::new();
    };
    let rotation = model.rotation(mv.a);
    let at = rotation.iter().position(|&h| h == at_a).expect("own spoke");
    let arriving = rotation[(at + 1) % rotation.len()] ^ 1;
    let mut arc = HashSet::new();
    for h in model.cycle(at_b) {
        arc.insert(h);
        if h == arriving {
            break;
        }
    }
    arc
}

/// Whether the corner of `attach` that the cut edge from `last` enters lies
/// on the left arc.
fn attaches_on(
    arena: &Arena,
    last: FixedVertexHandle,
    attach: FixedVertexHandle,
    arc: &HashSet<HalfEdgeId>,
) -> Result<bool, Unroutable> {
    let corners = arena.corners(attach);
    let corner = corners
        .iter()
        .find(|c| {
            c.tris.iter().any(|&t| {
                let verts = arena.ccw_vertices(t);
                verts.contains(&last)
            })
        })
        .ok_or(Unroutable::Corner)?;
    let at = corner.at.ok_or(Unroutable::Corner)?;
    Ok(arc.contains(&at))
}

/// Interior edges of the face as a graph over its vertices.
fn interior_adjacency(arena: &Arena) -> HashMap<FixedVertexHandle, Vec<FixedVertexHandle>> {
    let mut adjacency: HashMap<FixedVertexHandle, Vec<FixedVertexHandle>> = HashMap::new();
    for t in 0..arena.tri_count() {
        let verts = arena.ccw_vertices(t);
        for k in 0..3 {
            let (p, q) = (verts[k], verts[(k + 1) % 3]);
            if arena.is_constraint(p, q) {
                continue;
            }
            adjacency.entry(p).or_default().push(q);
            adjacency.entry(q).or_default().push(p);
        }
    }
    adjacency
}

/// Shortest chain of interior edges from any vertex of a piece to a target
/// vertex, never passing through the move's end nodes.
fn trail(
    adjacency: &HashMap<FixedVertexHandle, Vec<FixedVertexHandle>>,
    sources: &[FixedVertexHandle],
    targets: &HashSet<FixedVertexHandle>,
    forbidden: &HashSet<FixedVertexHandle>,
) -> Option<Vec<(FixedVertexHandle, FixedVertexHandle)>> {
    let mut parent: HashMap<FixedVertexHandle, Option<FixedVertexHandle>> = HashMap::new();
    let mut queue = VecDeque::new();
    for &s in sources {
        if parent.insert(s, None).is_none() {
            queue.push_back(s);
        }
    }
    while let Some(v) = queue.pop_front() {
        for &w in adjacency.get(&v).map(Vec::as_slice).unwrap_or(&[]) {
            if parent.contains_key(&w) {
                continue;
            }
            parent.insert(w, Some(v));
            if targets.contains(&w) {
                let mut edges = Vec::new();
                let mut cur = w;
                while let Some(Some(prev)) = parent.get(&cur) {
                    edges.push((*prev, cur));
                    cur = *prev;
                }
                edges.reverse();
                return Some(edges);
            }
            if !forbidden.contains(&w) {
                queue.push_back(w);
            }
        }
    }
    None
}

/// A search state: where the corridor is and its parity bits.
type Key = (Where, u32);
type Portals = Vec<(Point, Point)>;

/// What a corridor search is over: the arena, the start and goal corners'
/// triangles, the parity cuts, and the move's end nodes and positions.
struct Query<'a> {
    arena: &'a Arena,
    start: &'a [usize],
    goal: &'a [usize],
    cuts: &'a Cuts,
    ends: [Option<FixedVertexHandle>; 2],
    a: Point,
    b: Point,
    clearance: f64,
}
/// Each reached state's predecessor and the portal crossed to get there.
type Parents = HashMap<Key, (Key, Option<(Point, Point)>)>;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Where {
    Start(usize),
    Entered(usize, usize, usize),
    Done,
}

#[derive(Clone, Copy)]
struct Item {
    cost: f64,
    /// Cost plus the straight-line distance still to go, which orders the
    /// heap so the search heads for the goal first.
    bound: f64,
    at: Where,
    mask: u32,
    /// Portals out of each end node crossed so far.
    wraps: [u8; 2],
    pos: Point,
}

impl PartialEq for Item {
    fn eq(&self, other: &Self) -> bool {
        self.bound == other.bound
    }
}
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Item {
    fn cmp(&self, other: &Self) -> Ordering {
        other.bound.total_cmp(&self.bound)
    }
}

/// Dijkstra over (triangle entered, parity mask) from the start corner to
/// the goal corner, returning the portals crossed as (left, right) pairs
/// in travel order.
fn search(
    query: &Query,
    barred: [bool; 2],
    simple: bool,
) -> Result<(Portals, [u8; 2]), Unroutable> {
    let Query {
        arena,
        start,
        cuts,
        ends,
        a,
        b,
        clearance,
        ..
    } = *query;
    let goal: HashSet<usize> = query.goal.iter().copied().collect();
    let mut heap = BinaryHeap::new();
    let mut best: HashMap<Key, f64> = HashMap::new();
    let mut parent: Parents = HashMap::new();
    // The simple pass keys by triangle alone, so each is entered once.
    let keyed = |at: Where, mask: u32| match (simple, at) {
        (true, Where::Start(t) | Where::Entered(t, _, _)) => (Where::Entered(t, 0, 0), 0),
        (true, Where::Done) => (Where::Done, 0),
        (false, at) => (at, mask),
    };
    for &t in start {
        let item = Item {
            cost: 0.0,
            bound: a.distance_to(&b),
            at: Where::Start(t),
            mask: 0,
            wraps: [0, 0],
            pos: a,
        };
        best.insert(keyed(item.at, 0), 0.0);
        heap.push(item);
    }
    while let Some(item) = heap.pop() {
        let key: Key = keyed(item.at, item.mask);
        if best.get(&key).is_some_and(|&c| c < item.cost) {
            continue;
        }
        let tri = match item.at {
            Where::Done => {
                let mut portals = Vec::new();
                let mut key = key;
                while let Some((prev, portal)) = parent.get(&key) {
                    if let Some(p) = portal {
                        portals.push(*p);
                    }
                    key = *prev;
                    // Parent links form a tree; anything longer than the
                    // number of states is a cycle, not a corridor.
                    if portals.len() > parent.len() {
                        return Err(Unroutable::Corridor);
                    }
                }
                portals.reverse();
                return Ok((portals, item.wraps));
            }
            Where::Start(t) | Where::Entered(t, _, _) => t,
        };
        if best.len() > SEARCH_STATES {
            return Err(Unroutable::Corridor);
        }
        let mut relax = |next: Item, portal: Option<(Point, Point)>| {
            let next_key = keyed(next.at, next.mask);
            if best.get(&next_key).is_none_or(|&c| next.cost < c) {
                best.insert(next_key, next.cost);
                parent.insert(next_key, (key, portal));
                heap.push(next);
            }
        };
        if goal.contains(&tri) && item.mask == cuts.required {
            let cost = item.cost + item.pos.distance_to(&b);
            relax(
                Item {
                    cost,
                    bound: cost,
                    at: Where::Done,
                    mask: item.mask,
                    wraps: item.wraps,
                    pos: b,
                },
                None,
            );
        }
        let verts = arena.ccw_vertices(tri);
        for k in 0..3 {
            let (p, q) = (verts[k], verts[(k + 1) % 3]);
            let Some(next_tri) = arena.across(tri, p, q) else {
                continue;
            };
            let bits = cuts.bits.get(&pair(p, q)).copied().unwrap_or(0);
            let mask = item.mask ^ bits;
            // Crossing a portal out of an end node, other than on the first
            // step out of the start or the last into the goal, is charged
            // extra: a corridor wrapping its own end node would cross a run
            // of them, and the path drawn from it would cross itself there.
            let touches = |v: Option<FixedVertexHandle>| v.is_some_and(|v| v == p || v == q);
            let first_step = matches!(item.at, Where::Start(_));
            let last_step = goal.contains(&next_tri) && mask == cuts.required;
            let mut wraps = item.wraps;
            let mut wrap_cost = 0.0;
            if touches(ends[0]) && !first_step {
                if barred[0] {
                    continue;
                }
                wraps[0] = wraps[0].saturating_add(1);
                wrap_cost += WRAP_COST * clearance;
            }
            if touches(ends[1]) && !last_step {
                if barred[1] {
                    continue;
                }
                wraps[1] = wraps[1].saturating_add(1);
                wrap_cost += WRAP_COST * clearance;
            }
            let (pp, qq) = (arena.vertex_point(p), arena.vertex_point(q));
            let mid = Point::new((pp.x + qq.x) / 2.0, (pp.y + qq.y) / 2.0);
            let width = pp.distance_to(&qq);
            let squeeze = SQUEEZE_WEIGHT * (2.0 * clearance - width).max(0.0);
            // Leaving through the edge `p` to `q` of a triangle on its
            // screen-left puts `q` on the traveller's left.
            let portal = (qq, pp);
            let cost = item.cost + item.pos.distance_to(&mid) + squeeze + wrap_cost;
            relax(
                Item {
                    cost,
                    bound: cost + mid.distance_to(&b),
                    at: Where::Entered(next_tri, p.index(), q.index()),
                    mask,
                    wraps,
                    pos: mid,
                },
                Some(portal),
            );
        }
    }
    Err(Unroutable::Corridor)
}

/// Which wall a taut-path corner wraps.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Side {
    Left,
    Right,
    End,
}

impl Side {
    /// The side as seen travelling the other way along the leg.
    fn flip(self) -> Self {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
            Side::End => Side::End,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Apex {
    pub at: Point,
    pub side: Side,
    /// Index into the gate list (portals with the ends added) it sits on.
    pub gate: usize,
}

/// The string pulled taut from `start` to `end` through the portals, as
/// the corners it wraps.
pub(super) fn funnel(start: Point, portals: &[(Point, Point)], end: Point) -> Vec<Apex> {
    let mut gates = Vec::with_capacity(portals.len() + 2);
    gates.push((start, start));
    gates.extend_from_slice(portals);
    gates.push((end, end));
    let same = |p: Point, q: Point| p.x == q.x && p.y == q.y;

    let mut path = vec![Apex {
        at: start,
        side: Side::End,
        gate: 0,
    }];
    let (mut apex, mut left, mut right) = (start, start, start);
    let (mut left_i, mut right_i) = (0usize, 0usize);
    let mut i = 1;
    while i < gates.len() {
        let (l, r) = gates[i];
        // A funnel edge of no length, as around a wrapped vertex, cannot be
        // crossed over, so the side just tightens.
        if cross(apex, right, r) <= 0.0 {
            if same(apex, right) || same(apex, left) || cross(apex, left, r) > 0.0 {
                right = r;
                right_i = i;
            } else {
                path.push(Apex {
                    at: left,
                    side: Side::Left,
                    gate: left_i,
                });
                apex = left;
                right = apex;
                right_i = left_i;
                i = left_i + 1;
                continue;
            }
        }
        if cross(apex, left, l) >= 0.0 {
            if same(apex, left) || same(apex, right) || cross(apex, right, l) < 0.0 {
                left = l;
                left_i = i;
            } else {
                path.push(Apex {
                    at: right,
                    side: Side::Right,
                    gate: right_i,
                });
                apex = right;
                left = apex;
                left_i = right_i;
                i = right_i + 1;
                continue;
            }
        }
        i += 1;
    }
    let last = gates.len() - 1;
    match path.last_mut() {
        Some(apex) if apex.gate == last => apex.side = Side::End,
        _ => path.push(Apex {
            at: end,
            side: Side::End,
            gate: last,
        }),
    }
    path
}

/// Points along each arc, about this fraction of the clearance apart.
const ARC_STEP: f64 = 0.5;
/// Sweep, in radians, past which a corner's arc is taken the other way.
const MISREAD_SWEEP: f64 = 1.25 * std::f64::consts::PI;
/// Share of a portal's length an arc around its end may take.
const ARC_ROOM: f64 = 0.45;
/// Share of the distance to the nearest wall an arc may take.
const WALL_ROOM: f64 = 0.8;

/// The taut path moved inside the face: it crosses every portal at least
/// `c` from the portal's ends, and rounds each corner it wraps with an arc
/// of radius `c`, or the node room at a node, on the open side, joined by
/// runs parallel to the taut segments.
pub(super) fn thread(
    start: Point,
    portals: &[(Point, Point)],
    end: Point,
    nodes: &[Point],
    walls: &[(Point, Point)],
    c: f64,
    node_room: f64,
) -> Vec<Point> {
    let apexes = funnel(start, portals, end);
    let mut gates = Vec::with_capacity(portals.len() + 2);
    gates.push((start, start));
    gates.extend_from_slice(portals);
    gates.push((end, end));
    let same = |p: Point, q: Point| p.x == q.x && p.y == q.y;

    // Each corner's arc radius: what it wants, held short of the portals
    // and walls around it.
    let radius: Vec<f64> = apexes
        .iter()
        .map(|apex| {
            if apex.side == Side::End {
                return 0.0;
            }
            let v = apex.at;
            // A corner at a node gets the node room, the path's own ends
            // included when it wraps back past one of them.
            let at_node = nodes.iter().any(|n| same(*n, v)) || same(start, v) || same(end, v);
            let wanted = if at_node { node_room } else { c };
            // The start and end sit in the gate list as zero-length gates,
            // which say nothing about room at a corner the path wraps there.
            let portal_room = gates
                .iter()
                .filter(|(l, r)| !same(*l, *r) && (same(*l, v) || same(*r, v)))
                .map(|(l, r)| l.distance_to(r) * ARC_ROOM)
                .fold(wanted, f64::min);
            walls
                .iter()
                .filter(|(p, q)| !same(*p, v) && !same(*q, v))
                .map(|(p, q)| crate::geometry::closest_point_on_segment(p, q, &v).1 * WALL_ROOM)
                .fold(portal_room, f64::min)
        })
        .collect();
    // A corner the path wraps more than once gets its arcs nested, each
    // pass a step further out, so the passes do not lie on each other.
    let radius: Vec<f64> = (0..apexes.len())
        .map(|k| {
            let v = apexes[k].at;
            let passes = apexes
                .iter()
                .filter(|x| x.side != Side::End && same(x.at, v))
                .count();
            let pass = apexes[..k].iter().filter(|x| same(x.at, v)).count();
            radius[k] * (pass + 1) as f64 / passes.max(1) as f64
        })
        .collect();

    let mut out = vec![start];
    for k in 1..apexes.len() {
        let (from, to) = (apexes[k - 1], apexes[k]);
        // The leg runs between the corners' arcs, not through the corners,
        // so the two legs of a wrap cross their shared portals apart.
        let leg_from = offset(from.at, to.at, from.side, radius[k - 1]);
        let leg_to = offset(to.at, from.at, to.side.flip(), radius[k]);
        for &(l, r) in &gates[(from.gate + 1).min(to.gate)..to.gate] {
            // A portal ending at a corner is crossed by that corner's arc,
            // and one at the start or end is crossed there.
            let at_corner = [l, r].iter().any(|&e| same(e, from.at) || same(e, to.at));
            if !at_corner {
                out.push(crossing(leg_from, leg_to, l, r, c));
            }
        }
        if to.side == Side::End {
            break;
        }
        let next = apexes[k + 1].at;
        out.extend(arc(to.at, from.at, next, to.side, radius[k]));
    }
    out.push(end);
    out
}

/// `v` moved `r` to the open side of a wall on `side`, for a leg heading
/// from `v` toward `next`.
fn offset(v: Point, next: Point, side: Side, r: f64) -> Point {
    let (dx, dy) = (next.x - v.x, next.y - v.y);
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 || r == 0.0 {
        return v;
    }
    let (dx, dy) = (dx / len, dy / len);
    // Screen-right of the heading for a wall on the left, and the reverse.
    let (nx, ny) = match side {
        Side::Left => (-dy, dx),
        Side::Right => (dy, -dx),
        Side::End => return v,
    };
    Point::new(v.x + nx * r, v.y + ny * r)
}

/// Where the segment `a` to `b` crosses the portal `l` to `r`, pulled at
/// least `c` from either end, or the middle of a portal narrower than that.
fn crossing(a: Point, b: Point, l: Point, r: Point, c: f64) -> Point {
    let len = l.distance_to(&r);
    let t = if len <= 2.0 * c {
        0.5
    } else {
        let d = (b.x - a.x, b.y - a.y);
        let e = (r.x - l.x, r.y - l.y);
        let denom = d.0 * e.1 - d.1 * e.0;
        let t = if denom.abs() < 1e-12 {
            0.5
        } else {
            ((a.x - l.x) * d.1 - (a.y - l.y) * d.0) / -denom
        };
        t.clamp(c / len, 1.0 - c / len)
    };
    Point::new(l.x + (r.x - l.x) * t, l.y + (r.y - l.y) * t)
}

/// The circle of radius `c` about the corner `v`, from the offset of the
/// leg arriving from `before` to the offset of the leg leaving for `after`,
/// swept around the open side.
fn arc(v: Point, before: Point, after: Point, side: Side, c: f64) -> Vec<Point> {
    let heading = |from: Point, to: Point| (to.y - from.y).atan2(to.x - from.x);
    // Offsets sit on the open side: screen-right of the heading for a wall
    // on the left, screen-left for a wall on the right.
    let quarter = std::f64::consts::FRAC_PI_2;
    let offset = if side == Side::Left {
        quarter
    } else {
        -quarter
    };
    let from = heading(before, v) + offset;
    let to = heading(v, after) + offset;
    let turn = (to - from).rem_euclid(std::f64::consts::TAU);
    let mut sweep = if side == Side::Left {
        turn - std::f64::consts::TAU
    } else {
        turn
    };
    // A taut string never bends past a half turn at one corner, and a
    // wrap around a lone node is exactly that; a clearly longer sweep means
    // the corner's side was misread, so take the short way round.
    if sweep.abs() > MISREAD_SWEEP {
        sweep -= std::f64::consts::TAU * sweep.signum();
    }
    let steps = (sweep.abs() / ARC_STEP).ceil().max(1.0) as usize;
    (0..=steps)
        .map(|k| {
            let t = from + sweep * k as f64 / steps as f64;
            Point::new(v.x + c * t.cos(), v.y + c * t.sin())
        })
        .collect()
}
