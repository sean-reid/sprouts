//! The triangle corridor through the eroded face and the taut path in it.
//!
//! Which side of the new line each other boundary piece lands on is a
//! homotopy class. Each piece has a cut from it to the rim; a path's side
//! for the piece flips every time it crosses the cut, so the search carries
//! a parity bit per cut. The first search takes any parity and the sides
//! of the path it finds are read off geometrically; a second search, when
//! one is needed, asks for the parity vector that flips exactly the pieces
//! that landed wrong.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use spade::handles::FixedVertexHandle;

use super::cross;
use super::erode::{leg_bits, Arena, Cuts, Entry};
use super::Unroutable;
use crate::types::Point;

/// States one corridor search may hold before it gives the move up.
const SEARCH_STATES: usize = 400_000;

type Portals = Vec<(Point, Point)>;

/// A corridor from an entry to an exit: the portals crossed in travel
/// order, the parity reached, and which entry and exit it joins.
pub(super) struct Corridor {
    pub portals: Portals,
    pub entry: usize,
    pub exit: usize,
    /// Parity of every cut along the whole path, legs included.
    pub parity: u64,
}

/// What a corridor must satisfy at the exit.
#[derive(Clone, Copy)]
pub(super) enum Want {
    Any,
    /// The parity vector or its complement, which is the same move drawn
    /// round the other side of its own piece.
    Parity(u64),
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct Key {
    tri: usize,
    mask: u64,
    entry: usize,
}

#[derive(Clone, Copy)]
struct Item {
    cost: f64,
    /// Cost plus the straight-line distance still to go, which orders the
    /// heap so the search heads for the goal first.
    bound: f64,
    key: Key,
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

fn accepts(want: Want, parity: u64, all: u64) -> bool {
    match want {
        Want::Any => true,
        Want::Parity(mask) => parity == mask || parity ^ all == mask,
    }
}

/// Dijkstra over (triangle, parity) from any entry to any exit whose
/// parity the move accepts, returning the portals crossed as (left, right)
/// pairs in travel order.
pub(super) fn search(
    arena: &Arena,
    cuts: &Cuts,
    entries: &[Entry],
    exits: &[Entry],
    want: Want,
    is_loop: bool,
) -> Result<Corridor, Unroutable> {
    let entry_bits: Vec<u64> = entries.iter().map(|e| leg_bits(cuts, e.leg)).collect();
    let exit_bits: Vec<u64> = exits.iter().map(|e| leg_bits(cuts, e.leg)).collect();
    // Exits by triangle, for the goal test.
    let mut goal: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, e) in exits.iter().enumerate() {
        for &t in &e.tris {
            goal.entry(t).or_default().push(i);
        }
    }
    let nearest_exit = |p: Point| {
        exits
            .iter()
            .map(|e| e.at.distance_to(&p))
            .fold(f64::INFINITY, f64::min)
    };
    let mut heap = BinaryHeap::new();
    let mut best: HashMap<Key, f64> = HashMap::new();
    let mut parent: HashMap<Key, (Key, (Point, Point))> = HashMap::new();
    for (i, e) in entries.iter().enumerate() {
        for &t in &e.tris {
            let key = Key {
                tri: t,
                mask: entry_bits[i],
                entry: i,
            };
            best.insert(key, 0.0);
            heap.push(Item {
                cost: 0.0,
                bound: nearest_exit(e.at),
                key,
                pos: e.at,
            });
        }
    }
    while let Some(item) = heap.pop() {
        let key = item.key;
        if best.get(&key).is_some_and(|&c| c < item.cost) {
            continue;
        }
        if let Some(exits_here) = goal.get(&key.tri) {
            // A loop has to have gone somewhere before it returns, and
            // returns by another entry when it can.
            let open = !is_loop || parent.contains_key(&key);
            let fits = |x: &&usize| open && accepts(want, key.mask ^ exit_bits[**x], cuts.all);
            let found = exits_here
                .iter()
                .filter(|x| **x != key.entry)
                .find(fits)
                .or_else(|| exits_here.iter().find(fits));
            if let Some(&exit) = found {
                let mut portals = Vec::new();
                let mut cur = key;
                while let Some((prev, portal)) = parent.get(&cur) {
                    portals.push(*portal);
                    cur = *prev;
                    if portals.len() > parent.len() {
                        return Err(Unroutable::Bug("corridor cycle"));
                    }
                }
                portals.reverse();
                return Ok(Corridor {
                    portals,
                    entry: key.entry,
                    exit,
                    parity: key.mask ^ exit_bits[exit],
                });
            }
        }
        if best.len() > SEARCH_STATES {
            return Err(Unroutable::Bug("search states"));
        }
        let verts = arena.ccw_vertices(key.tri);
        for k in 0..3 {
            let (p, q) = (verts[k], verts[(k + 1) % 3]);
            let Some((next_tri, edge)) = arena.across(key.tri, p, q) else {
                continue;
            };
            let (pp, qq) = (arena.vertex_point(p), arena.vertex_point(q));
            let mid = Point::new((pp.x + qq.x) / 2.0, (pp.y + qq.y) / 2.0);
            let next = Key {
                tri: next_tri,
                mask: key.mask ^ cuts.bits[edge],
                entry: key.entry,
            };
            let cost = item.cost + item.pos.distance_to(&mid);
            if best.get(&next).is_none_or(|&c| cost < c) {
                best.insert(next, cost);
                // Leaving through the edge `p` to `q` of a triangle on its
                // screen-left puts `q` on the traveller's left.
                parent.insert(next, (key, (qq, pp)));
                heap.push(Item {
                    cost,
                    bound: cost + nearest_exit(mid),
                    key: next,
                    pos: mid,
                });
            }
        }
    }
    Err(Unroutable::Room)
}

/// Whether an exit triangle can be reached from an entry triangle through
/// the free triangles, parity aside, crossing at least one edge: a loop's
/// exits are its entries, and a path has to go somewhere first.
pub(super) fn reachable(arena: &Arena, entries: &[Entry], exits: &[Entry]) -> bool {
    let goal: HashSet<usize> = exits.iter().flat_map(|e| e.tris.iter().copied()).collect();
    let mut seen: HashSet<usize> = HashSet::new();
    let mut stack: Vec<usize> = entries
        .iter()
        .flat_map(|e| e.tris.iter().copied())
        .collect();
    while let Some(t) = stack.pop() {
        if !seen.insert(t) {
            continue;
        }
        let verts = arena.ccw_vertices(t);
        for k in 0..3 {
            if let Some((next, _)) = arena.across(t, verts[k], verts[(k + 1) % 3]) {
                if goal.contains(&next) {
                    return true;
                }
                if !seen.contains(&next) {
                    stack.push(next);
                }
            }
        }
    }
    false
}

/// The string pulled taut from `start` to `end` through the portals.
pub(super) fn funnel(start: Point, portals: &[(Point, Point)], end: Point) -> Vec<Point> {
    let mut gates = Vec::with_capacity(portals.len() + 2);
    gates.push((start, start));
    gates.extend_from_slice(portals);
    gates.push((end, end));
    let same = |p: Point, q: Point| p.x == q.x && p.y == q.y;

    let mut path = vec![start];
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
                path.push(left);
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
                path.push(right);
                apex = right;
                left = apex;
                left_i = right_i;
                i = right_i + 1;
                continue;
            }
        }
        i += 1;
    }
    if path.last().is_none_or(|p| !same(*p, end)) {
        path.push(end);
    }
    path
}

/// The whole path of a corridor: the leg in, the taut path through the
/// free region, the leg out.
pub(super) fn draw(corridor: &Corridor, entries: &[Entry], exits: &[Entry]) -> Vec<Point> {
    let entry = &entries[corridor.entry];
    let exit = &exits[corridor.exit];
    let mut out = Vec::new();
    if let Some((a, _)) = entry.leg {
        out.push(a);
    }
    out.extend(funnel(entry.at, &corridor.portals, exit.at));
    if let Some((b, _)) = exit.leg {
        out.push(b);
    }
    out.dedup_by(|p, q| p.distance_to(q) < 1e-9);
    out
}

/// Vertices the cuts keep off: the end nodes, entries and exits, so no leg
/// starts or ends on a cut.
pub(super) fn blocked(
    arena: &Arena,
    entries: &[Entry],
    exits: &[Entry],
    ends: [Point; 2],
) -> Vec<FixedVertexHandle> {
    entries
        .iter()
        .chain(exits)
        .map(|e| e.at)
        .chain(ends)
        .filter_map(|p| arena.vertex_at(p))
        .collect()
}
