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
use std::hash::{BuildHasherDefault, Hasher};

use spade::handles::FixedVertexHandle;

use super::cross;
use super::erode::{Arena, Cuts, Entry};
use super::Unroutable;
use crate::types::Point;

/// States one corridor search may hold before it gives the move up.
const SEARCH_STATES: usize = 400_000;

type Portals = Vec<(Point, Point)>;

/// A corridor from the entry to the exit: the portals crossed in travel
/// order and the parity reached.
pub(super) struct Corridor {
    pub portals: Portals,
    /// Parity of every cut along the whole path.
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
    /// Whether the path has been outside the exit's guard.
    out: bool,
}

/// A multiply and fold over the words of a key: the search looks its
/// states up by the hundred thousand, and keys are a few small integers
/// from a trusted source.
#[derive(Default)]
struct FoldHasher(u64);

impl Hasher for FoldHasher {
    fn finish(&self) -> u64 {
        self.0 ^ (self.0 >> 29)
    }

    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_u64(u64::from(b));
        }
    }

    fn write_u64(&mut self, v: u64) {
        self.0 = (self.0.rotate_left(23) ^ v).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }

    fn write_usize(&mut self, v: usize) {
        self.write_u64(v as u64);
    }

    fn write_u8(&mut self, v: u8) {
        self.write_u64(u64::from(v));
    }
}

type Fold = BuildHasherDefault<FoldHasher>;

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

/// Whether a corridor may step from triangle `from` into `to`, and
/// whether it has then been outside the exit's guard.
fn step_out(arena: &Arena, from: usize, to: usize, exit: &Entry, out: bool) -> Option<bool> {
    step_bits(arena.guard(from), arena.guard(to), 1u8 << exit.end, out)
}

/// The guard rule on the bits of the triangle left, `was`, and the one
/// entered, `now`: the middle of the path keeps out of each end's guard,
/// so no guard is entered except the exit's, and entering a triangle with
/// the exit's bit commits the path to finishing there, since the exit's
/// guard is left only by a path that has never been outside it, a loop or
/// a chord setting out. A triangle carrying both ends' bits may be entered
/// on the exit's account, and nothing inside it leads back into the
/// entry's guard alone.
pub(super) fn step_bits(was: u8, now: u8, exit_bit: u8, out: bool) -> Option<bool> {
    let entering = now & !was;
    let leaving = was & !now;
    if (entering & !exit_bit != 0 && now & exit_bit == 0) || (leaving & exit_bit != 0 && out) {
        return None;
    }
    Some(out || leaving & exit_bit != 0)
}

/// Whether a path setting out from entry triangle `t` counts as having
/// been outside the exit's guard: a chord does, unless its ends sit so
/// close that the entry already lies within the exit's guard, and a loop
/// never does.
fn starts_out(arena: &Arena, t: usize, exit: &Entry, is_loop: bool) -> bool {
    !is_loop && arena.guard(t) & (1u8 << exit.end) == 0
}

fn accepts(want: Want, parity: u64, all: u64) -> bool {
    match want {
        Want::Any => true,
        Want::Parity(mask) => parity == mask || parity ^ all == mask,
    }
}

/// Dijkstra over (triangle, parity) from the entry's triangles to an exit
/// triangle whose parity the move accepts, returning the portals crossed
/// as (left, right) pairs in travel order.
pub(super) fn search(
    arena: &Arena,
    cuts: &Cuts,
    entry: &Entry,
    exit: &Entry,
    want: Want,
    is_loop: bool,
) -> Result<Corridor, Unroutable> {
    let goal: HashSet<usize, Fold> = exit.tris.iter().copied().collect();
    let mut heap = BinaryHeap::new();
    let mut best: HashMap<Key, f64, Fold> = HashMap::default();
    let mut parent: HashMap<Key, (Key, (Point, Point)), Fold> = HashMap::default();
    for &t in &entry.tris {
        let key = Key {
            tri: t,
            mask: 0,
            out: starts_out(arena, t, exit, is_loop),
        };
        best.insert(key, 0.0);
        heap.push(Item {
            cost: 0.0,
            bound: exit.at.distance_to(&entry.at),
            key,
            pos: entry.at,
        });
    }
    while let Some(item) = heap.pop() {
        let key = item.key;
        if best.get(&key).is_some_and(|&c| c < item.cost) {
            continue;
        }
        // A loop has to have gone somewhere before it returns.
        let open = !is_loop || parent.contains_key(&key);
        if goal.contains(&key.tri) && open && accepts(want, key.mask, cuts.all) {
            #[cfg(test)]
            super::stats::with(|s| s.states_ok_max = s.states_ok_max.max(best.len()));
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
                parity: key.mask,
            });
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
            let Some(out) = step_out(arena, key.tri, next_tri, exit, key.out) else {
                continue;
            };
            let (pp, qq) = (arena.vertex_point(p), arena.vertex_point(q));
            let mid = Point::new((pp.x + qq.x) / 2.0, (pp.y + qq.y) / 2.0);
            let next = Key {
                tri: next_tri,
                mask: key.mask ^ cuts.bits[edge],
                out,
            };
            let cost = item.cost + item.pos.distance_to(&mid);
            if best.get(&next).is_none_or(|&c| cost < c) {
                best.insert(next, cost);
                // Leaving through the edge `p` to `q` of a triangle on its
                // screen-left puts `q` on the traveller's left.
                parent.insert(next, (key, (qq, pp)));
                heap.push(Item {
                    cost,
                    bound: cost + exit.at.distance_to(&mid),
                    key: next,
                    pos: mid,
                });
            }
        }
    }
    Err(Unroutable::Room)
}

/// Whether an exit triangle can be reached from an entry triangle through
/// the free triangles under the guard rule, parity aside, crossing at
/// least one edge: a loop's exit is its entry, and a path has to go
/// somewhere first.
pub(super) fn reachable(arena: &Arena, entry: &Entry, exit: &Entry, is_loop: bool) -> bool {
    let goal: HashSet<usize, Fold> = exit.tris.iter().copied().collect();
    let mut seen: HashSet<(usize, bool), Fold> = HashSet::default();
    let mut stack: Vec<(usize, bool)> = entry
        .tris
        .iter()
        .map(|&t| (t, starts_out(arena, t, exit, is_loop)))
        .collect();
    while let Some((t, out)) = stack.pop() {
        if !seen.insert((t, out)) {
            continue;
        }
        let verts = arena.ccw_vertices(t);
        for k in 0..3 {
            if let Some((next, _)) = arena.across(t, verts[k], verts[(k + 1) % 3]) {
                let Some(out) = step_out(arena, t, next, exit, out) else {
                    continue;
                };
                if goal.contains(&next) {
                    return true;
                }
                if !seen.contains(&(next, out)) {
                    stack.push((next, out));
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

/// The whole path of a corridor: the taut path from the entry node to the
/// exit node through the free region.
pub(super) fn draw(corridor: &Corridor, entry: &Entry, exit: &Entry) -> Vec<Point> {
    let mut out = funnel(entry.at, &corridor.portals, exit.at);
    out.dedup_by(|p, q| p.distance_to(q) < 1e-9);
    out
}

/// Vertices the cuts keep off: the end nodes, so no path starts or ends
/// on a cut.
pub(super) fn blocked(arena: &Arena, entry: &Entry, exit: &Entry) -> Vec<FixedVertexHandle> {
    [entry.at, exit.at]
        .into_iter()
        .filter_map(|p| arena.vertex_at(p))
        .collect()
}
