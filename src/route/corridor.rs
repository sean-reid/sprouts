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
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};
use std::hash::{BuildHasherDefault, Hasher};

use spade::handles::FixedVertexHandle;

use super::cross;
use super::erode::{Arena, Cuts, Entry, Fences};
use super::Unroutable;
use crate::types::Point;

/// States one corridor search may hold before it gives the move up.
pub(super) const SEARCH_STATES: usize = 400_000;

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
    /// In a triangle a fence runs through, whether the path is on the
    /// side of the segment holding the vertex its two edges share.
    near: bool,
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

/// What bounds a search: the fences it may not cross and the states it
/// may hold before it gives the move up.
#[derive(Clone, Copy)]
pub(super) struct Limits<'a> {
    pub fences: &'a Fences,
    pub cap: usize,
}

/// Dijkstra over (triangle, parity) from the entry's triangles to an exit
/// triangle whose parity the move accepts, never crossing a fence,
/// returning the portals crossed as (left, right) pairs in travel order,
/// or a bug once it holds more states than the limit. A fence meets an
/// edge at its midpoint, so a step across such an edge is on one half of
/// it, and in a fenced triangle the half tells which side of the segment
/// the path is on.
pub(super) fn search(
    arena: &Arena,
    cuts: &Cuts,
    entry: &Entry,
    exit: &Entry,
    want: Want,
    is_loop: bool,
    limits: Limits<'_>,
) -> Result<Corridor, Unroutable> {
    let Limits { fences, cap } = limits;
    let goal: HashSet<usize, Fold> = exit.tris.iter().copied().collect();
    let mut heap = BinaryHeap::new();
    let mut best: HashMap<Key, f64, Fold> = HashMap::default();
    let mut parent: HashMap<Key, (Key, (Point, Point)), Fold> = HashMap::default();
    for &t in &entry.tris {
        let key = Key {
            tri: t,
            mask: 0,
            out: starts_out(arena, t, exit, is_loop),
            near: false,
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
        if best.len() > cap {
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
            // The vertex the crossing keeps to, when a fence fixes it.
            let mut half = None;
            if let Some(seg) = fences.at(key.tri) {
                if seg.edges.contains(&edge) {
                    half = Some(if key.near {
                        seg.shared
                    } else if p == seg.shared {
                        q
                    } else {
                        p
                    });
                } else if key.near {
                    continue;
                }
            }
            let nears: &[bool] = match fences.at(next_tri) {
                Some(seg) if seg.edges.contains(&edge) => match half {
                    Some(v) => {
                        if v == seg.shared {
                            &[true]
                        } else {
                            &[false]
                        }
                    }
                    None => &[false, true],
                },
                _ => &[false],
            };
            let (pp, qq) = (arena.vertex_point(p), arena.vertex_point(q));
            let mid = Point::new((pp.x + qq.x) / 2.0, (pp.y + qq.y) / 2.0);
            for &near in nears {
                // The vertex the crossing keeps to once the fence on either
                // side has fixed it, so the portal is that half of the edge
                // and the taut path stays off the fence.
                let keep = half.or_else(|| {
                    fences.at(next_tri).and_then(|seg| {
                        seg.edges.contains(&edge).then(|| {
                            if near {
                                seg.shared
                            } else if p == seg.shared {
                                q
                            } else {
                                p
                            }
                        })
                    })
                });
                // Leaving through the edge `p` to `q` of a triangle on its
                // screen-left puts `q` on the traveller's left.
                let portal = match keep {
                    Some(v) if v == q => (qq, mid),
                    Some(_) => (mid, pp),
                    None => (qq, pp),
                };
                let through = Point::new(
                    (portal.0.x + portal.1.x) / 2.0,
                    (portal.0.y + portal.1.y) / 2.0,
                );
                let cost = item.cost + item.pos.distance_to(&through);
                let next = Key {
                    tri: next_tri,
                    mask: key.mask ^ cuts.bits[edge],
                    out,
                    near,
                };
                if best.get(&next).is_none_or(|&c| cost < c) {
                    best.insert(next, cost);
                    parent.insert(next, (key, portal));
                    heap.push(Item {
                        cost,
                        bound: cost + exit.at.distance_to(&through),
                        key: next,
                        pos: through,
                    });
                }
            }
        }
    }
    Err(Unroutable::Room)
}

/// A basis over Z_2 of the parities of closed walks, kept with distinct
/// leading bits in descending order so a vector reduces to zero exactly
/// when it is a sum of basis vectors.
#[derive(Default)]
struct Basis {
    rows: Vec<u64>,
}

impl Basis {
    fn reduce(&self, mut v: u64) -> u64 {
        for &r in &self.rows {
            v = v.min(v ^ r);
        }
        v
    }

    fn insert(&mut self, v: u64) {
        let v = self.reduce(v);
        if v != 0 {
            self.rows.push(v);
            self.rows.sort_unstable_by(|a, b| b.cmp(a));
        }
    }

    fn contains(&self, v: u64) -> bool {
        self.reduce(v) == 0
    }
}

/// Whether some walk from the entry to the exit under the guard rule has
/// a parity the move accepts. The parity of any walk is the parity along
/// a search tree's path to its last state plus a sum of closed walks, so
/// the closed walks met while the tree is grown form a basis and the
/// question is membership. The tree is grown over the moves the search
/// may make, so a parity this refuses is one no corridor has.
pub(super) fn parity_reachable(
    arena: &Arena,
    cuts: &Cuts,
    entry: &Entry,
    exit: &Entry,
    want: Want,
    is_loop: bool,
) -> bool {
    let index = |tri: usize, out: bool| tri * 2 + usize::from(out);
    let states = arena.triangle_count() * 2;
    let mut seen = vec![false; states];
    let mut along = vec![0u64; states];
    let mut basis = Basis::default();
    let mut queue = VecDeque::new();
    for &t in &entry.tris {
        let out = starts_out(arena, t, exit, is_loop);
        if !seen[index(t, out)] {
            seen[index(t, out)] = true;
            queue.push_back((t, out));
        }
    }
    while let Some((t, out)) = queue.pop_front() {
        let here = along[index(t, out)];
        let verts = arena.ccw_vertices(t);
        for k in 0..3 {
            let Some((next, edge)) = arena.across(t, verts[k], verts[(k + 1) % 3]) else {
                continue;
            };
            let Some(out) = step_out(arena, t, next, exit, out) else {
                continue;
            };
            let parity = here ^ cuts.bits[edge];
            let s = index(next, out);
            if seen[s] {
                basis.insert(parity ^ along[s]);
            } else {
                seen[s] = true;
                along[s] = parity;
                queue.push_back((next, out));
            }
        }
    }
    let Want::Parity(mask) = want else {
        return exit
            .tris
            .iter()
            .any(|&t| seen[index(t, false)] || seen[index(t, true)]);
    };
    exit.tris.iter().any(|&t| {
        [false, true].into_iter().any(|out| {
            let s = index(t, out);
            seen[s]
                && (basis.contains(mask ^ along[s]) || basis.contains(mask ^ cuts.all ^ along[s]))
        })
    })
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
