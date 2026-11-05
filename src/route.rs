//! Draws a face-model move as a curve inside its face.
//!
//! The face is eroded first: every wall moves inward by the line clearance
//! and every other node is cut out to the node clearance, and the move is
//! routed through what is left (src/route/erode.rs). A path through the
//! eroded face keeps the clearances by construction, so the two ends being
//! disconnected there is exactly "no room". The corridor search picks the
//! homotopy class the move names, the funnel pulls the path taut, an
//! optional relaxation moves it off the walls where the room allows, and
//! the spline is fit and read back through the embedding.

use std::collections::{BTreeMap, BTreeSet};
use std::rc::Rc;

use crate::ai::rank_node_placements;
use crate::curve;
use crate::embedding;
#[cfg(any(test, feature = "hash"))]
use crate::exact::Fnv;
use crate::face::{Component, FaceId, HalfEdgeId, Move as FaceMove, NodeId};
use crate::geometry::resample_polyline;
use crate::types::{
    ai_node_clearance, node_disc, scale, GameState, Move, Player, Point, AI_INTERSECTION_TOLERANCE,
    HUMAN_INTERSECTION_TOLERANCE, MIN_NODE_SPACING, MIN_NODE_SPACING_AI, MIN_PATH_CLEARANCE,
    MIN_PATH_CLEARANCE_AI,
};
use crate::validation::{validate_ai_move, validate_move};

mod clearance;
mod corridor;
mod erode;
mod small_loop;
#[cfg(any(test, feature = "bench"))]
pub mod sweep;
#[cfg(test)]
mod tests;

/// Counts of the router's work since the last `take`, for the probes.
#[cfg(test)]
pub(crate) mod stats {
    use std::cell::RefCell;

    #[derive(Debug, Default, Clone)]
    pub struct Stats {
        pub attempts: usize,
        pub arenas: usize,
        pub arena_ms: f64,
        /// Longest `route` or `route_cheap` call, its arena build included.
        pub call_ms_max: f64,
        /// Most states a corridor search that found its path held.
        pub states_ok_max: usize,
        /// Each parity search: bits to flip, bits in play, and whether it
        /// found its corridor.
        pub parity: Vec<(u32, u32, bool)>,
        /// Chords refused from the no room memo without a search.
        pub memo_hits: usize,
        /// Parity searches the fences held back from their corridor, run
        /// again without them.
        pub fence_misses: usize,
        /// Fenced corridors the relaxation or the fit refused, so the
        /// class's own corridor was searched for.
        pub fence_retries: usize,
        /// Parity searches the class's own corridor answered within
        /// EXACT_STATES, before any fence went up.
        pub exact_hits: usize,
        /// Milliseconds by phase: parity searches, tighten, relax, finish.
        pub phase_ms: [f64; 4],
        /// The move the last curve refused as Topology read back as.
        pub last_read_back: Option<crate::face::Move>,
        pub refused: super::sweep::Tally,
    }

    thread_local! {
        static STATS: RefCell<Stats> = RefCell::new(Stats::default());
    }

    pub fn with(f: impl FnOnce(&mut Stats)) {
        STATS.with(|s| f(&mut s.borrow_mut()));
    }

    pub fn take() -> Stats {
        STATS.with(|s| std::mem::take(&mut *s.borrow_mut()))
    }

    pub fn note_call(started: std::time::Instant) {
        let ms = started.elapsed().as_secs_f64() * 1000.0;
        with(|s| s.call_ms_max = s.call_ms_max.max(ms));
    }
}

/// Why a legal move could not be drawn.
#[derive(Debug, Clone, PartialEq)]
pub enum Unroutable {
    /// The eroded face has no path between the move's ends at these
    /// clearances, or no circle for an empty loop.
    Room,
    /// No point on the path keeps the node spacing from every node.
    NoSpot,
    /// The path found could not be drawn as one curve: tangled, touching
    /// itself, or too short for its node.
    Shape(&'static str),
    /// The drawn curve reads back as a different move.
    Topology,
    /// The first path puts pieces on the wrong side and the route was
    /// asked to stop short of the parity search.
    Sides,
    /// A failure the router should never produce in a face it found a path
    /// through; the sweep counts these and holds them at zero.
    Bug(&'static str),
}

/// Whether a route may run the parity search when its first path puts
/// pieces on the wrong side. The search is exponential in the pieces of
/// the face and is most of a refused route's cost.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    Search,
    Skip,
}

/// Whether the parity search runs with the fences up. Down is the retry
/// after a fenced corridor failed later on, in the relaxation or the fit.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Fencing {
    Up,
    Down,
}

/// Room wanted around every node, in node spacings, where the face allows
/// it, so a later loop at that node still has somewhere to go.
const NODE_ROOM: f64 = 2.0;

/// Within this many clearances of an end node the legs of a loop may run
/// as close together as they like.
const END_FREE: f64 = 3.0;

pub(crate) use crate::exact::cross;

/// Clearances the validator holds the player to move to.
#[derive(Clone, Copy)]
struct Rules {
    clearance: f64,
    /// Nearest the line comes to the centre of a node it does not touch.
    node_clearance: f64,
    /// Within this distance of a node a crossing of that node's own spoke
    /// changes nothing: the validator forgives it and the embedding reads
    /// the corner from where the curve leaves the MIN_PATH_CLEARANCE circle.
    /// A corner between two spokes is drawable while the fit margin fits
    /// between them at this radius.
    forgiven: f64,
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
            forgiven: AI_INTERSECTION_TOLERANCE.min(MIN_PATH_CLEARANCE) * s,
            fallback: MIN_PATH_CLEARANCE * s,
            spacing: MIN_NODE_SPACING_AI * s,
        },
        Player::Human => Rules {
            clearance: MIN_PATH_CLEARANCE * s,
            node_clearance: MIN_PATH_CLEARANCE * s,
            forgiven: HUMAN_INTERSECTION_TOLERANCE.min(MIN_PATH_CLEARANCE) * s,
            fallback: MIN_PATH_CLEARANCE * s,
            spacing: MIN_NODE_SPACING * s,
        },
    }
}

/// The curve for `mv`, drawn to the clearances of the player to move,
/// validated as that player and read back as `mv`.
pub fn route(state: &GameState, mv: &FaceMove) -> Result<Move, Unroutable> {
    Router::new().route(state, mv)
}

/// The bits of the three clearances an erosion is built to: the wall
/// clearance, the node disc and the forgiven radius.
type Clearances = (u64, u64, u64);
/// An eroded face by face, end pair and clearances.
type ArenaKey = (FaceId, NodeId, NodeId, Clearances);
/// A chord's corners and clearances.
type CornerKey = (
    FaceId,
    NodeId,
    Option<HalfEdgeId>,
    NodeId,
    Option<HalfEdgeId>,
    Clearances,
);

fn clearances(state: &GameState, want: &Rules) -> Clearances {
    (
        eroded_by(state, want).to_bits(),
        disc_of(state, want).to_bits(),
        want.forgiven.to_bits(),
    )
}

/// Work shared by every route of one position. An eroded face depends on
/// the face, the move's end nodes and the clearances, not on which pieces
/// the move encloses, so it is built once per those; a chord's ends being
/// disconnected in it refuses every chord between the same corners at the
/// same clearances. The keys do not cover the position itself: a Router
/// serves one `GameState` and is dropped with it.
#[derive(Default)]
pub struct Router {
    arenas: BTreeMap<ArenaKey, Result<Rc<erode::Arena>, Unroutable>>,
    no_room: BTreeSet<CornerKey>,
}

impl Router {
    pub fn new() -> Self {
        Self::default()
    }

    /// Corner pairs remembered as having no room.
    #[cfg(test)]
    pub(crate) fn corners_without_room(&self) -> usize {
        self.no_room.len()
    }

    /// The curve for `mv`, drawn to the clearances of the player to move,
    /// validated as that player and read back as `mv`.
    pub fn route(&mut self, state: &GameState, mv: &FaceMove) -> Result<Move, Unroutable> {
        #[cfg(test)]
        let started = std::time::Instant::now();
        let want = rules(state);
        let drawn = self.route_at(state, mv, &want, Parity::Search);
        // The disc clearance is what the AI aims for; a line that cannot
        // have it is still legal at the clearance a human line gets.
        let worth_retrying = matches!(
            drawn,
            Err(Unroutable::Room | Unroutable::NoSpot | Unroutable::Shape(_))
        );
        let drawn = if !worth_retrying || want.node_clearance <= want.fallback {
            drawn
        } else {
            self.route_cheap(state, mv)
        };
        #[cfg(test)]
        stats::note_call(started);
        drawn
    }

    /// `route` at the clearance a human line gets, one attempt.
    pub fn route_cheap(&mut self, state: &GameState, mv: &FaceMove) -> Result<Move, Unroutable> {
        self.route_cheap_with(state, mv, Parity::Search)
    }

    /// `route_cheap`, refusing with `Sides` instead of running the parity
    /// search when the first path puts pieces on the wrong side.
    pub fn route_cheap_with(
        &mut self,
        state: &GameState,
        mv: &FaceMove,
        parity: Parity,
    ) -> Result<Move, Unroutable> {
        #[cfg(test)]
        let started = std::time::Instant::now();
        let want = rules(state);
        let drawn = self.route_at(
            state,
            mv,
            &Rules {
                node_clearance: want.fallback,
                ..want
            },
            parity,
        );
        #[cfg(test)]
        stats::note_call(started);
        drawn
    }

    fn route_at(
        &mut self,
        state: &GameState,
        mv: &FaceMove,
        want: &Rules,
        parity: Parity,
    ) -> Result<Move, Unroutable> {
        let drawn = self.route_at_once(state, mv, want, parity);
        #[cfg(test)]
        stats::with(|s| {
            s.attempts += 1;
            if let Err(why) = &drawn {
                s.refused.count(why);
            }
        });
        drawn
    }

    fn route_at_once(
        &mut self,
        state: &GameState,
        mv: &FaceMove,
        want: &Rules,
        parity: Parity,
    ) -> Result<Move, Unroutable> {
        if mv.a == mv.b && mv.enclosed.is_empty() {
            return small_loop::route(state, mv, want.clearance, want.node_clearance);
        }
        let (arena, path, fenced) = plan(self, state, mv, want, parity, Fencing::Up)?;
        let drawn = self.draw(state, mv, want, &arena, path);
        // A fenced corridor is the shortest the fences allow, not the
        // shortest in its class, and the relaxation or the fit can refuse
        // it where the class's own corridor draws; that one is searched
        // for once, and the fenced refusal stands if the search gives out.
        if !fenced || matches!(drawn, Ok(_) | Err(Unroutable::Room)) {
            return drawn;
        }
        #[cfg(test)]
        stats::with(|s| s.fence_retries += 1);
        match plan(self, state, mv, want, parity, Fencing::Down) {
            Ok((arena, path, _)) => self.draw(state, mv, want, &arena, path),
            Err(Unroutable::Bug("search states")) => drawn,
            Err(why) => Err(why),
        }
    }

    /// The path relaxed off the walls and fitted, or the taut path fitted
    /// when the relaxation ends up somewhere it cannot be used.
    fn draw(
        &self,
        state: &GameState,
        mv: &FaceMove,
        want: &Rules,
        arena: &erode::Arena,
        path: Vec<Point>,
    ) -> Result<Move, Unroutable> {
        // Uncrossing a figure of eight joins its lobes, and a piece inside
        // a lobe changes sides: that path is another move's, so the
        // corridor has no drawing of this one.
        if !sides_match(state, mv, &path) {
            return Err(Unroutable::Shape("twisted"));
        }
        #[cfg(test)]
        let t = std::time::Instant::now();
        // The relaxation can carry the path past a node; the taut path is
        // drawn instead when it has.
        let relaxed = relax(state, mv, want, arena, &path).filter(|r| sides_match(state, mv, r));
        #[cfg(test)]
        stats::with(|s| s.phase_ms[2] += t.elapsed().as_secs_f64() * 1000.0);
        #[cfg(test)]
        let t = std::time::Instant::now();
        let out = match relaxed {
            Some(relaxed) => finish(state, mv, relaxed).or_else(|_| finish(state, mv, path)),
            None => finish(state, mv, path),
        };
        #[cfg(test)]
        stats::with(|s| s.phase_ms[3] += t.elapsed().as_secs_f64() * 1000.0);
        out
    }

    /// `face` eroded for a move between `mv`'s ends, built once.
    fn arena(
        &mut self,
        state: &GameState,
        face: FaceId,
        mv: &FaceMove,
        want: &Rules,
    ) -> Result<Rc<erode::Arena>, Unroutable> {
        let key = (face, mv.a, mv.b, clearances(state, want));
        if let Some(hit) = self.arenas.get(&key) {
            return hit.clone();
        }
        #[cfg(test)]
        let started = std::time::Instant::now();
        let c = eroded_by(state, want);
        let disc = disc_of(state, want);
        let built =
            erode::Arena::build(state, face, mv.a, mv.b, c, disc, want.forgiven).map(Rc::new);
        #[cfg(test)]
        stats::with(|s| {
            s.arenas += 1;
            s.arena_ms += started.elapsed().as_secs_f64() * 1000.0;
        });
        self.arenas.insert(key, built.clone());
        built
    }
}

/// The wall clearance the face is eroded by: the fit may move the curve a
/// tolerance off the path, and the walls are flattenings a tolerance off
/// their curves.
fn eroded_by(state: &GameState, want: &Rules) -> f64 {
    want.clearance + curve::tolerance(state.board_size)
}

/// The disc cut out round every other node: the fit may move the curve a
/// tolerance off the path at a disc too.
fn disc_of(state: &GameState, want: &Rules) -> f64 {
    want.node_clearance + curve::tolerance(state.board_size)
}

/// The taut path for `mv` through the eroded face, in the homotopy class
/// the move names, with its pinches parted and its crossings undone.
fn plan(
    router: &mut Router,
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    parity: Parity,
    fencing: Fencing,
) -> Result<(Rc<erode::Arena>, Vec<Point>, bool), Unroutable> {
    let (arena, path, fenced) = taut_path(router, state, mv, want, parity, fencing)?;
    let path = split_pinches(path, eroded_by(state, want) / 4.0, |q| arena.free_at(q));
    let path = settle(path)?;
    Ok((arena, path, fenced))
}

/// Whether `path` puts every piece on the side `mv` names, read as the
/// embedding reads it: a chord between two pieces encloses nothing, and
/// a loop at a bare node may read as the other side.
fn sides_match(state: &GameState, mv: &FaceMove, path: &[Point]) -> bool {
    let (Ok(own), Ok(other)) = (
        state.face.component_of(mv.a, mv.at_a),
        state.face.component_of(mv.b, mv.at_b),
    ) else {
        return false;
    };
    if own != other {
        return true;
    }
    let mut got = embedding::sides(state, mv.a, mv.at_a, mv.b, mv.at_b, own, path);
    got.sort_unstable();
    let mut want = mv.enclosed.clone();
    want.sort_unstable();
    let bare_loop = mv.a == mv.b && mv.at_a.is_none();
    got == want || (bare_loop && got == other_side(state, mv))
}

/// The path with a doubled-back leg cut and its crossings undone, or
/// tangled.
fn settle(path: Vec<Point>) -> Result<Vec<Point>, Unroutable> {
    uncross(untwist_legs(path)).ok_or(Unroutable::Shape("tangled"))
}

/// The taut path for `mv` through the eroded face as the funnel leaves it,
/// and whether a parity search with the fences up produced it.
fn taut_path(
    router: &mut Router,
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    parity: Parity,
    fencing: Fencing,
) -> Result<(Rc<erode::Arena>, Vec<Point>, bool), Unroutable> {
    let face = state
        .face
        .corner_face(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Bug("corner"))?;
    let corners = (face, mv.a, mv.at_a, mv.b, mv.at_b, clearances(state, want));
    if router.no_room.contains(&corners) {
        #[cfg(test)]
        stats::with(|s| s.memo_hits += 1);
        return Err(Unroutable::Room);
    }
    let arena = router.arena(state, face, mv, want)?;
    let is_loop = mv.a == mv.b;
    // A corner with no free triangle refuses every move from it, and a
    // chord's Room means the free triangles disconnect its ends, which
    // refuses every chord between those corners. A loop's Room means no
    // closed walk from the node crosses a cut, which only the parity
    // search decides.
    let mut refuse_corners = |why: Unroutable| {
        if why == Unroutable::Room {
            router.no_room.insert(corners);
        }
        why
    };
    let entry = arena
        .entries(state, mv.a, mv.at_a)
        .map_err(&mut refuse_corners)?;
    let exit = arena
        .entries(state, mv.b, mv.at_b)
        .map_err(&mut refuse_corners)?;
    let blocked = corridor::blocked(&arena, &entry, &exit);
    let cuts = arena.cuts(state, mv, face, &blocked)?;
    let own = state
        .face
        .component_of(mv.a, mv.at_a)
        .map_err(|_| Unroutable::Bug("corner"))?;
    let first = corridor::search(
        &arena,
        &cuts,
        &entry,
        &exit,
        corridor::Want::Any,
        is_loop,
        corridor::Limits {
            fences: &erode::Fences::none(),
            cap: corridor::SEARCH_STATES,
        },
    );
    let first = match first {
        Err(Unroutable::Room) if !is_loop => {
            // A path the search missed is a bug.
            if corridor::reachable(&arena, &entry, &exit, is_loop) {
                return Err(Unroutable::Bug("room with a path"));
            }
            return Err(refuse_corners(Unroutable::Room));
        }
        other => other?,
    };
    #[cfg(test)]
    let t = std::time::Instant::now();
    let mut path = arena.tighten(&cuts, &corridor::draw(&first, &entry, &exit), &entry, &exit);
    #[cfg(test)]
    stats::with(|s| s.phase_ms[1] += t.elapsed().as_secs_f64() * 1000.0);
    let mut fenced = false;
    if cuts.all != 0 {
        let wrong = wrong_sides(state, mv, own, &cuts, &path);
        if wrong != 0 {
            if parity == Parity::Skip {
                return Err(Unroutable::Sides);
            }
            // Flipping every bit at once is the same move drawn round the
            // other side of its own piece, and a bare node has no sides.
            let want = corridor::Want::Parity(first.parity ^ wrong);
            let second =
                parity_search(&arena, &cuts, &entry, &exit, want, is_loop, fencing, || {
                    arena.fences(state, mv, face)
                });
            fenced = second.as_ref().is_ok_and(|(_, by_fences)| *by_fences);
            let second = second.map(|(found, _)| found);
            #[cfg(test)]
            stats::with(|s| {
                s.parity
                    .push((wrong.count_ones(), cuts.all.count_ones(), second.is_ok()))
            });
            let second = second?;
            #[cfg(test)]
            let t = std::time::Instant::now();
            path = arena.tighten(
                &cuts,
                &corridor::draw(&second, &entry, &exit),
                &entry,
                &exit,
            );
            #[cfg(test)]
            stats::with(|s| s.phase_ms[1] += t.elapsed().as_secs_f64() * 1000.0);
        }
    }
    Ok((arena, path, fenced))
}

/// States the search for the class's own corridor may hold before the
/// fences go up: enough for the faces of the seeded sweeps, where the
/// corridors found peak near 5,000, and a few milliseconds in a face too
/// crowded for it.
const EXACT_STATES: usize = 20_000;

/// The corridor in the parity class `want`, and whether fences shaped it:
/// refused as Room when no walk through the arena reaches that class.
/// With the fences up, the class's own corridor is searched for without
/// fences within EXACT_STATES; else the corridor the fences allow, which
/// keep the search to a few parities; else, when a fence stood in the
/// only way through, the search without any under the full cap. With the
/// fences down, the retry after a fenced corridor failed later, that
/// last search runs alone.
#[allow(clippy::too_many_arguments)]
fn parity_search(
    arena: &erode::Arena,
    cuts: &erode::Cuts,
    entry: &erode::Entry,
    exit: &erode::Entry,
    want: corridor::Want,
    is_loop: bool,
    fencing: Fencing,
    fences: impl FnOnce() -> erode::Fences,
) -> Result<(corridor::Corridor, bool), Unroutable> {
    #[cfg(test)]
    let started = std::time::Instant::now();
    if !corridor::parity_reachable(arena, cuts, entry, exit, want, is_loop) {
        return Err(Unroutable::Room);
    }
    let none = erode::Fences::none();
    if fencing == Fencing::Up {
        let exact = corridor::search(
            arena,
            cuts,
            entry,
            exit,
            want,
            is_loop,
            corridor::Limits {
                fences: &none,
                cap: EXACT_STATES,
            },
        );
        if let Ok(found) = exact {
            #[cfg(test)]
            stats::with(|s| {
                s.exact_hits += 1;
                s.phase_ms[0] += started.elapsed().as_secs_f64() * 1000.0;
            });
            return Ok((found, false));
        }
        let fenced = corridor::search(
            arena,
            cuts,
            entry,
            exit,
            want,
            is_loop,
            corridor::Limits {
                fences: &fences(),
                cap: corridor::SEARCH_STATES,
            },
        );
        #[cfg(test)]
        stats::with(|s| s.phase_ms[0] += started.elapsed().as_secs_f64() * 1000.0);
        if let Ok(found) = fenced {
            return Ok((found, true));
        }
        #[cfg(test)]
        stats::with(|s| s.fence_misses += 1);
    }
    corridor::search(
        arena,
        cuts,
        entry,
        exit,
        want,
        is_loop,
        corridor::Limits {
            fences: &none,
            cap: corridor::SEARCH_STATES,
        },
    )
    .map(|found| (found, false))
}

/// Whether the parity `mv` names is one no corridor reaches, as the
/// filter and the exhaustive search without fences each decide it; the
/// two must agree. None when the first search finds no path at all or
/// the first path already has every piece on its side.
#[cfg(test)]
pub(crate) fn parity_unreachable(state: &GameState, mv: &FaceMove, cheap: bool) -> Option<bool> {
    let full = rules(state);
    let want = if cheap {
        Rules {
            node_clearance: full.fallback,
            ..full
        }
    } else {
        full
    };
    let mut router = Router::new();
    let face = state.face.corner_face(mv.a, mv.at_a).ok()?;
    let arena = router.arena(state, face, mv, &want).ok()?;
    let is_loop = mv.a == mv.b;
    let entry = arena.entries(state, mv.a, mv.at_a).ok()?;
    let exit = arena.entries(state, mv.b, mv.at_b).ok()?;
    let blocked = corridor::blocked(&arena, &entry, &exit);
    let cuts = arena.cuts(state, mv, face, &blocked).ok()?;
    let own = state.face.component_of(mv.a, mv.at_a).ok()?;
    let none = erode::Fences::none();
    let first = corridor::search(
        &arena,
        &cuts,
        &entry,
        &exit,
        corridor::Want::Any,
        is_loop,
        corridor::Limits {
            fences: &none,
            cap: corridor::SEARCH_STATES,
        },
    )
    .ok()?;
    let path = arena.tighten(&cuts, &corridor::draw(&first, &entry, &exit), &entry, &exit);
    let wrong = wrong_sides(state, mv, own, &cuts, &path);
    if wrong == 0 {
        return None;
    }
    let wanted = corridor::Want::Parity(first.parity ^ wrong);
    let filter = corridor::parity_reachable(&arena, &cuts, &entry, &exit, wanted, is_loop);
    let exhaustive = corridor::search(
        &arena,
        &cuts,
        &entry,
        &exit,
        wanted,
        is_loop,
        corridor::Limits {
            fences: &none,
            cap: corridor::SEARCH_STATES,
        },
    );
    assert_eq!(
        filter,
        exhaustive.is_ok(),
        "the parity filter and the exhaustive search disagree: {:?}",
        exhaustive.as_ref().err()
    );
    Some(!filter)
}

/// Two legs of a path the funnel pulled through the same vertex are set a
/// little apart there, so they leave the vertex on different sides and the
/// relaxation can open them further instead of crossing them: a leg moves
/// to the left of its way through, a hairpin back the way it came, each
/// only where that is free. Legs through one vertex the same way are left
/// as they are.
fn split_pinches(mut path: Vec<Point>, by: f64, free: impl Fn(Point) -> bool) -> Vec<Point> {
    let n = path.len();
    let unit = |p: Point, q: Point| {
        let (dx, dy) = (q.x - p.x, q.y - p.y);
        let len = (dx * dx + dy * dy).sqrt();
        (len >= 1e-9).then(|| (dx / len, dy / len))
    };
    // A pass through vertex k: the way in and the way out, as unit vectors.
    let pass =
        |path: &[Point], k: usize| Some((unit(path[k - 1], path[k])?, unit(path[k], path[k + 1])?));
    let same_way = |a: ((f64, f64), (f64, f64)), b: ((f64, f64), (f64, f64))| {
        a.0 .0 * b.0 .0 + a.0 .1 * b.0 .1 > 0.0 && a.1 .0 * b.1 .0 + a.1 .1 * b.1 .1 > 0.0
    };
    let interior: Vec<usize> = (1..n.saturating_sub(1)).collect();
    let mut moved: Vec<Option<Point>> = vec![None; n];
    for &i in &interior {
        let Some(this) = pass(&path, i) else {
            continue;
        };
        let pinched = interior.iter().any(|&j| {
            j != i
                && path[i].distance_to(&path[j]) < 1e-9
                && pass(&path, j).is_some_and(|other| !same_way(this, other))
        });
        if !pinched {
            continue;
        }
        let ((ax, ay), (bx, by_)) = this;
        let (sx, sy) = (ax + bx, ay + by_);
        let through = (sx * sx + sy * sy).sqrt();
        // Left of the way through, or straight back out of a hairpin.
        let (dx, dy) = if through >= 1e-6 {
            (sy / through, -sx / through)
        } else {
            (-ax, -ay)
        };
        let to = Point::new(path[i].x + dx * by, path[i].y + dy * by);
        moved[i] = free(to).then_some(to);
    }
    for (i, at) in moved.into_iter().enumerate() {
        if let Some(at) = at {
            path[i] = at;
        }
    }
    path
}

/// The path moved off the walls and nodes as far as the room allows.
fn relax(
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    arena: &erode::Arena,
    path: &[Point],
) -> Option<Vec<Point>> {
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    let nodes: Vec<Point> = state
        .nodes
        .iter()
        .filter(|n| n.id != mv.a && n.id != mv.b)
        .map(|n| n.position)
        .collect();
    // The relaxation settles a little short of its room, so the room sits a
    // line clearance above what the validator demands.
    let room = (NODE_ROOM * want.spacing).max(want.node_clearance + want.clearance);
    // The fit may move the curve a tolerance toward a node, and a settling
    // game's grid a half diagonal more.
    let rooms = clearance::Rooms {
        clearance: eroded_by(state, want),
        node_clearance: want.node_clearance
            + curve::tolerance(state.board_size)
            + erode::grid_snap(state),
        node_room: room,
        end_room: want.spacing,
    };
    let obstacles = clearance::Obstacles::new(arena.walls(), nodes, ends, rooms, path);
    let relaxed = clearance::push(path.to_vec(), &obstacles)?;
    // The pushes move points, so two clear points can leave a segment
    // between them skimming a wall, and the fit, the flattening and the
    // stored grid can each move the curve further; every segment is
    // sampled against what the validator will forgive, and a path that
    // fails anywhere gives way to the taut one.
    let tolerance = curve::tolerance(state.board_size);
    let snap = erode::grid_snap(state);
    let margin = 2.0 * tolerance + snap;
    let node_clearance = want.clearance + tolerance + snap;
    let sampled = clearance::resample(&relaxed, clearance::SPACING * rooms.clearance);
    sampled
        .iter()
        .all(|q| arena.keeps_validity(*q, margin, node_clearance))
        .then_some(relaxed)
}

/// Feeds the flattening and node of every routed search move of `state`
/// into `fnv`, in the model's move order.
#[cfg(any(test, feature = "hash"))]
pub fn hash_routes(fnv: &mut Fnv, state: &GameState) {
    for mv in state.face.search_moves() {
        hash_route(fnv, state, &mv);
    }
}

/// Feeds the flattening and node of `mv` routed in `state` into `fnv`.
#[cfg(any(test, feature = "hash"))]
pub fn hash_route(fnv: &mut Fnv, state: &GameState, mv: &FaceMove) {
    match route(state, mv) {
        Ok(drawn) => {
            fnv.points(&drawn.flatten(curve::tolerance(state.board_size)));
            fnv.points(&[drawn.new_node_pos]);
        }
        // A refusal is part of the output too: a platform that turns
        // one into a route, or the other way, has to show.
        Err(why) => fnv.feed(match why {
            Unroutable::Room => 1.0,
            Unroutable::NoSpot => 2.0,
            Unroutable::Shape(_) => 3.0,
            Unroutable::Topology => 4.0,
            Unroutable::Bug(_) => 5.0,
            Unroutable::Sides => 6.0,
        }),
    }
}

/// Bits of the pieces the path puts on the wrong side of the move.
fn wrong_sides(
    state: &GameState,
    mv: &FaceMove,
    own: Component,
    cuts: &erode::Cuts,
    path: &[Point],
) -> u64 {
    let got = embedding::sides(state, mv.a, mv.at_a, mv.b, mv.at_b, own, path);
    cuts.piece_bits
        .iter()
        .filter(|(comp, _)| {
            let rep = state.face.lowest_node(*comp);
            got.contains(&rep) != mv.enclosed.contains(&rep)
        })
        .fold(0, |acc, (_, bit)| acc | bit)
}

/// Fits the spline, places the node, validates, and checks the curve reads
/// back as the move. A loop at a bare node may read as the other side.
pub(crate) fn finish(
    state: &GameState,
    mv: &FaceMove,
    path: Vec<Point>,
) -> Result<Move, Unroutable> {
    let tolerance = curve::tolerance(state.board_size);
    let Rules {
        clearance, spacing, ..
    } = rules(state);
    // A point on a path of length L is within L/2 of one end, so a path
    // under two spacings has nowhere for its node.
    let length: f64 = path.windows(2).map(|w| w[0].distance_to(&w[1])).sum();
    if length < 2.0 * spacing {
        return Err(Unroutable::Shape("short"));
    }
    // The fit measures its error at the points alone, so a long straight
    // run gets points along it to hold the cubic down. The face is eroded
    // by one fit tolerance, so nothing else may move the path before the
    // fit, and Move::new refits the stored points at this tolerance.
    let path = densify(&path, clearance);
    let segments = curve::fit(&path, tolerance);
    let flat = curve::flatten(&segments, tolerance);
    let node = place_node(state, &segments, &flat, spacing).ok_or(Unroutable::NoSpot)?;
    let ends = [state.nodes[mv.a].position, state.nodes[mv.b].position];
    // Two passes the relaxation could not pull apart touch, and the fit
    // can cross where they touch: no room for both.
    if self_crossing(&flat) || self_touching(&flat, ends, clearance) {
        return Err(Unroutable::Shape("touching"));
    }
    let mov = Move::from_segments(mv.a, mv.b, path, segments, node, state.current_player);
    // The checks run on the move as the board will store it: a settling
    // game rounds it to the grid before apply looks at it.
    let stored = state.entering(mov.clone());
    match state.current_player {
        Player::AI => validate_ai_move(state, &stored),
        Player::Human => validate_move(state, &stored),
    }
    .map_err(|r| Unroutable::Bug(kind_name(r.kind)))?;
    let read = embedding::embed(state, &stored).map_err(|r| Unroutable::Bug(kind_name(r.kind)))?;
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
        #[cfg(test)]
        stats::with(|s| s.last_read_back = Some(read.clone()));
        Err(Unroutable::Topology)
    }
}

fn kind_name(kind: crate::validation::RejectionKind) -> &'static str {
    use crate::validation::RejectionKind::*;
    match kind {
        PathTooCloseToNode => "PathTooCloseToNode",
        PathCrossesLine => "PathCrossesLine",
        NodeTooClose => "NodeTooClose",
        PathTooShort => "PathTooShort",
        PathAlongLine => "PathAlongLine",
        _ => "Rejected",
    }
}

/// A leg the taut path doubles back across is cut at the crossing, so the
/// path runs from the end node straight to where it was heading.
fn untwist_legs(mut path: Vec<Point>) -> Vec<Point> {
    let crossing = |a: Point, b: Point, c: Point, d: Point| {
        clearance::proper_crossing(a, b, c, d)
            .then(|| crate::geometry::segment_intersection(&a, &b, &c, &d))
            .flatten()
    };
    'again: loop {
        let n = path.len().saturating_sub(1);
        for j in 2..n {
            if let Some(x) = crossing(path[0], path[1], path[j], path[j + 1]) {
                let mut out = vec![path[0], x];
                out.extend_from_slice(&path[j + 1..]);
                path = out;
                continue 'again;
            }
        }
        for i in 0..n.saturating_sub(2) {
            if let Some(x) = crossing(path[i], path[i + 1], path[n - 1], path[n]) {
                let mut out = path[..=i].to_vec();
                out.push(x);
                out.push(path[n]);
                path = out;
                continue 'again;
            }
        }
        return path;
    }
}

/// Rounds of uncrossing before a path is given up as tangled.
const UNCROSS_ROUNDS: usize = 8;

/// The shortest path in a parity class can twist into a figure of eight.
/// Reversing the part between the two visits of a crossing joins the
/// lobes into one simple curve through the same free triangles; the
/// relaxation then pulls the two visits apart. None when the path stays
/// tangled.
fn uncross(mut path: Vec<Point>) -> Option<Vec<Point>> {
    for _ in 0..UNCROSS_ROUNDS {
        let n = path.len() - 1;
        let hit = (0..n).find_map(|i| {
            (i + 2..n).find_map(|j| {
                clearance::proper_crossing(path[i], path[i + 1], path[j], path[j + 1])
                    .then(|| {
                        crate::geometry::segment_intersection(
                            &path[i],
                            &path[i + 1],
                            &path[j],
                            &path[j + 1],
                        )
                    })
                    .flatten()
                    .map(|x| (i, j, x))
            })
        });
        let Some((i, j, x)) = hit else {
            return Some(path);
        };
        let mut out = path[..=i].to_vec();
        out.push(x);
        out.extend(path[i + 1..=j].iter().rev());
        out.push(x);
        out.extend_from_slice(&path[j + 1..]);
        path = out;
    }
    None
}

/// The path with points added along any segment longer than `step`, the
/// original points kept.
fn densify(path: &[Point], step: f64) -> Vec<Point> {
    let mut out = Vec::with_capacity(path.len());
    for w in path.windows(2) {
        let (p, q) = (w[0], w[1]);
        let pieces = (p.distance_to(&q) / step).ceil().max(1.0) as usize;
        for k in 0..pieces {
            let t = k as f64 / pieces as f64;
            out.push(Point::new(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t));
        }
    }
    if let Some(last) = path.last() {
        out.push(*last);
    }
    out
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
/// board edge; none when the curve offers no such spot. Spots are measured
/// where the move will snap them, on the curve itself.
fn place_node(
    state: &GameState,
    segments: &[curve::Cubic],
    flat: &[Point],
    spacing: f64,
) -> Option<Point> {
    let s = scale(state.board_size);
    let dense = resample_polyline(flat, MIN_PATH_CLEARANCE_AI * s / 2.0);
    let inset = MIN_NODE_SPACING * s;
    let far = state.board_size as f64 - inset;
    let ranked: Vec<Point> = rank_node_placements(&dense, state)
        .into_iter()
        .map(|q| curve::nearest(segments, &q).map_or(q, |hit| hit.point))
        .collect();
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
    // The validator's test is a strict less-than on a spot snapped twice.
    spot(spacing.max(2.0 * node_disc(state.board_size))).or_else(|| spot(spacing + 1e-6))
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
