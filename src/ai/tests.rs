use std::time::Instant;

use super::*;
use crate::route::stats;

/// One AI reply of a self-played game, with the router's work behind it.
struct Row {
    index: usize,
    candidates: usize,
    passed: usize,
    drawn: bool,
    search_ms: f64,
    draw_ms: f64,
    stats: stats::Stats,
}

impl Row {
    fn total_ms(&self) -> f64 {
        self.search_ms + self.draw_ms
    }
}

/// Plays the AI against itself from `state` for up to `moves` replies and
/// measures each one the way the page sees it: the ranking, then the
/// drawing.
fn self_play(mut state: GameState, moves: usize) -> Vec<Row> {
    let mut rows = Vec::new();
    for index in 0..moves {
        state.current_player = Player::AI;
        let candidates = state.face.search_moves();
        if candidates.is_empty() {
            break;
        }
        stats::take();
        let t = Instant::now();
        let order = rank(&state, &candidates);
        let search_ms = t.elapsed().as_secs_f64() * 1000.0;
        let t = Instant::now();
        let (drawn, passed) = draw(&state, &candidates, &order);
        let draw_ms = t.elapsed().as_secs_f64() * 1000.0;
        rows.push(Row {
            index,
            candidates: candidates.len(),
            passed,
            drawn: drawn.is_ok(),
            search_ms,
            draw_ms,
            stats: stats::take(),
        });
        let Ok(mov) = drawn else {
            break;
        };
        state.apply_move(mov).unwrap();
    }
    rows
}

/// The found and failed parity searches as (bits to flip, bits in play)
/// pairs, each with its count.
fn parity_summary(parity: &[(u32, u32, bool)]) -> String {
    let mut found = std::collections::BTreeMap::new();
    let mut failed = std::collections::BTreeMap::new();
    for &(flip, all, ok) in parity {
        *if ok { &mut found } else { &mut failed }
            .entry((flip, all))
            .or_insert(0usize) += 1;
    }
    format!("found {found:?} failed {failed:?}")
}

fn print_rows(title: &str, rows: &[Row]) {
    eprintln!("\n{title}");
    eprintln!(
        "{:>4} {:>9} {:>6} {:>5} {:>9} {:>9} {:>8} {:>7} {:>9} {:>5} {:>6} {:>6} {:>5} {:>5}",
        "move",
        "total ms",
        "search",
        "draw",
        "listed",
        "passed",
        "attempts",
        "arenas",
        "arena ms",
        "room",
        "nospot",
        "shape",
        "topo",
        "bug"
    );
    for r in rows {
        let s = &r.stats;
        eprintln!(
            "{:>4} {:>9.0} {:>6.0} {:>5.0} {:>9} {:>9} {:>8} {:>7} {:>9.0} {:>5} {:>6} {:>6} {:>5} {:>5}{}",
            r.index,
            r.total_ms(),
            r.search_ms,
            r.draw_ms,
            r.candidates,
            r.passed,
            s.attempts,
            s.arenas,
            s.arena_ms,
            s.refused.room,
            s.refused.no_spot,
            s.refused.shape.values().sum::<usize>(),
            s.refused.topology,
            s.refused.bug,
            if r.drawn { "" } else { " undrawn" }
        );
        if !s.refused.bugs.is_empty() || !s.refused.shape.is_empty() {
            eprintln!("     bugs {:?} shape {:?}", s.refused.bugs, s.refused.shape);
        }
        eprintln!(
            "     most states in a corridor found {}; parity searches {}",
            s.states_ok_max,
            parity_summary(&s.parity)
        );
    }
    let worst = rows.iter().map(Row::total_ms).fold(0.0, f64::max);
    eprintln!("worst reply {worst:.0} ms over {} replies", rows.len());
}

/// A clock that already reads `ms`.
struct Stopped(f64);

impl Clock for Stopped {
    fn elapsed_ms(&mut self) -> f64 {
        self.0
    }
}

/// The 24-node opening on the 1000 board after `moves` moves, each the
/// listed move enclosing the most pieces its player can draw, so the
/// board splits into regions, with the AI to move.
fn opening_after(moves: usize) -> GameState {
    let mut state = GameState::with_board_size(24, 1000);
    for k in 0..moves {
        state.current_player = if k % 2 == 0 {
            Player::Human
        } else {
            Player::AI
        };
        let mut listed = state.face.search_moves();
        listed.sort_by_key(|m| std::cmp::Reverse(m.enclosed.len()));
        let mov = listed
            .iter()
            .find_map(|m| route::route(&state, m).ok())
            .expect("a drawable move");
        state.apply_move(mov).unwrap();
    }
    state.current_player = Player::AI;
    state
}

/// A clock that reads zero for `calls` readings and past the draw limit
/// after that.
struct Expiring {
    calls: usize,
}

impl Clock for Expiring {
    fn elapsed_ms(&mut self) -> f64 {
        if self.calls == 0 {
            return DRAW_LIMIT_MS + 1.0;
        }
        self.calls -= 1;
        0.0
    }
}

/// Past the draw limit nothing is routed and the draw is out of time.
#[test]
fn past_the_draw_limit_the_rest_of_the_ranking_goes_untried() {
    let state = GameState::with_board_size(4, 1000);
    let candidates = state.face.search_moves();
    let order: Vec<usize> = (0..candidates.len()).collect();
    stats::take();
    let (drawn, passed) = draw_within(&state, &candidates, &order, Stopped(DRAW_LIMIT_MS + 1.0));
    assert!(matches!(drawn, Err(Undrawn::OutOfTime)));
    assert_eq!(passed, 0);
    assert_eq!(stats::take().attempts, 0);
}

/// The limit holds inside one candidate's class as well: when the clock
/// passes it after the first attempt, the other members go untried.
#[test]
fn the_draw_limit_stops_a_class_walk_between_members() {
    let state = opening_after(2);
    let candidates = state.face.search_moves();
    let refused = candidates
        .iter()
        .position(|m| route::route(&state, m).is_err() && !class_members(&state, m).is_empty())
        .expect("a refused move with class members");
    stats::take();
    let (drawn, passed) = draw_within(&state, &candidates, &[refused], Expiring { calls: 1 });
    assert!(matches!(drawn, Err(Undrawn::OutOfTime)));
    assert_eq!(passed, 0);
    let attempts = stats::take().attempts;
    assert!(attempts <= 2, "{attempts} attempts for one member");
}

/// Between the route budget and the draw limit every candidate gets its
/// one cheap attempt, and the opening still draws.
#[test]
fn past_the_route_budget_each_candidate_gets_one_cheap_attempt() {
    let state = GameState::with_board_size(4, 1000);
    let candidates = state.face.search_moves();
    let order: Vec<usize> = (0..candidates.len()).collect();
    stats::take();
    let (drawn, passed) = draw_within(&state, &candidates, &order, Stopped(ROUTE_BUDGET_MS + 1.0));
    assert!(drawn.is_ok());
    let attempts = stats::take().attempts;
    assert_eq!(attempts, passed + 1);
}

/// Every listed move refused, with time to spare, is Exhausted. The sealed
/// board hands the AI one node inside a loop too small for any line.
#[test]
fn every_listed_move_refused_is_exhausted() {
    let fixture = crate::replay::load(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures/undrawn/sealed-2n-1000.json"),
    )
    .unwrap();
    let state = crate::replay::play_as(&fixture, Player::AI);
    assert_eq!(state.current_player, Player::AI);
    let candidates = state.face.search_moves();
    assert!(!candidates.is_empty());
    let order: Vec<usize> = (0..candidates.len()).collect();
    let (drawn, passed) = draw_within(&state, &candidates, &order, Stopped(0.0));
    assert!(matches!(drawn, Err(Undrawn::Exhausted)));
    assert_eq!(passed, candidates.len());
}

/// A shared Router answers every candidate as a fresh one does.
#[test]
fn a_shared_router_answers_as_fresh_routers_do() {
    let state = opening_after(3);
    let candidates = state.face.search_moves();
    let mut shared = route::Router::new();
    let mut compared = 0;
    for mv in candidates.iter().take(40) {
        let together = shared.route(&state, mv);
        let alone = route::route(&state, mv);
        match (&together, &alone) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.segments, b.segments, "{mv:?}");
                assert_eq!(a.new_node_pos, b.new_node_pos, "{mv:?}");
            }
            (Err(a), Err(b)) => assert_eq!(a, b, "{mv:?}"),
            _ => panic!("{mv:?}: shared {together:?}, fresh {alone:?}"),
        }
        compared += 1;
    }
    assert!(compared >= 20, "{compared} compared");
}

/// A chord whose ends the arena disconnects puts its corners in the no
/// room memo, and the next chord between those corners is refused without
/// another arena or search.
#[test]
fn a_second_chord_between_corners_without_room_is_refused_from_memory() {
    for moves in [6, 8, 10, 12] {
        let state = opening_after(moves);
        let candidates = state.face.search_moves();
        for first in candidates.iter().filter(|m| m.a != m.b) {
            let mut router = route::Router::new();
            let refused = router.route_cheap(&state, first);
            if router.corners_without_room() == 0 {
                continue;
            }
            assert!(matches!(refused, Err(route::Unroutable::Room)));
            let Some(second) = candidates.iter().find(|m| {
                *m != first
                    && m.a == first.a
                    && m.at_a == first.at_a
                    && m.b == first.b
                    && m.at_b == first.at_b
            }) else {
                continue;
            };
            stats::take();
            assert!(matches!(
                router.route_cheap(&state, second),
                Err(route::Unroutable::Room)
            ));
            let again = stats::take();
            assert_eq!(again.attempts, 1);
            assert_eq!(again.arenas, 0);
            assert_eq!(again.parity.len(), 0);
            assert_eq!(router.corners_without_room(), 1);
            return;
        }
    }
    panic!("no chord with disconnected ends and a sibling between the same corners");
}

/// Work the draw does on the pinned position under a stopped clock past
/// the route budget, one cheap attempt per candidate, with the most
/// enclosing moves first as the ranking tends to put them: 32 attempts
/// and 5 arena builds measured, 31 candidates passed over out of 60; the
/// bounds are twice that.
const PINNED_ATTEMPTS: usize = 64;
const PINNED_ARENAS: usize = 10;

/// The 24-node opening after four splitting moves draws under the cheap
/// pass within its measured work, most enclosing first and least first.
#[test]
fn the_draw_on_a_24_node_position_stays_within_its_work_bound() {
    let state = opening_after(4);
    let candidates = state.face.search_moves();
    let mut most_first: Vec<usize> = (0..candidates.len()).collect();
    most_first.sort_by_key(|&i| std::cmp::Reverse(candidates[i].enclosed.len()));
    let least_first: Vec<usize> = most_first.iter().rev().copied().collect();
    for order in [&most_first, &least_first] {
        stats::take();
        let (drawn, passed) =
            draw_within(&state, &candidates, order, Stopped(ROUTE_BUDGET_MS + 1.0));
        let s = stats::take();
        eprintln!(
            "{} listed, {passed} passed, {} attempts, {} arenas",
            candidates.len(),
            s.attempts,
            s.arenas
        );
        assert!(drawn.is_ok(), "{passed} passed and nothing drew");
        assert!(s.attempts <= PINNED_ATTEMPTS, "{} attempts", s.attempts);
        assert!(s.arenas <= PINNED_ARENAS, "{} arenas", s.arenas);
    }
}

/// One router erodes a face once for every chord between the same ends,
/// whatever the chords enclose.
#[test]
fn a_router_builds_the_arena_once_per_end_pair() {
    let state = opening_after(0);
    let candidates = state.face.search_moves();
    let mut by_ends: std::collections::BTreeMap<(usize, usize), Vec<&FaceMove>> =
        std::collections::BTreeMap::new();
    for m in candidates.iter().filter(|m| !m.enclosed.is_empty()) {
        by_ends.entry((m.a, m.b)).or_default().push(m);
    }
    let sizes: Vec<((usize, usize), usize)> = by_ends.iter().map(|(k, v)| (*k, v.len())).collect();
    let chords: Vec<&FaceMove> = by_ends
        .into_values()
        .find(|chords| chords.len() >= 3)
        .unwrap_or_else(|| panic!("three moves between one pair of ends in {sizes:?}"))
        .into_iter()
        .take(3)
        .collect();
    stats::take();
    let mut router = route::Router::new();
    for mv in &chords {
        let _ = router.route_cheap(&state, mv);
    }
    let shared = stats::take();
    assert_eq!(shared.attempts, 3);
    assert_eq!(shared.arenas, 1);
    for mv in &chords {
        let _ = route::Router::new().route_cheap(&state, mv);
    }
    assert_eq!(stats::take().arenas, 3);
}

/// Prints the reply timings of a 24-node self-play on the 1000 board:
/// `cargo test --release probe_24_node_self_play -- --ignored --nocapture`.
#[test]
#[ignore = "prints timings, run in release"]
fn probe_24_node_self_play() {
    let rows = self_play(GameState::with_board_size(24, 1000), 12);
    print_rows("24 nodes, board 1000, AI against AI", &rows);
}

/// Prints the most states a found corridor held over the bench's seeded
/// games, as the AI and as the human, which sizes the search's state cap.
#[test]
#[ignore = "prints the corridor search's state use, run in release"]
fn probe_sweep_search_states() {
    use crate::route::sweep;
    for (label, player) in [("AI", Player::AI), ("human", Player::Human)] {
        stats::take();
        let mut tally = sweep::Tally::default();
        for (nodes, picks) in sweep::games(7, 24) {
            tally.add(&sweep::game(nodes, &picks, player).expect("no forfeit"));
        }
        let s = stats::take();
        eprintln!(
            "{label}: {tally}; most states in a corridor found {}, bugs {:?}; parity searches {}",
            s.states_ok_max,
            s.refused.bugs,
            parity_summary(&s.parity)
        );
    }
}
