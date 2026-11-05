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
        "{:>4} {:>9} {:>6} {:>5} {:>9} {:>9} {:>8} {:>7} {:>9} {:>8} {:>5} {:>6} {:>6} {:>5} {:>5}",
        "move",
        "total ms",
        "search",
        "draw",
        "listed",
        "passed",
        "attempts",
        "arenas",
        "arena ms",
        "call ms",
        "room",
        "nospot",
        "shape",
        "topo",
        "bug"
    );
    for r in rows {
        let s = &r.stats;
        eprintln!(
            "{:>4} {:>9.0} {:>6.0} {:>5.0} {:>9} {:>9} {:>8} {:>7} {:>9.0} {:>8.0} {:>5} {:>6} {:>6} {:>5} {:>5}{}",
            r.index,
            r.total_ms(),
            r.search_ms,
            r.draw_ms,
            r.candidates,
            r.passed,
            s.attempts,
            s.arenas,
            s.arena_ms,
            s.call_ms_max,
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
            "     most states in a corridor found {}; parity searches {}; fence misses {}; ms in parity searches {:.0}, tighten {:.0}, relax {:.0}, finish {:.0}",
            s.states_ok_max,
            parity_summary(&s.parity),
            s.fence_misses,
            s.phase_ms[0],
            s.phase_ms[1],
            s.phase_ms[2],
            s.phase_ms[3]
        );
    }
    let worst = rows.iter().map(Row::total_ms).fold(0.0, f64::max);
    let longest_call = rows.iter().map(|r| r.stats.call_ms_max).fold(0.0, f64::max);
    eprintln!(
        "worst reply {worst:.0} ms over {} replies; longest route call {longest_call:.0} ms against DRAW_LIMIT_MS {DRAW_LIMIT_MS:.0}",
        rows.len()
    );
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

/// The face move a drawn move reads back as, with its enclosed pieces
/// sorted as the enumeration lists them.
fn reads_as(state: &GameState, mov: &Move) -> FaceMove {
    let read = crate::embedding::embed(state, mov).expect("the drawn move reads back");
    let mut enclosed = read.enclosed;
    enclosed.sort_unstable();
    FaceMove {
        a: read.a,
        at_a: read.at_a,
        b: read.b,
        at_b: read.at_b,
        enclosed,
    }
}

/// Between the route budget and the draw limit the opening still draws
/// under the cheap pass: one attempt per candidate on the first sweep,
/// which runs no parity search, and one more per candidate it held back.
#[test]
fn past_the_route_budget_the_cheap_pass_sweeps_once_then_retries_the_deferred() {
    let state = GameState::with_board_size(4, 1000);
    let candidates = state.face.search_moves();
    let order: Vec<usize> = (0..candidates.len()).collect();
    stats::take();
    let (drawn, passed) = draw_within(&state, &candidates, &order, Stopped(ROUTE_BUDGET_MS + 1.0));
    let s = stats::take();
    let mov = drawn.expect("the opening draws");
    assert!(candidates.contains(&reads_as(&state, &mov)));
    assert!(s.attempts > passed);
    assert!(s.attempts <= candidates.len() + s.refused.sides);
    assert!(s.parity.len() <= s.refused.sides);
}

/// Two nodes joined by a straight line, with a bare third node below it:
/// the two chords listed between the line's ends run round it on the
/// side the first path happens to take and on the other, so one has a
/// wrong-sided first path. Skipping the parity search refuses that one
/// as Sides and draws the other, searching draws it, and no corner pair
/// is remembered as having no room.
#[test]
fn skipping_the_parity_search_refuses_a_wrong_sided_first_path_as_sides() {
    use crate::types::Point;
    let (a, b) = (Point::new(300.0, 500.0), Point::new(700.0, 500.0));
    let mut state = GameState::with_nodes(vec![a, b, Point::new(500.0, 650.0)], 1000);
    let along: Vec<Point> = (0..=8)
        .map(|k| Point::new(a.x + (b.x - a.x) * k as f64 / 8.0, a.y))
        .collect();
    state
        .apply_move(Move::new(
            0,
            1,
            along,
            Point::new(500.0, 500.0),
            Player::Human,
            1000,
        ))
        .unwrap();
    state.current_player = Player::AI;
    let chords: Vec<FaceMove> = state
        .face
        .search_moves()
        .into_iter()
        .filter(|m| m.a == 0 && m.b == 1)
        .collect();
    assert!(chords.len() >= 2, "{chords:?}");
    let mut router = route::Router::new();
    let skipped: Vec<Result<Move, route::Unroutable>> = chords
        .iter()
        .map(|m| router.route_cheap_with(&state, m, route::Parity::Skip))
        .collect();
    let wrong: Vec<usize> = (0..chords.len())
        .filter(|&i| matches!(skipped[i], Err(route::Unroutable::Sides)))
        .collect();
    assert_eq!(wrong.len(), 1, "{chords:?} gave {skipped:?}");
    for (i, drawn) in skipped.iter().enumerate() {
        if i != wrong[0] {
            let drawn = drawn.as_ref().expect("the other chords draw");
            assert_eq!(reads_as(&state, drawn), chords[i]);
        }
    }
    let searched = router
        .route_cheap(&state, &chords[wrong[0]])
        .expect("the parity search draws it");
    assert_eq!(reads_as(&state, &searched), chords[wrong[0]]);
    assert_eq!(router.corners_without_room(), 0);
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
/// another arena or search. The live record has such chords; the opening
/// positions are tried first.
#[test]
fn a_second_chord_between_corners_without_room_is_refused_from_memory() {
    let positions = [6, 8, 10, 12]
        .into_iter()
        .map(opening_after)
        .chain(std::iter::once(position_of_record("room-24n-1000.json")));
    for state in positions {
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
/// enclosing moves first as the ranking tends to put them: 43 attempts
/// and 7 arena builds measured on the settled board, 1 candidate passed
/// over out of 56; the bounds are twice that.
const PINNED_ATTEMPTS: usize = 86;
const PINNED_ARENAS: usize = 14;

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

/// The position of a saved page record with the AI to move: the record's
/// stroke moves are applied as recorded and its topology moves are drawn
/// by the router, as the page does on load.
fn position_of_record(name: &str) -> GameState {
    use crate::types::Point;
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures/undrawn")
        .join(name);
    let raw: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let point = |p: &serde_json::Value| Point::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap());
    let nodes: Vec<Point> = raw["nodes"].as_array().unwrap().iter().map(point).collect();
    let first = match raw["firstPlayer"].as_str().unwrap() {
        "ai" => Player::AI,
        _ => Player::Human,
    };
    let board = raw["boardSize"].as_u64().unwrap() as usize;
    let settle = raw["settleVersion"].as_u64().unwrap_or(0) as u8;
    let mut state = crate::protocol::start(nodes.len(), board, first, Some(nodes), settle).unwrap();
    for m in raw["moves"].as_array().unwrap() {
        let (from, to) = (
            m["from"].as_u64().unwrap() as usize,
            m["to"].as_u64().unwrap() as usize,
        );
        let mov = if let Some(polyline) = m["polyline"].as_array() {
            Move::new(
                from,
                to,
                polyline.iter().map(point).collect(),
                point(&m["newNode"]),
                state.current_player,
                board,
            )
        } else {
            let face = &m["face"];
            let listed = FaceMove {
                a: face["a"].as_u64().unwrap() as usize,
                at_a: face["atA"].as_u64().map(|h| h as usize),
                b: face["b"].as_u64().unwrap() as usize,
                at_b: face["atB"].as_u64().map(|h| h as usize),
                enclosed: face["enclosed"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_u64().unwrap() as usize)
                    .collect(),
            };
            let routed = route::route(&state, &listed).expect("the recorded topology draws");
            Move::from_segments(
                from,
                to,
                routed.polyline,
                routed.segments,
                point(&m["node"]),
                state.current_player,
            )
        };
        crate::protocol::check(&state, &mov).unwrap();
        state.apply_move(mov).unwrap();
    }
    state
}

/// The live position where the page answered Undrawn with room to spare
/// has moves that draw without a parity search, so the cheap pass draws
/// one on its first sweep, before any parity search runs, in listed order
/// and reversed, and the move reads back as a listed one. The ranked walk
/// on this position took 1_300 attempts; the sweep takes 41 and 40.
#[test]
fn the_live_room_position_draws_under_the_cheap_pass() {
    let state = position_of_record("room-24n-1000.json");
    assert_eq!(state.current_player, Player::AI);
    let candidates = state.face.search_moves();
    let forward: Vec<usize> = (0..candidates.len()).collect();
    let backward: Vec<usize> = forward.iter().rev().copied().collect();
    for order in [&forward, &backward] {
        stats::take();
        let (drawn, passed) =
            draw_within(&state, &candidates, order, Stopped(ROUTE_BUDGET_MS + 1.0));
        let s = stats::take();
        eprintln!(
            "{} listed, {passed} passed, {} attempts, {} arenas, {} memo hits",
            candidates.len(),
            s.attempts,
            s.arenas,
            s.memo_hits
        );
        let mov = drawn.unwrap_or_else(|why| panic!("{passed} passed and nothing drew: {why:?}"));
        assert!(candidates.contains(&reads_as(&state, &mov)));
        assert!(
            s.parity.is_empty(),
            "{} parity searches ran",
            s.parity.len()
        );
        assert!(s.attempts <= candidates.len());
    }
}

/// Replies on the live position where the page answered Undrawn with room
/// to spare: `cargo test --release probe_live_room_position -- --ignored
/// --nocapture`.
#[test]
#[ignore = "prints replies, run in release"]
fn probe_live_room_position() {
    let state = position_of_record("room-24n-1000.json");
    assert_eq!(state.current_player, Player::AI);
    let candidates = state.face.search_moves();
    eprintln!("{} candidates", candidates.len());
    for run in 0..5 {
        stats::take();
        let t = Instant::now();
        let order = rank(&state, &candidates);
        let search_ms = t.elapsed().as_secs_f64() * 1000.0;
        let t = Instant::now();
        let (drawn, passed) = draw(&state, &candidates, &order);
        let draw_ms = t.elapsed().as_secs_f64() * 1000.0;
        let s = stats::take();
        eprintln!(
            "run {run}: {} after {passed} passed, search {search_ms:.0} ms, draw {draw_ms:.0} ms, {} attempts, {} arenas, {} memo hits, longest call {:.0} ms, room {}, bugs {:?}, first ranked {:?}",
            match &drawn {
                Ok(m) => format!("drew {}-{}", m.from_node, m.to_node),
                Err(why) => format!("{why:?}"),
            },
            s.attempts,
            s.arenas,
            s.memo_hits,
            s.call_ms_max,
            s.refused.room,
            s.refused.bugs,
            candidates[order[0]]
        );
    }
}

/// Every candidate of the live position routed once at the cheap clearance
/// through a fresh Router and once through one shared Router: what each
/// came to, whether a parity search ran, and what it cost. `cargo test
/// --release probe_live_room_candidates -- --ignored --nocapture`.
#[test]
#[ignore = "routes every candidate, run in release"]
fn probe_live_room_candidates() {
    let state = position_of_record("room-24n-1000.json");
    let candidates = state.face.search_moves();
    let mut shared = route::Router::new();
    let mut tally: std::collections::BTreeMap<String, (usize, f64)> =
        std::collections::BTreeMap::new();
    let mut drawable = Vec::new();
    let mut disagreements = 0;
    for (i, mv) in candidates.iter().enumerate() {
        stats::take();
        let t = Instant::now();
        let fresh = route::Router::new().route_cheap(&state, mv);
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        let s = stats::take();
        let parity = if s.parity.is_empty() {
            "no parity"
        } else if s.fence_misses > 0 {
            "parity, fences missed"
        } else {
            "parity"
        };
        let kind = match &fresh {
            Ok(_) => format!("ok, {parity}"),
            Err(route::Unroutable::Bug(what)) => format!("bug {what}"),
            Err(route::Unroutable::Shape(what)) => format!("shape {what}, {parity}"),
            Err(why) => format!("{why:?}, {parity}"),
        };
        let slot = tally.entry(kind).or_insert((0, 0.0));
        slot.0 += 1;
        slot.1 += ms;
        if fresh.is_ok() {
            drawable.push(i);
        }
        let together = shared.route_cheap(&state, mv);
        let same = match (&fresh, &together) {
            (Ok(a), Ok(b)) => a.segments == b.segments,
            (Err(a), Err(b)) => a == b,
            _ => false,
        };
        if !same {
            disagreements += 1;
            eprintln!("candidate {i} {mv:?}: fresh {fresh:?} shared {together:?}");
        }
    }
    eprintln!(
        "{} candidates, {disagreements} shared/fresh disagreements",
        candidates.len()
    );
    for (kind, (n, ms)) in &tally {
        eprintln!(
            "  {kind}: {n}, {:.0} ms total, {:.1} ms each",
            ms,
            ms / *n as f64
        );
    }
    eprintln!(
        "drawable at the cheap clearance: {} of {}",
        drawable.len(),
        candidates.len()
    );
    for i in drawable.iter().take(12) {
        eprintln!("  {i}: {:?}", candidates[*i]);
    }
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
            "{label}: {tally}; most states in a corridor found {}, bugs {:?}; parity searches {}; fence misses {}",
            s.states_ok_max,
            s.refused.bugs,
            parity_summary(&s.parity),
            s.fence_misses
        );
    }
}

/// The board of a page record under fixtures/undrawn replayed as the page
/// does: every move applied as it committed, the board settled after each.
fn position_of_fixture(name: &str) -> GameState {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("fixtures/undrawn")
        .join(name);
    let fixture = crate::replay::load(&path).unwrap();
    crate::replay::play_as(&fixture, Player::Human)
}

/// Where the corner at node 1 of the live 4-node position is widest at
/// radius `r`: the point and its distance to the nearer of the node's two
/// spokes, over directions between 20 and 85 degrees above due left.
fn widest_at(state: &GameState, r: f64) -> (crate::types::Point, f64) {
    use crate::geometry::closest_point_on_polyline;
    let tol = crate::curve::tolerance(state.board_size);
    let at = state.nodes[1].position;
    let spokes: Vec<Vec<crate::types::Point>> = state
        .lines
        .iter()
        .filter(|l| l.from_node == 1 || l.to_node == 1)
        .map(|l| l.flatten(tol))
        .collect();
    (20..=85)
        .map(|deg| {
            let rad = f64::from(deg).to_radians();
            let p = crate::types::Point::new(at.x - rad.cos() * r, at.y - rad.sin() * r);
            let gap = spokes
                .iter()
                .map(|s| closest_point_on_polyline(s, &p).1)
                .fold(f64::INFINITY, f64::min);
            (p, gap)
        })
        .fold(
            (at, 0.0),
            |best, next| if next.1 > best.1 { next } else { best },
        )
}

/// The chord from node 1 to node 12 on the live 4-node position, drawn by
/// hand: out of the corner at node 1 along its widest direction to 30
/// units, up the middle of the corridor between node 1's two spokes, then
/// across the open face to node 12.
fn hand_chord(state: &GameState) -> Move {
    use crate::types::Point;
    let tol = crate::curve::tolerance(state.board_size);
    let mut stroke = vec![state.nodes[1].position];
    for r in [3.0, 5.0, 7.0, 9.0, 12.0, 15.0, 18.0, 22.0, 26.0, 30.0] {
        stroke.push(widest_at(state, r).0);
    }
    let x_at = |poly: &[Point], y: f64| -> f64 {
        poly.windows(2)
            .find_map(|w| {
                let (p, q) = (w[0], w[1]);
                ((p.y - y) * (q.y - y) <= 0.0 && p.y != q.y)
                    .then(|| p.x + (q.x - p.x) * (y - p.y) / (q.y - p.y))
            })
            .expect("the wall spans this height")
    };
    let to_node_4 = state.lines[1].flatten(tol);
    let arc_to_node_2 = state.lines[7].flatten(tol);
    let mut y = 705.0;
    while y >= 440.0 {
        let x = (x_at(&to_node_4, y) + x_at(&arc_to_node_2, y)) / 2.0;
        stroke.push(Point::new(x, y));
        y -= 15.0;
    }
    for (x, y) in [
        (178.0, 420.0),
        (200.0, 370.0),
        (240.0, 330.0),
        (290.0, 300.0),
        (335.0, 275.0),
    ] {
        stroke.push(Point::new(x, y));
    }
    stroke.push(state.nodes[12].position);
    let node = stroke[stroke.len() / 2];
    Move::new(1, 12, stroke, node, Player::AI, state.board_size)
}

/// The listed move of the live 4-node position: a chord from node 1 to
/// node 12 through the big face, the only move the model lists.
fn pocket_chord() -> FaceMove {
    FaceMove {
        a: 1,
        at_a: Some(4),
        b: 12,
        at_b: Some(34),
        enclosed: Vec::new(),
    }
}

/// The live 4-node position where the page answered Undrawn lists one
/// move, and that move is legal: a chord drawn by hand passes the AI's
/// validation, reads back as the listed move and applies.
#[test]
fn the_live_pocket_position_lists_a_legal_move() {
    let state = position_of_fixture("pocket-4n-1000.json");
    assert_eq!(state.current_player, Player::AI);
    assert_eq!(state.face.search_moves(), vec![pocket_chord()]);
    let mov = hand_chord(&state);
    crate::protocol::check(&state, &mov).unwrap();
    assert_eq!(embedding::embed(&state, &mov).unwrap(), pocket_chord());
    let mut after = state.clone();
    let wire = crate::protocol::Move {
        from_node: mov.from_node,
        to_node: mov.to_node,
        polyline: mov.polyline.clone(),
        new_node_pos: mov.new_node_pos,
        segments: Some(mov.segments.clone()),
    };
    assert!(matches!(
        crate::protocol::apply(&mut after, wire),
        crate::protocol::Outcome::Applied { .. }
    ));
    assert_eq!(after.lines.len(), state.lines.len() + 1);
}

/// The AI draws the live 4-node position's one listed move.
#[test]
fn the_live_pocket_position_draws() {
    let state = position_of_fixture("pocket-4n-1000.json");
    let candidates = state.face.search_moves();
    let order = rank(&state, &candidates);
    let (drawn, passed) = draw(&state, &candidates, &order);
    let mov = drawn.unwrap_or_else(|why| panic!("{passed} passed and nothing drew: {why:?}"));
    assert_eq!(reads_as(&state, &mov), pocket_chord());
}

/// Every attempt on the live 4-node position: the listed moves and their
/// class members at the AI clearance, at the human clearance, and without
/// the parity search, then the draw as the page runs it, and how wide the
/// corner at node 1 is at each radius. `cargo test --release
/// probe_live_pocket_position -- --ignored --nocapture`.
#[test]
#[ignore = "prints every attempt, run in release"]
fn probe_live_pocket_position() {
    let state = position_of_fixture("pocket-4n-1000.json");
    assert_eq!(state.current_player, Player::AI);
    for (i, n) in state.nodes.iter().enumerate() {
        eprintln!(
            "node {i}: ({:.1}, {:.1}) degree {}",
            n.position.x,
            n.position.y,
            state.face.degree(i)
        );
    }
    let candidates = state.face.search_moves();
    let order = rank(&state, &candidates);
    eprintln!("{} listed", candidates.len());
    for (r, &idx) in order.iter().enumerate() {
        let listed = &candidates[idx];
        let face = state.face.corner_face(listed.a, listed.at_a).unwrap();
        eprintln!(
            "rank {r}: {listed:?} in face {face:?} with pieces {:?}",
            state.face.components(face)
        );
        let members = class_members(&state, listed);
        for (what, attempt) in
            std::iter::once(("listed", listed)).chain(members.iter().map(|m| ("member", m)))
        {
            let outcome = |drawn: Result<Move, route::Unroutable>| match drawn {
                Ok(m) => format!("drew {}-{}", m.from_node, m.to_node),
                Err(why) => format!("{why:?}"),
            };
            stats::take();
            let t = Instant::now();
            let full = route::Router::new().route(&state, attempt);
            let ms = t.elapsed().as_secs_f64() * 1000.0;
            let s = stats::take();
            eprintln!(
                "  {what} {attempt:?}: {} in {ms:.1} ms over {} attempts, parity searches {}",
                outcome(full),
                s.attempts,
                s.parity.len()
            );
            let cheap = route::Router::new().route_cheap(&state, attempt);
            let skip = route::Router::new().route_cheap_with(&state, attempt, route::Parity::Skip);
            eprintln!(
                "    human clearance: {}; without the parity search: {}",
                outcome(cheap),
                outcome(skip)
            );
        }
    }
    stats::take();
    let t = Instant::now();
    let (drawn, passed) = draw(&state, &candidates, &order);
    let ms = t.elapsed().as_secs_f64() * 1000.0;
    let s = stats::take();
    eprintln!(
        "draw: {:?} after {passed} passed in {ms:.1} ms, {} attempts, {} arenas, {}",
        drawn.map(|m| (m.from_node, m.to_node)),
        s.attempts,
        s.arenas,
        s.refused
    );
    for r in [3.0, 4.0, 5.0, 5.5, 6.0, 7.0, 8.0, 10.0] {
        let (p, gap) = widest_at(&state, r);
        eprintln!(
            "corner at node 1, {r} units out: {gap:.2} units to the nearer spoke at ({:.1}, {:.1})",
            p.x, p.y
        );
    }
    let mut as_human = state.clone();
    as_human.current_player = Player::Human;
    eprintln!(
        "the same chord routed for the human: {:?}",
        route::route(&as_human, &pocket_chord()).map(|m| (m.from_node, m.to_node))
    );
    let mov = hand_chord(&state);
    eprintln!(
        "hand chord: {} points, check {:?}, reads as {:?}",
        mov.polyline.len(),
        crate::protocol::check(&state, &mov).map_err(|r| r.message),
        embedding::embed(&state, &mov).map_err(|r| r.message)
    );
}
