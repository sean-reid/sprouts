use super::golden::{
    bits, hash, play_random, seeded_hashes, seeded_picks, stage_report, VERSION_1_GOLDEN,
    VERSION_1_SEEDED_GAME, VERSION_2_GOLDEN, VERSION_2_SEEDED_GAME,
};
use super::*;
use crate::protocol::{self, SETTLE_VERSION};
use crate::replay;
use crate::types::grid;
use crate::types::Player;
use proptest::prelude::*;
use std::path::Path;

/// A version 0 record's strokes are applied as drawn and the settle is
/// measured on a copy of each position at the current version.
fn settled_copy(state: &GameState) -> GameState {
    let mut copy = state.clone();
    copy.settle_version = SETTLE_VERSION;
    settle(&mut copy);
    copy
}

fn fixtures() -> Vec<replay::Fixture> {
    fixtures_under("fixtures/games")
}

/// The games under fixtures/games plus the large ones under fixtures/settle.
fn all_fixtures() -> Vec<replay::Fixture> {
    let mut out = fixtures();
    out.extend(fixtures_under("fixtures/settle"));
    out
}

fn fixtures_under(dir: &str) -> Vec<replay::Fixture> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join(dir);
    let mut paths: Vec<_> = std::fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    paths.iter().map(|p| replay::load(p).unwrap()).collect()
}

/// The farthest any node or control point sits from where it was.
fn drift(before: &GameState, after: &GameState) -> f64 {
    let mut worst: f64 = 0.0;
    for (a, b) in before.nodes.iter().zip(&after.nodes) {
        worst = worst.max(a.position.distance_to(&b.position));
    }
    for (a, b) in before.lines.iter().zip(&after.lines) {
        assert_eq!(a.segments.len(), b.segments.len());
        for (x, y) in a.segments.iter().zip(&b.segments) {
            for (p, q) in [(x.p0, y.p0), (x.p1, y.p1), (x.p2, y.p2), (x.p3, y.p3)] {
                worst = worst.max(p.distance_to(&q));
            }
        }
    }
    worst
}

/// Replays every line of `state` as a fresh move through the embedding
/// alone, without the settle's own check, and compares the canonical form
/// of the model that gives with the model the state holds; a bare loop may
/// read as either side, which the canonical form treats as one position.
fn replays_to_the_same_model(state: &GameState) -> bool {
    let mut fresh = GameState::with_nodes_at(
        state.nodes[..state.initial_node_count]
            .iter()
            .map(|n| n.position)
            .collect(),
        state.board_size,
        state.settle_version,
    );
    for line in &state.lines {
        fresh.current_player = line.player;
        let mov = crate::types::Move::as_given(
            line.from_node,
            line.to_node,
            line.polyline.clone(),
            line.segments.clone(),
            line.new_node_pos,
            line.player,
        );
        if fresh.apply_move(mov).is_err() {
            return false;
        }
    }
    crate::face::canonical_form(&fresh.face) == crate::face::canonical_form(&state.face)
}

fn same_face_model(state: &GameState) -> bool {
    let targets = targets_of(state);
    let layout = Layout::from_state(state, &targets).unwrap();
    layout.holds(state, &journal(state))
}

#[test]
fn every_fixture_position_settles_without_changing_the_model_or_losing_clearance() {
    for fixture in fixtures() {
        let mut drawn = fixture.start();
        assert_eq!(drawn.settle_version, 0);
        let mut worst_after = f64::INFINITY;
        for (index, recorded) in fixture.moves.iter().enumerate() {
            let mov = fixture_move(&drawn, recorded);
            protocol::check(&drawn, &mov)
                .unwrap_or_else(|e| panic!("move {index} rejected: {}", e.message));
            drawn.apply_move(mov).unwrap();
            let (before, _) = least_clearance(&drawn);
            let state = settled_copy(&drawn);
            let (after, at) = least_clearance(&state);
            assert!(
                after + 1e-9 >= before,
                "move {index}: least clearance fell from {before:.3} to {after:.3}"
            );
            assert!(same_face_model(&state), "move {index} changed the model");
            assert!(
                replays_to_the_same_model(&state),
                "move {index}: the settled lines do not replay to the model"
            );
            worst_after = worst_after.min(after);
            println!(
                "{} nodes, board {}, move {index}: least ratio {before:.2} -> {after:.2} at {at:?}",
                fixture.initial_nodes, fixture.board_size
            );
            for (line, was) in state.lines.iter().zip(&drawn.lines) {
                assert_eq!(line.flat, line.flatten(curve::tolerance(state.board_size)));
                let node = state.find_node(state.initial_node_count + line.id).unwrap();
                assert_eq!(node.position, line.new_node_pos);
                if line.segments == was.segments {
                    assert_eq!(line.polyline, was.polyline);
                    continue;
                }
                let name = format!(
                    "{} nodes, move {index}, line {}",
                    fixture.initial_nodes, line.id
                );
                assert_eq!(line.polyline, line.flat, "{name}");
                assert_eq!(
                    line.segments[0].p0, state.nodes[line.from_node].position,
                    "{name}"
                );
                assert_eq!(
                    line.segments.last().unwrap().p3,
                    state.nodes[line.to_node].position,
                    "{name}"
                );
            }
        }
        println!(
            "{} nodes, board {}: least ratio after settling over the game {worst_after:.2}",
            fixture.initial_nodes, fixture.board_size
        );
    }
}

/// A board settles once per move and undo restores it, so a second settle
/// never runs in play; when it does it may still move the board, since
/// the grid keeps the relaxation from a fixed point, but it never lowers
/// the least clearance ratio or changes the model.
#[test]
fn settling_again_never_lowers_the_least_clearance() {
    let mut drift: f64 = 0.0;
    for fixture in all_fixtures() {
        let mut drawn = fixture.start();
        for recorded in &fixture.moves {
            let mov = fixture_move(&drawn, recorded);
            drawn.apply_move(mov).unwrap();
            if drawn.settle_version > 0 {
                settle(&mut drawn);
            }
            let once = settled_copy(&drawn);
            let mut twice = once.clone();
            settle(&mut twice);
            let (before, _) = least_clearance(&once);
            let (after, _) = least_clearance(&twice);
            assert!(
                after + 1e-9 >= before,
                "a second settle lowered the least ratio from {before:.3} to {after:.3} after {} moves of a {} node game",
                once.lines.len(),
                fixture.initial_nodes
            );
            assert!(same_face_model(&twice));
            drift = drift.max(self::drift(&once, &twice));
        }
    }
    println!("a second settle moved at most {drift:.1} units");
}

/// A recording made with no settling replays with none: the board after
/// every move is the one the strokes alone give, and every settle record
/// is empty.
#[test]
fn a_version_0_record_replays_as_drawn() {
    for fixture in fixtures() {
        let mut through = fixture.start();
        let mut plain = fixture.start();
        for recorded in &fixture.moves {
            let wire = protocol::Move {
                from_node: recorded.from,
                to_node: recorded.to,
                polyline: recorded
                    .polyline
                    .iter()
                    .map(|p| Point::new(p[0], p[1]))
                    .collect(),
                new_node_pos: Point::new(recorded.new_node[0], recorded.new_node[1]),
                segments: None,
            };
            assert!(matches!(
                protocol::apply(&mut through, wire),
                protocol::Outcome::Applied { .. }
            ));
            plain.apply_move(fixture_move(&plain, recorded)).unwrap();
            assert_eq!(bits(&through), bits(&plain));
        }
        assert_eq!(through.settles.len(), fixture.moves.len());
        assert!(through
            .settles
            .iter()
            .all(|s| s.nodes.is_empty() && s.lines.is_empty()));
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

    #[test]
    fn random_games_settle_to_the_same_bytes_in_two_passes(
        nodes in 3usize..=9,
        picks in proptest::collection::vec((0usize..64, 0usize..64, 0.0..1.0f64, 0.0..1.0f64), 40),
    ) {
        let (first, played) = play_random(nodes, &picks, SETTLE_VERSION);
        let (second, again) = play_random(nodes, &picks, SETTLE_VERSION);
        prop_assert!(!played.is_empty());
        prop_assert_eq!(played.len(), again.len());
        prop_assert_eq!(bits(&first), bits(&second));
        prop_assert!(same_face_model(&first));
        prop_assert!(replays_to_the_same_model(&first));
    }
}

/// A fresh board has no move to settle around; the first chord, clear as
/// it is, still opens the board up around it.
#[test]
fn a_fresh_board_is_left_alone_and_a_clear_chord_still_opens_the_board() {
    let mut state = GameState::new(4);
    let before = bits(&state);
    settle(&mut state);
    assert_eq!(bits(&state), before);
    assert!(state.settles.pop().is_some());
    let a = state.nodes[0].position;
    let b = state.nodes[1].position;
    let polyline: Vec<Point> = (0..=10)
        .map(|i| {
            let t = i as f64 / 10.0;
            Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
        })
        .collect();
    let mid = polyline[5];
    state
        .apply_move(crate::types::Move::new(
            0,
            1,
            polyline.clone(),
            mid,
            Player::Human,
            1000,
        ))
        .unwrap();
    let (before, _) = least_clearance(&state);
    settle(&mut state);
    let (after, _) = least_clearance(&state);
    assert!(after > before + 0.5, "{before:.3} -> {after:.3}");
    assert!(same_face_model(&state));
    assert_eq!(state.lines[0].polyline, state.lines[0].flat);
}

/// A chord drawn a human clearance past a node: the node and the line
/// part toward the disc clearance, the stroke becomes the flattening and
/// the node the line made stays on it.
#[test]
fn a_line_hugging_a_node_is_pushed_off_it() {
    let mut state = GameState::new(4);
    state.nodes[0].position = Point::new(300.0, 500.0);
    state.nodes[1].position = Point::new(700.0, 500.0);
    state.nodes[2].position = Point::new(500.0, 512.0);
    state.nodes[3].position = Point::new(500.0, 200.0);
    let polyline: Vec<Point> = (0..=40)
        .map(|i| Point::new(300.0 + 10.0 * i as f64, 500.0))
        .collect();
    let mov = crate::types::Move::new(
        0,
        1,
        polyline.clone(),
        Point::new(400.0, 500.0),
        Player::Human,
        1000,
    );
    crate::protocol::check(&state, &mov).unwrap();
    state.apply_move(mov).unwrap();
    let (before, _) = least_clearance(&state);
    assert!(before < 1.0);
    settle(&mut state);
    let (after, _) = least_clearance(&state);
    assert!(after > before, "{before:.3} -> {after:.3}");
    let line = &state.lines[0];
    let gap = curve::nearest(&line.segments, &state.nodes[2].position)
        .unwrap()
        .distance;
    assert!(gap > 12.0, "node 2 is still {gap:.1} from the line");
    assert_ne!(line.polyline, polyline);
    assert_eq!(line.polyline, line.flat);
    let on_curve = curve::nearest(&line.segments, &state.nodes[4].position)
        .unwrap()
        .distance;
    assert!(on_curve <= grid(1000) * 2f64.sqrt() / 2.0, "{on_curve}");
    assert!(same_face_model(&state));
}

fn fixture_move(state: &GameState, recorded: &replay::FixtureMove) -> crate::types::Move {
    recorded.enter(state).unwrap()
}

/// No node or control point moves farther in one settle than the rounds
/// at their longest stride allow, and nothing leaves the board; the
/// largest move and the drift over a whole game are printed for the
/// record.
#[test]
fn nothing_moves_more_than_the_rounds_allow_or_leaves_the_board() {
    for fixture in all_fixtures() {
        let mut drawn = fixture.start();
        let board = drawn.board_size as f64;
        let bound = ROUNDS as f64 * STRIDE_MAX * MIN_NODE_SPACING * scale(drawn.board_size);
        let mut drift: f64 = 0.0;
        let mut largest: f64 = 0.0;
        let mut first_seen: Vec<Point> = Vec::new();
        for recorded in &fixture.moves {
            let mov = fixture_move(&drawn, recorded);
            drawn.apply_move(mov).unwrap();
            if drawn.settle_version > 0 {
                settle(&mut drawn);
            }
            let before: Vec<Point> = drawn.nodes.iter().map(|n| n.position).collect();
            let controls: Vec<Vec<Cubic>> =
                drawn.lines.iter().map(|l| l.segments.clone()).collect();
            if first_seen.len() < before.len() {
                first_seen.extend_from_slice(&before[first_seen.len()..]);
            }
            let state = settled_copy(&drawn);
            let inside = |p: &Point| p.x >= 0.0 && p.y >= 0.0 && p.x <= board && p.y <= board;
            for (line, was) in state.lines.iter().zip(&controls) {
                for (c, w) in line.segments.iter().zip(was) {
                    for (p, q) in [(c.p0, w.p0), (c.p1, w.p1), (c.p2, w.p2), (c.p3, w.p3)] {
                        let moved = p.distance_to(&q);
                        largest = largest.max(moved);
                        assert!(
                            moved <= bound + 1e-6,
                            "{} nodes: a control point of line {} moved {moved:.2} in one settle",
                            fixture.initial_nodes,
                            line.id
                        );
                    }
                }
                assert!(
                    line.flat.iter().all(inside),
                    "{} nodes: line {} left the board",
                    fixture.initial_nodes,
                    line.id
                );
            }
            for (node, was) in state.nodes.iter().zip(&before) {
                let moved = node.position.distance_to(was);
                largest = largest.max(moved);
                assert!(
                    moved <= bound + 1e-6,
                    "{} nodes: node {} moved {moved:.2} in one settle",
                    fixture.initial_nodes,
                    node.id
                );
                assert!(
                    inside(&node.position),
                    "{} nodes: node {} left the board",
                    fixture.initial_nodes,
                    node.id
                );
                drift = drift.max(node.position.distance_to(&first_seen[node.id]));
            }
        }
        println!(
            "{} nodes, board {}: largest move in one settle {largest:.1}, a node drifted at most {drift:.1} units over {} moves",
            fixture.initial_nodes,
            fixture.board_size,
            fixture.moves.len()
        );
    }
}

/// Games played with the settle, as the page plays them, at 3 to 9 nodes,
/// with the moves as sent.
fn settled_games() -> Vec<(GameState, Vec<protocol::Move>)> {
    (3..=9)
        .map(|nodes| play_random(nodes, &seeded_picks(nodes as u64, 40), SETTLE_VERSION))
        .filter(|(_, moves)| !moves.is_empty())
        .collect()
}

/// Undo puts back the geometry the undone move was drawn on, so the
/// stroke that followed re-applies as it did, and the moves as drawn
/// replay through the protocol to the same settled board.
#[test]
fn undo_restores_the_board_the_next_move_was_drawn_on() {
    let games = settled_games();
    assert!(games.len() >= 5);
    for (played, moves) in games {
        let mut state = GameState::with_board_size(played.initial_node_count, played.board_size);
        state.current_player = Player::Human;
        let mut boards = vec![bits(&state)];
        for mov in &moves {
            assert!(matches!(
                protocol::apply(&mut state, mov.clone()),
                protocol::Outcome::Applied { .. }
            ));
            boards.push(bits(&state));
        }
        assert_eq!(bits(&state), bits(&played));
        assert_eq!(state.settles.len(), moves.len());
        let finished = boards.last().unwrap().clone();
        while protocol::undo(&mut state) {
            boards.pop();
            assert_eq!(
                bits(&state),
                *boards.last().unwrap(),
                "{} nodes: undo to {} moves left a different board",
                played.initial_node_count,
                state.lines.len()
            );
        }
        assert!(state.settles.is_empty());
        for mov in &moves {
            assert!(matches!(
                protocol::apply(&mut state, mov.clone()),
                protocol::Outcome::Applied { .. }
            ));
        }
        assert_eq!(bits(&state), finished);
    }
}

/// The seeded 7-node game settles to the board recorded for `version`.
fn settles_the_seeded_game_to_the_golden_board(version: u8, golden: u64) {
    let (state, moves) = play_random(7, &seeded_picks(7, 40), version);
    assert!(moves.len() >= 6, "{} moves", moves.len());
    assert_eq!(state.settle_version, version);
    assert_eq!(
        hash(&state),
        golden,
        "version {version}: {:#x}; stages of move 1: {}",
        hash(&state),
        stage_report(7, 1, version)
    );
}

/// Version 1 stays as recorded, since records and links at that version
/// replay through it.
#[test]
fn version_1_settles_the_seeded_game_to_the_golden_board() {
    settles_the_seeded_game_to_the_golden_board(1, VERSION_1_GOLDEN);
}

#[test]
fn version_2_settles_the_seeded_game_to_the_golden_board() {
    assert_eq!(SETTLE_VERSION, 2);
    settles_the_seeded_game_to_the_golden_board(2, VERSION_2_GOLDEN);
}

/// A link rounds every point to a whole unit. The moves a settled game
/// applied already sit on that grid, so the rounding changes nothing and
/// the visitor's board is the sharer's, bit for bit.
#[test]
fn moves_rounded_to_the_link_quantum_replay_a_settled_game() {
    for (played, moves) in settled_games() {
        let mut state = GameState::with_board_size(played.initial_node_count, played.board_size);
        state.current_player = Player::Human;
        let round = |p: &Point| Point::new(p.x.round(), p.y.round());
        for mov in &moves {
            let rounded = protocol::Move {
                from_node: mov.from_node,
                to_node: mov.to_node,
                polyline: mov.polyline.iter().map(round).collect(),
                new_node_pos: round(&mov.new_node_pos),
                segments: mov.segments.as_ref().map(|segments| {
                    segments
                        .iter()
                        .map(|c| Cubic {
                            p0: round(&c.p0),
                            p1: round(&c.p1),
                            p2: round(&c.p2),
                            p3: round(&c.p3),
                        })
                        .collect()
                }),
            };
            assert_eq!(&rounded, mov, "an applied move is not on the grid");
            let outcome = protocol::apply(&mut state, rounded);
            assert!(
                matches!(outcome, protocol::Outcome::Applied { .. }),
                "{} nodes, move {}: {outcome:?}",
                played.initial_node_count,
                state.lines.len()
            );
        }
        assert_eq!(bits(&state), bits(&played));
    }
}

/// The seeded 9-node game settles move for move to the boards recorded
/// for `version`.
fn settles_the_seeded_game_the_same_on_every_platform(version: u8, want: &[u64]) {
    let hashes = seeded_hashes(9, version);
    let printed: Vec<String> = hashes.iter().map(|h| format!("{h:#x}")).collect();
    let first = hashes.iter().zip(want).position(|(got, want)| got != want);
    assert_eq!(
        hashes.as_slice(),
        want,
        "version {version}: first differing move {first:?}; got {}; stages of that move: {}",
        printed.join(" "),
        stage_report(9, first.unwrap_or(0), version)
    );
}

#[test]
fn version_1_settles_the_seeded_game_the_same_on_every_platform() {
    settles_the_seeded_game_the_same_on_every_platform(1, VERSION_1_SEEDED_GAME);
}

#[test]
fn version_2_settles_the_seeded_game_the_same_on_every_platform() {
    settles_the_seeded_game_the_same_on_every_platform(2, VERSION_2_SEEDED_GAME);
}

/// Forty points round `center` at `radius`, starting at `start`.
fn ring(start: Point, center: Point) -> Vec<Point> {
    let radius = start.distance_to(&center);
    let (c, s) = (0.987_688_340_595_137_8, 0.156_434_465_040_230_87);
    let mut d = Point::new(start.x - center.x, start.y - center.y);
    (0..=40)
        .map(|_| {
            let p = Point::new(center.x + d.x, center.y + d.y);
            d = Point::new(d.x * c - d.y * s, d.x * s + d.y * c);
            Point::new(p.x, p.y)
        })
        .map(|p| {
            let dx = p.x - center.x;
            let dy = p.y - center.y;
            let k = radius / (dx * dx + dy * dy).sqrt();
            Point::new(center.x + dx * k, center.y + dy * k)
        })
        .collect()
}

/// The guard every round passes refuses a node dragged out of the loop
/// that held it, a changed face, and a chord dragged through another
/// line, a crossing, while it accepts the layout as drawn.
#[test]
fn the_round_guard_refuses_a_changed_face_and_a_new_crossing() {
    let mut state = GameState::new(4);
    let center = Point::new(500.0, 500.0);
    state.nodes[0].position = Point::new(400.0, 500.0);
    state.nodes[1].position = Point::new(200.0, 200.0);
    state.nodes[2].position = Point::new(540.0, 500.0);
    state.nodes[3].position = Point::new(800.0, 800.0);
    let polyline = ring(state.nodes[0].position, center);
    let mov = crate::types::Move::new(
        0,
        0,
        polyline,
        Point::new(500.0, 400.0),
        Player::Human,
        1000,
    );
    protocol::check(&state, &mov).unwrap();
    state.apply_move(mov).unwrap();
    let targets = targets_of(&state);
    let tolerance = curve::tolerance(state.board_size);
    let original = journal(&state);
    let layout = Layout::from_state(&state, &targets).unwrap();
    let before = layout.crossings(tolerance);
    assert!(layout.holds(&state, &original));
    assert!(layout.keeps(&state, &original, before, before));

    // Node 2 dragged out of the loop sits on another face.
    let mut moved = layout.clone();
    moved.nodes[2] = Point::new(700.0, 500.0);
    assert!(!moved.holds(&state, &original));
    assert!(!moved.keeps(&state, &original, before, moved.crossings(tolerance)));

    // A chord from node 2 straight to node 3 passes through the loop line.
    let chord: Vec<Point> = (0..=40)
        .map(|i| {
            let t = i as f64 / 40.0;
            Point::new(540.0 + 260.0 * t, 500.0 + 300.0 * t)
        })
        .collect();
    let bent = crate::types::Move::new(2, 3, chord, Point::new(670.0, 650.0), Player::AI, 1000);
    let line = Line {
        id: 1,
        from_node: bent.from_node,
        to_node: bent.to_node,
        polyline: bent.polyline.clone(),
        segments: bent.segments.clone(),
        flat: bent.flatten(tolerance),
        new_node_pos: bent.new_node_pos,
        player: bent.player,
    };
    let mut crossing = layout.clone();
    crossing
        .lines
        .push(Chain::from_line(&line, &targets).unwrap());
    crossing.nodes.push(bent.new_node_pos);
    let crossed = crossing.crossings(tolerance);
    assert!(crossed > before, "{crossed} crossings against {before}");
    assert!(!crossing.keeps(&state, &original, before, crossed));
}

/// The board after a version 0 fixture replays, rounded to a millionth
/// so platform rounding of the fit cannot move it, as recorded here.
#[test]
fn a_version_0_fixture_replays_to_the_recorded_board() {
    const HUMAN_4N_1000: u64 = 0xc87947d565ba34c4;
    let fixture = replay::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/games/human-4n-1000.json"),
    )
    .unwrap();
    let mut state = fixture.start();
    for recorded in &fixture.moves {
        assert!(matches!(
            protocol::apply(&mut state, recorded.wire()),
            protocol::Outcome::Applied { .. }
        ));
    }
    let mut fnv = crate::exact::Fnv::default();
    for b in bits(&state) {
        fnv.feed((f64::from_bits(b) * 1e6).round() / 1e6);
    }
    assert_eq!(fnv.0, HUMAN_4N_1000, "{}", fnv.hex());
}

/// The settle under `version` of the position after `count` moves of a
/// recorded game, with its report.
fn settled_at(fixture: &replay::Fixture, count: usize, version: u8) -> (GameState, Report) {
    let mut state = fixture.start();
    for recorded in fixture.moves.iter().take(count) {
        let mov = fixture_move(&state, recorded);
        state.apply_move(mov).unwrap();
    }
    state.settle_version = version;
    let report = settle_reported(&mut state);
    (state, report)
}

/// Move 6 of the 5-node self-play game leaves a pair no round can free.
/// Version 1 stops there; version 2 freezes that pair, holds what takes
/// part in it, and goes on with the rest, with the least no lower than
/// drawn and no bend left below the target that version 1 had above it.
#[test]
fn a_stuck_tightest_pair_still_lets_the_rest_of_the_board_spread() {
    let fixture = replay::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/games/selfplay-5n-1000.json"),
    )
    .unwrap();
    let (drawn, _) = least_clearance(&settled_at(&fixture, 6, 0).0);
    let (one, first) = settled_at(&fixture, 6, 1);
    let (two, second) = settled_at(&fixture, 6, 2);
    assert_eq!(first.stop, Stop::Stuck);
    assert!(!second.frozen.is_empty(), "{second:?}");
    assert!(
        least_clearance(&two).0 + 1e-9 >= drawn,
        "{drawn} -> {second:?}"
    );
    let sharpest = |state: &GameState| {
        ratios(state)
            .into_iter()
            .filter(|(at, _)| matches!(at, Tightest::Bend(_)))
            .map(|(_, r)| r)
            .fold(f64::INFINITY, f64::min)
    };
    assert!(
        sharpest(&two) + 1e-9 >= sharpest(&one).min(1.0),
        "sharpest bend {:.3} -> {:.3}",
        sharpest(&one),
        sharpest(&two)
    );
    // The free pairs spread: lines and hairpins end farther apart.
    let free = |state: &GameState| {
        median(
            ratios(state)
                .into_iter()
                .filter(|(at, _)| matches!(at, Tightest::Lines(..) | Tightest::Hairpin(_)))
                .map(|(_, r)| r)
                .collect(),
        )
    };
    assert!(
        free(&two) > free(&one),
        "free pairs {:.3} -> {:.3}",
        free(&one),
        free(&two)
    );
    assert!(
        second.held_lines.len() < two.lines.len() && second.held_nodes < two.initial_node_count,
        "{second:?}"
    );
    assert!(same_face_model(&two));
    assert!(replays_to_the_same_model(&two));
}

/// Every settle ends within its round and work caps, one round's tests
/// over the cap at most; the least distance ratio never drops across a
/// settle, and a frozen pair ends at or above the ratio it was frozen at.
#[test]
fn the_leximin_ends_within_its_rounds_and_work() {
    let mut frozen_pairs = 0;
    for fixture in all_fixtures() {
        let mut drawn = fixture.start();
        for recorded in &fixture.moves {
            let mov = fixture_move(&drawn, recorded);
            drawn.apply_move(mov).unwrap();
            if drawn.settle_version > 0 {
                settle(&mut drawn);
            }
            let mut copy = drawn.clone();
            copy.settle_version = SETTLE_VERSION;
            let (before, _) = least_clearance(&copy);
            let report = settle_reported(&mut copy);
            let (after, at) = least_clearance(&copy);
            assert!(report.rounds <= ROUNDS_V2 + SHAPE_ROUNDS, "{report:?}");
            assert!(report.work <= WORK_CAP + 400_000, "{report:?}");
            assert!(
                after + 1e-9 >= before,
                "{} nodes, {} lines: {before} -> {after} at {at:?} {report:?}",
                fixture.initial_nodes,
                copy.lines.len()
            );
            let now = ratios(&copy);
            for (pair, floor) in &report.frozen {
                frozen_pairs += 1;
                let ratio = now[pair];
                assert!(
                    ratio + 1e-9 >= *floor,
                    "{pair:?} frozen at {floor:.4} ends at {ratio:.4}"
                );
            }
        }
    }
    assert!(frozen_pairs > 0);
}

/// The issue 98 game as recorded under version 1 hooks its fifth line
/// into node 11 with a corner under the bend target. The same game as
/// this Mac routed it under version 2 is kept as a fixture with every
/// applied spline, so what is checked here is the settle on pinned
/// splines, not the router: replayed, the line goes in clean and the
/// settle keeps it at the target or wider at every move after, and the
/// board hashes as recorded on macOS.
#[test]
fn the_hook_of_the_issue_98_game_does_not_form_under_version_2() {
    const ROUTED_HASH: u64 = 0xbb67fdbefd0f4c54;
    let hook = |state: &GameState| {
        ratios(state)
            .get(&Tightest::Bend(4))
            .copied()
            .unwrap_or(f64::INFINITY)
    };
    let recorded = replay::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/undrawn/pocket-4n-1000.json"),
    )
    .unwrap();
    assert_eq!(recorded.settle_version, 1);
    let mut hooked = recorded.start();
    for recorded in &recorded.moves {
        let mov = fixture_move(&hooked, recorded);
        hooked.apply_move(mov).unwrap();
        settle(&mut hooked);
    }
    let was = hook(&hooked);
    assert!(was < 1.0, "version 1 hooked the line at {was:.3}");

    let routed = replay::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/settle/pocket-routed-4n-1000.json"),
    )
    .unwrap();
    assert_eq!(routed.settle_version, 2);
    let mut state = routed.start();
    let mut least = f64::INFINITY;
    for recorded in &routed.moves {
        let mov = fixture_move(&state, recorded);
        state.apply_move(mov).unwrap();
        settle(&mut state);
        if state.lines.len() >= 5 {
            least = least.min(hook(&state));
        }
    }
    assert!(least >= 1.0, "version 2 let the line bend to {least:.3}");
    assert_eq!(hash(&state), ROUTED_HASH, "{:#x}", hash(&state));
    assert!(same_face_model(&state));
}

/// After every settle each line's made node sits where the line says it
/// does, over every fixture position and the version 2 seeded game.
#[test]
fn a_made_node_and_its_line_agree_after_every_settle() {
    let agree = |state: &GameState| {
        for line in &state.lines {
            let node = state.find_node(state.initial_node_count + line.id).unwrap();
            assert_eq!(
                node.position, line.new_node_pos,
                "line {} and node {} disagree",
                line.id, node.id
            );
        }
    };
    for fixture in all_fixtures() {
        let mut drawn = fixture.start();
        for recorded in &fixture.moves {
            let mov = fixture_move(&drawn, recorded);
            drawn.apply_move(mov).unwrap();
            if drawn.settle_version > 0 {
                settle(&mut drawn);
                agree(&drawn);
            }
            let mut copy = drawn.clone();
            copy.settle_version = SETTLE_VERSION;
            settle(&mut copy);
            agree(&copy);
            settle(&mut copy);
            agree(&copy);
        }
    }
    let (state, moves) = play_random(9, &seeded_picks(9, 40), SETTLE_VERSION);
    assert!(moves.len() >= 6);
    agree(&state);
}

/// The shape phase runs after the distance pairs are done, so a board
/// carrying a kink settles its distances exactly as it would with shape
/// off: the same pairs freeze and the least ends no lower.
#[test]
fn shape_costs_the_distance_pairs_nothing() {
    let mut kinked = 0;
    for fixture in all_fixtures() {
        let mut drawn = fixture.start();
        for recorded in &fixture.moves {
            let mov = fixture_move(&drawn, recorded);
            drawn.apply_move(mov).unwrap();
            if drawn.settle_version > 0 {
                settle(&mut drawn);
            }
            let mut with = drawn.clone();
            with.settle_version = SETTLE_VERSION;
            let mut without = with.clone();
            let shaped = settle_reported(&mut with);
            let plain = settle_without_shape(&mut without);
            assert_eq!(
                shaped.frozen, plain.frozen,
                "{} nodes",
                fixture.initial_nodes
            );
            let (a, _) = least_clearance(&with);
            let (b, _) = least_clearance(&without);
            assert!(
                a + 1e-9 >= b,
                "{} nodes: shape left the least at {a} against {b}",
                fixture.initial_nodes
            );
            if ratios(&without)
                .iter()
                .any(|(at, r)| matches!(at, Tightest::Bend(_) | Tightest::Spokes(..)) && *r < 1.0)
            {
                kinked += 1;
            }
        }
    }
    assert!(
        kinked > 0,
        "no fixture position carries a kink to straighten"
    );
}

/// Settled again under version 2, the issue 98 recording keeps three
/// bends under target, every one on a line the distance rounds hold
/// still, so the shape phase has nothing free to move there.
#[test]
fn the_kinks_the_shape_phase_leaves_are_on_held_lines() {
    let fixture = replay::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/undrawn/pocket-4n-1000.json"),
    )
    .unwrap();
    let mut state = fixture.start();
    for recorded in &fixture.moves {
        let mov = fixture_move(&state, recorded);
        state.apply_move(mov).unwrap();
        settle(&mut state);
    }
    state.settle_version = SETTLE_VERSION;
    let report = settle_reported(&mut state);
    let under: Vec<Tightest> = ratios(&state)
        .into_iter()
        .filter(|(at, r)| matches!(at, Tightest::Bend(_) | Tightest::Spokes(..)) && *r < 1.0)
        .map(|(at, _)| at)
        .collect();
    assert!(!under.is_empty(), "{report:?}");
    for at in &under {
        let on_held = match at {
            Tightest::Bend(k) => report.held_lines.contains(k),
            Tightest::Spokes(_, k, l) => {
                report.held_lines.contains(k) || report.held_lines.contains(l)
            }
            _ => false,
        };
        assert!(on_held, "{at:?} is free and still under target: {report:?}");
    }
}

/// A chord drawn to 10 units short of a node and hooked down into it: the
/// shape phase straightens the hook past the target.
#[test]
fn a_free_hook_is_straightened_by_the_shape_phase() {
    let mut state = GameState::new(4);
    state.nodes[0].position = Point::new(200.0, 800.0);
    state.nodes[1].position = Point::new(600.0, 800.0);
    state.nodes[2].position = Point::new(200.0, 250.0);
    state.nodes[3].position = Point::new(800.0, 250.0);
    let mut hook: Vec<Point> = (0..=58)
        .map(|i| Point::new(200.0 + 10.0 * i as f64, 250.0 - 6.0 * (i as f64 / 58.0)))
        .collect();
    hook.extend([
        Point::new(790.0, 246.0),
        Point::new(796.0, 249.0),
        Point::new(799.0, 255.0),
        Point::new(800.0, 250.0),
    ]);
    let into = crate::types::Move::new(2, 3, hook, Point::new(500.0, 247.0), Player::Human, 1000);
    protocol::check(&state, &into).unwrap();
    state.apply_move(into).unwrap();
    let bend = |state: &GameState| {
        ratios(state)
            .get(&Tightest::Bend(0))
            .copied()
            .unwrap_or(f64::INFINITY)
    };
    let before = bend(&state);
    assert!(before < 1.0, "the hook reads as a kink: {before:.3}");
    let mut plain = state.clone();
    settle_without_shape(&mut plain);
    settle(&mut state);
    let after = bend(&state);
    assert!(
        after >= 1.0,
        "hook {before:.3} -> {after:.3}, {:.3} without shape",
        bend(&plain)
    );
    assert!(same_face_model(&state));
}

/// Two chords leaving a node a few degrees apart: the settle turns their
/// handles apart until the angle clears the target.
#[test]
fn close_spokes_at_a_node_are_turned_apart() {
    let mut state = GameState::new(4);
    state.nodes[0].position = Point::new(200.0, 500.0);
    state.nodes[1].position = Point::new(700.0, 470.0);
    state.nodes[2].position = Point::new(700.0, 530.0);
    state.nodes[3].position = Point::new(450.0, 850.0);
    for (to, y) in [(1usize, 470.0f64), (2, 530.0)] {
        let polyline: Vec<Point> = (0..=50)
            .map(|i| {
                let t = i as f64 / 50.0;
                Point::new(200.0 + 500.0 * t, 500.0 + (y - 500.0) * t)
            })
            .collect();
        let mid = polyline[25];
        let mov = crate::types::Move::new(0, to, polyline, mid, Player::Human, 1000);
        protocol::check(&state, &mov).unwrap();
        state.apply_move(mov).unwrap();
        state.current_player = Player::Human;
    }
    let chord = |state: &GameState| ratios(state)[&Tightest::Spokes(0, 0, 1)];
    let before = chord(&state);
    assert!(before < 1.0, "the spokes read as close: {before:.3}");
    settle(&mut state);
    let after = chord(&state);
    assert!(after >= 1.0, "spokes {before:.3} -> {after:.3}");
    assert!(same_face_model(&state));
}

/// Version 1 on the 24-node fixture, where the work cap binds: the rounds,
/// the work and the settled geometry over the game, as recorded on macOS.
/// Release only, since it runs 68 million pair tests.
#[test]
#[cfg(not(debug_assertions))]
fn version_1_settles_the_24_node_fixture_as_recorded() {
    const ROUNDS_24: usize = 918;
    const WORK_24: usize = 68_494_272;
    const HASH_24: u64 = 0x78fa209cff5f4fd7;
    let fixture = replay::load(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/settle/selfplay-24n-1000.json"),
    )
    .unwrap();
    let mut drawn = fixture.start();
    let (mut rounds, mut work) = (0, 0);
    let mut fnv = crate::exact::Fnv::default();
    for recorded in &fixture.moves {
        let mov = fixture_move(&drawn, recorded);
        drawn.apply_move(mov).unwrap();
        let mut copy = drawn.clone();
        copy.settle_version = 1;
        let report = settle_reported(&mut copy);
        rounds += report.rounds;
        work += report.work;
        for b in bits(&copy) {
            fnv.feed(f64::from_bits(b));
        }
    }
    assert_eq!(
        (rounds, work, fnv.0),
        (ROUNDS_24, WORK_24, HASH_24),
        "rounds {rounds}, work {work}, hash {}",
        fnv.hex()
    );
}

/// A game recorded at version 1 replays through the version 1 settle to
/// the same bytes on a fresh board.
#[test]
fn a_version_1_record_replays_through_version_1() {
    let (played, moves) = play_random(6, &seeded_picks(6, 40), 1);
    assert!(moves.len() >= 4);
    let mut state = GameState::with_nodes_at(crate::types::opening_layout(6, 1000), 1000, 1);
    state.current_player = Player::Human;
    for mov in &moves {
        assert!(matches!(
            protocol::apply(&mut state, mov.clone()),
            protocol::Outcome::Applied { .. }
        ));
    }
    assert_eq!(state.settle_version, 1);
    assert_eq!(bits(&state), bits(&played));
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// The pruned nearest-point scan finds the same distance as the full
    /// one on random chains and points.
    #[test]
    fn pruned_and_full_nearest_scans_agree(
        pts in proptest::collection::vec((0.0..1000.0f64, 0.0..1000.0f64), 4..=10),
        probes in proptest::collection::vec((-100.0..1100.0f64, -100.0..1100.0f64), 1..=8),
    ) {
        let cubics = 3 * ((pts.len() - 1) / 3) + 1;
        let pts: Vec<Point> = pts[..cubics].iter().map(|(x, y)| Point::new(x.round(), y.round())).collect();
        let targets = Targets::for_version(2, 1000);
        let make = |pruned: bool| {
            let mut chain = Chain {
                from: 0,
                to: 1,
                pts: pts.clone(),
                seg: 0,
                t: 0.0,
                smooth: Vec::new(),
                samples: Vec::new(),
                blocks: Vec::new(),
                pruned,
            };
            chain.resample(&targets);
            chain
        };
        let (full, pruned) = (make(false), make(true));
        for (x, y) in probes {
            let p = Point::new(x, y);
            let (_, a, _, _) = full.nearest(&p);
            let (_, b, _, _) = pruned.nearest(&p);
            prop_assert_eq!(a, b);
        }
    }
}

/// Replays a saved page record (strokes and face moves) from the path in
/// SPROUTS_RECORD under the settle version SPROUTS_SETTLE names, printing
/// the pair table after every move and the bend of every line at the end.
#[test]
#[ignore]
fn probe_record_shape() {
    let path = std::env::var("SPROUTS_RECORD").expect("SPROUTS_RECORD names a record");
    let version: u8 = std::env::var("SPROUTS_SETTLE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(SETTLE_VERSION);
    let raw: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let point = |v: &serde_json::Value| Point::new(v[0].as_f64().unwrap(), v[1].as_f64().unwrap());
    let nodes: Vec<Point> = raw["nodes"].as_array().unwrap().iter().map(point).collect();
    let first = if raw["firstPlayer"] == "ai" {
        Player::AI
    } else {
        Player::Human
    };
    let mut state = protocol::start(
        raw["initialNodes"].as_u64().unwrap() as usize,
        raw["boardSize"].as_u64().unwrap() as usize,
        first,
        Some(nodes),
        version,
    )
    .unwrap();
    println!("{path} under version {version}");
    println!("{}", header());
    for (index, m) in raw["moves"].as_array().unwrap().iter().enumerate() {
        let outcome = if m.get("face").is_some() {
            let face: protocol::FaceMove = serde_json::from_value(m["face"].clone()).unwrap();
            protocol::apply_face_move(&mut state, face, point(&m["node"]))
        } else {
            let wire = protocol::Move {
                from_node: m["from"].as_u64().unwrap() as usize,
                to_node: m["to"].as_u64().unwrap() as usize,
                polyline: m["polyline"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(point)
                    .collect(),
                new_node_pos: point(&m["newNode"]),
                segments: m
                    .get("segments")
                    .map(|s| serde_json::from_value(s.clone()).unwrap()),
            };
            protocol::apply(&mut state, wire)
        };
        if let protocol::Outcome::Rejected { rejection } = &outcome {
            println!("move {index} refused: {}", rejection.message);
            break;
        }
        let (least, at) = least_clearance(&state);
        let report = Report {
            least,
            at,
            ..Report::nothing()
        };
        println!("{}", describe(index, &report, &state));
    }
    let bends: Vec<String> = ratios(&state)
        .iter()
        .filter(|(at, _)| matches!(at, Tightest::Bend(_)))
        .map(|(at, r)| format!("{at:?} {r:.2}"))
        .collect();
    println!("bends: {}", bends.join(", "));
}

fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    values[values.len() / 2]
}

fn class(at: Tightest) -> &'static str {
    match at {
        Tightest::Nothing => "none",
        Tightest::Nodes(..) => "nodes",
        Tightest::LineNode(..) => "line-node",
        Tightest::Lines(..) => "lines",
        Tightest::Hairpin(_) => "hairpin",
        Tightest::NodeEdge(_) => "node-edge",
        Tightest::LineEdge(_) => "line-edge",
        Tightest::Bend(_) => "bend",
        Tightest::Spokes(..) => "spokes",
    }
}

const CLASSES: [&str; 8] = [
    "nodes",
    "line-node",
    "lines",
    "hairpin",
    "node-edge",
    "line-edge",
    "bend",
    "spokes",
];

/// The 10th percentile and the median of `values`, or dashes.
fn spread(values: &mut [f64]) -> String {
    if values.is_empty() {
        return "   -  /  -  ".to_string();
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let at = |q: f64| values[((values.len() - 1) as f64 * q).round() as usize];
    format!("{:5.2}/{:5.2}", at(0.1), at(0.5))
}

/// One line per settled position: the least ratio and its pair, the
/// rounds and share of the work cap the settle spent, why it ended, and
/// per pair class the p10 and median ratio after it.
fn describe(index: usize, report: &Report, state: &GameState) -> String {
    let ratios = ratios(state);
    let mut by_class: Vec<Vec<f64>> = vec![Vec::new(); CLASSES.len()];
    for (at, ratio) in &ratios {
        let slot = CLASSES.iter().position(|c| *c == class(*at)).unwrap();
        by_class[slot].push(*ratio);
    }
    let spreads: Vec<String> = by_class.iter_mut().map(|v| spread(v)).collect();
    format!(
        "m{index:>2} least {:5.2} {:<9} rounds {:>2} work {:>3.0}% {:<6} | {}",
        report.least,
        class(report.at),
        report.rounds,
        100.0 * report.work as f64 / WORK_CAP as f64,
        format!("{:?}", report.stop),
        spreads.join(" | ")
    )
}

fn header() -> String {
    format!(
        "{:<54}| {}",
        "position",
        CLASSES
            .iter()
            .map(|c| format!("{c:^11}"))
            .collect::<Vec<_>>()
            .join(" | ")
    )
}

/// Replays `moves` as recorded under the current settle, settling after
/// each, and prints the table.
fn report_game(name: &str, fixture: &replay::Fixture) {
    println!(
        "\n{name}: {} nodes, {} moves",
        fixture.initial_nodes,
        fixture.moves.len()
    );
    println!("{}", header());
    let mut state = fixture.start();
    let mut stops: BTreeMap<String, usize> = BTreeMap::new();
    let mut least_over_game = f64::INFINITY;
    for (index, recorded) in fixture.moves.iter().enumerate() {
        let mov = fixture_move(&state, recorded);
        state.apply_move(mov).unwrap();
        // A game recorded with no settle, or any game with SPROUTS_RESETTLE
        // set, has a copy of each position settled at the env version.
        let again = std::env::var("SPROUTS_RESETTLE").is_ok();
        let report = if state.settle_version == 0 || again {
            if state.settle_version != 0 {
                settle_reported(&mut state);
            }
            let mut copy = state.clone();
            copy.settle_version = std::env::var("SPROUTS_SETTLE")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(SETTLE_VERSION);
            let report = settle_reported(&mut copy);
            println!("{}", describe(index, &report, &copy));
            report
        } else {
            let report = settle_reported(&mut state);
            println!("{}", describe(index, &report, &state));
            report
        };
        *stops.entry(format!("{:?}", report.stop)).or_default() += 1;
        least_over_game = least_over_game.min(report.least);
    }
    println!("stops {stops:?}; least ratio over the game {least_over_game:.2}");
}

/// Where the room goes: seeded self-play at 6 and 12 nodes played under
/// the current settle, and the recorded games settled position by
/// position. Run with --ignored --nocapture in release.
#[test]
#[ignore]
fn probe_settle_room() {
    for nodes in [6usize, 12] {
        let (moves, stop) = replay::self_play(nodes, 1000, true);
        let fixture = replay::Fixture {
            version: replay::FIXTURE_VERSION,
            initial_nodes: nodes,
            board_size: 1000,
            settle_version: SETTLE_VERSION,
            nodes: Some(
                crate::types::opening_layout(nodes, 1000)
                    .iter()
                    .map(|p| [p.x, p.y])
                    .collect(),
            ),
            moves,
        };
        report_game(
            &format!("self-play {nodes} nodes, stopped {stop:?}"),
            &fixture,
        );
    }
    for path in [
        "settle/selfplay-24n-1000",
        "saved/eight-moves-4n-1000",
        "saved/ten-moves-9n-1000",
        "saved/finished-4n-500",
    ] {
        let fixture = replay::load(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("fixtures/{path}.json")),
        )
        .unwrap();
        report_game(&format!("fixture {path}"), &fixture);
    }
}

/// The page record at SPROUTS_RECORD (strokes and face moves) as a record
/// of face moves alone, each with the node it lands on when the engine
/// routes and settles it under the settle version SPROUTS_SETTLE names;
/// printed as the JSON the page restores from localStorage.
#[test]
#[ignore]
fn print_topology_record() {
    let path = std::env::var("SPROUTS_RECORD").expect("SPROUTS_RECORD names a record");
    let version: u8 = std::env::var("SPROUTS_SETTLE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(SETTLE_VERSION);
    let raw: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let point = |v: &serde_json::Value| Point::new(v[0].as_f64().unwrap(), v[1].as_f64().unwrap());
    let nodes: Vec<Point> = raw["nodes"].as_array().unwrap().iter().map(point).collect();
    let first = if raw["firstPlayer"] == "ai" {
        Player::AI
    } else {
        Player::Human
    };
    let count = raw["initialNodes"].as_u64().unwrap() as usize;
    let board = raw["boardSize"].as_u64().unwrap() as usize;
    let recorded = raw["settleVersion"].as_u64().unwrap_or(0) as u8;
    // The record replayed as it was made gives every move's face move.
    let mut given = protocol::start(count, board, first, Some(nodes.clone()), recorded).unwrap();
    let mut faces: Vec<protocol::FaceMove> = Vec::new();
    let mut strokes: Vec<Option<protocol::Move>> = Vec::new();
    for m in raw["moves"].as_array().unwrap() {
        let outcome = if m.get("face").is_some() {
            let face: protocol::FaceMove = serde_json::from_value(m["face"].clone()).unwrap();
            faces.push(face.clone());
            strokes.push(None);
            protocol::apply_face_move(&mut given, face, point(&m["node"]))
        } else {
            let wire = protocol::Move {
                from_node: m["from"].as_u64().unwrap() as usize,
                to_node: m["to"].as_u64().unwrap() as usize,
                polyline: m["polyline"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(point)
                    .collect(),
                new_node_pos: point(&m["newNode"]),
                segments: m
                    .get("segments")
                    .map(|s| serde_json::from_value(s.clone()).unwrap()),
            };
            let entered = protocol::enter(wire.clone(), &given).unwrap();
            faces.push(crate::embedding::embed(&given, &entered).unwrap().into());
            strokes.push(Some(wire.clone()));
            protocol::apply(&mut given, wire)
        };
        assert!(
            matches!(outcome, protocol::Outcome::Applied { .. }),
            "the record replays as made: {outcome:?}"
        );
    }
    let mut state = protocol::start(count, board, first, Some(nodes), version).unwrap();
    let mut moves = Vec::new();
    for (index, face) in faces.iter().enumerate() {
        let listed: crate::face::Move = face.clone().into();
        let mut landed = None;
        let mut applied_polyline = Vec::new();
        let mut applied_segments = None;
        let mut refused = String::new();
        match crate::route::route(&state, &listed) {
            Ok(routed) => {
                let flat = routed.flatten(curve::tolerance(board));
                let mut tries = vec![flat[flat.len() / 2]];
                tries.extend(flat.iter().copied());
                for at in tries {
                    match protocol::apply_face_move(&mut state, face.clone(), at) {
                        protocol::Outcome::Applied { mov, .. } => {
                            landed = Some(mov.new_node_pos);
                            applied_polyline = mov.polyline.clone();
                            applied_segments = mov.segments.clone();
                            break;
                        }
                        protocol::Outcome::Rejected { rejection } => refused = rejection.message,
                    }
                }
            }
            Err(why) => eprintln!("move {index} does not route under version {version}: {why:?}"),
        }
        if let Some(node) = landed {
            // SPROUTS_AS_STROKES writes the applied spline instead, which
            // replays bit for bit on any platform as a version 1 fixture.
            if std::env::var("SPROUTS_AS_STROKES").is_ok() {
                let line = state.lines.last().unwrap();
                moves.push(serde_json::json!({
                    "from": line.from_node,
                    "to": line.to_node,
                    "polyline": applied_polyline.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>(),
                    "newNode": [node.x, node.y],
                    "segments": applied_segments,
                }));
                continue;
            }
            moves.push(serde_json::json!({
                "from": face.a,
                "to": face.b,
                "face": face,
                "node": [node.x, node.y],
            }));
            continue;
        }
        // The stroke as recorded, when the router will not draw the move
        // on this board.
        let Some(wire) = strokes[index].clone() else {
            panic!("move {index}: no point on the routed curve takes the node ({refused})");
        };
        let outcome = protocol::apply(&mut state, wire.clone());
        assert!(
            matches!(outcome, protocol::Outcome::Applied { .. }),
            "move {index}: the stroke is refused too: {outcome:?} ({refused})"
        );
        moves.push(serde_json::json!({
            "from": wire.from_node,
            "to": wire.to_node,
            "polyline": wire.polyline.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>(),
            "newNode": [wire.new_node_pos.x, wire.new_node_pos.y],
            "segments": wire.segments,
        }));
    }
    let record = serde_json::json!({
        "version": 1,
        "initialNodes": count,
        "boardSize": board,
        "nodes": raw["nodes"],
        "firstPlayer": raw["firstPlayer"],
        "cursor": moves.len(),
        "settleVersion": version,
        "moves": moves,
    });
    println!("RECORD {record}");
}
