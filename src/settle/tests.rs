use super::*;
use crate::protocol::{self, SETTLE_VERSION};
use crate::replay;
use crate::types::grid;
use crate::types::Player;
use proptest::prelude::*;
use std::path::Path;

/// The recorded games were played with no settling, so their strokes are
/// applied as drawn and the settle is measured on a copy of each
/// position at the current version.
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

/// Every coordinate on the board, bit for bit.
pub(crate) fn bits(state: &GameState) -> Vec<u64> {
    let mut out = Vec::new();
    for node in &state.nodes {
        out.extend([node.position.x.to_bits(), node.position.y.to_bits()]);
    }
    for line in &state.lines {
        for c in &line.segments {
            for p in [c.p0, c.p1, c.p2, c.p3] {
                out.extend([p.x.to_bits(), p.y.to_bits()]);
            }
        }
        out.extend([line.new_node_pos.x.to_bits(), line.new_node_pos.y.to_bits()]);
        for p in &line.polyline {
            out.extend([p.x.to_bits(), p.y.to_bits()]);
        }
    }
    out
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

fn same_face_model(state: &GameState) -> bool {
    let targets = Targets::for_board(state.board_size);
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
    for fixture in fixtures() {
        let mut drawn = fixture.start();
        for recorded in &fixture.moves {
            let mov = fixture_move(&drawn, recorded);
            drawn.apply_move(mov).unwrap();
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

/// A chord bent through a control point, or a loop round a node, from
/// the indices a random game picks, as the page would send it.
fn random_move(state: &GameState, pick: (usize, usize, f64, f64)) -> Option<protocol::Move> {
    let n = state.nodes.len();
    let (a, b, bend, side) = (pick.0 % n, pick.1 % n, pick.2, pick.3);
    let from = state.nodes[a].position;
    let to = state.nodes[b].position;
    let polyline: Vec<Point> = if a == b {
        let radius = 30.0 + 40.0 * bend;
        (0..=40)
            .map(|i| {
                let angle = side * std::f64::consts::TAU + std::f64::consts::TAU * i as f64 / 40.0;
                Point::new(
                    from.x + radius + radius * (angle + std::f64::consts::PI).cos(),
                    from.y + radius * (angle + std::f64::consts::PI).sin(),
                )
            })
            .collect()
    } else {
        let d = Point::new(to.x - from.x, to.y - from.y);
        let len = d.x.hypot(d.y);
        let normal = Point::new(-d.y / len, d.x / len);
        let bulge = (side - 0.5) * 2.0 * bend * len * 0.6;
        let ctrl = Point::new(
            (from.x + to.x) / 2.0 + normal.x * bulge,
            (from.y + to.y) / 2.0 + normal.y * bulge,
        );
        (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                let u = 1.0 - t;
                Point::new(
                    u * u * from.x + 2.0 * u * t * ctrl.x + t * t * to.x,
                    u * u * from.y + 2.0 * u * t * ctrl.y + t * t * to.y,
                )
            })
            .collect()
    };
    let new_node = polyline[20];
    let wire = protocol::Move {
        from_node: a,
        to_node: b,
        polyline,
        new_node_pos: new_node,
        segments: None,
    };
    let mov = crate::types::Move::new(
        a,
        b,
        wire.polyline.clone(),
        new_node,
        state.current_player,
        state.board_size,
    );
    protocol::check(state, &mov).ok()?;
    Some(wire)
}

/// A random game played through the protocol, settling after each move,
/// with the moves as the engine applied them, which is what a record and
/// a link carry.
fn play_random(
    nodes: usize,
    picks: &[(usize, usize, f64, f64)],
) -> (GameState, Vec<protocol::Move>) {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
    let mut played = Vec::new();
    for pick in picks {
        if state.face.is_game_over() {
            break;
        }
        // The check ran on the stroke as drawn; the engine rounds it to the
        // grid first, which can refuse a marginal one.
        if let Some(mov) = random_move(&state, *pick) {
            if let protocol::Outcome::Applied { mov, .. } = protocol::apply(&mut state, mov) {
                played.push(mov);
            }
        }
    }
    (state, played)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

    #[test]
    fn random_games_settle_to_the_same_bytes_in_two_passes(
        nodes in 3usize..=9,
        picks in proptest::collection::vec((0usize..64, 0usize..64, 0.0..1.0f64, 0.0..1.0f64), 40),
    ) {
        let (first, played) = play_random(nodes, &picks);
        let (second, again) = play_random(nodes, &picks);
        prop_assert!(!played.is_empty());
        prop_assert_eq!(played.len(), again.len());
        prop_assert_eq!(bits(&first), bits(&second));
        prop_assert!(same_face_model(&first));
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
    crate::types::Move::new(
        recorded.from,
        recorded.to,
        recorded
            .polyline
            .iter()
            .map(|p| Point::new(p[0], p[1]))
            .collect(),
        Point::new(recorded.new_node[0], recorded.new_node[1]),
        state.current_player,
        state.board_size,
    )
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

/// Picks for a random game from a small multiplicative generator, so the
/// games are fixed across runs.
fn seeded_picks(seed: u64, count: usize) -> Vec<(usize, usize, f64, f64)> {
    let mut x = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    let mut next = || {
        x = x
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (x >> 11) as f64 / (1u64 << 53) as f64
    };
    (0..count)
        .map(|_| {
            let a = (next() * 64.0) as usize;
            let b = (next() * 64.0) as usize;
            (a, b, next(), next())
        })
        .collect()
}

/// Games played with the settle, as the page plays them, at 3 to 9 nodes,
/// with the moves as sent.
fn settled_games() -> Vec<(GameState, Vec<protocol::Move>)> {
    (3..=9)
        .map(|nodes| play_random(nodes, &seeded_picks(nodes as u64, 40)))
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

/// A 64-bit fold of every coordinate on the board.
fn hash(state: &GameState) -> u64 {
    bits(state).iter().fold(0xcbf29ce484222325u64, |h, b| {
        (h ^ b).wrapping_mul(0x100000001b3)
    })
}

/// The settled board of the seeded 7-node random game, bit for bit. A
/// change here means version 1 settles differently: bump SETTLE_VERSION
/// in src/protocol.rs and record the new value.
const VERSION_1_GOLDEN: u64 = 0xa032cb53618d8318;

#[test]
fn version_1_settles_the_seeded_game_to_the_golden_board() {
    let (state, moves) = play_random(7, &seeded_picks(7, 40));
    assert!(moves.len() >= 6, "{} moves", moves.len());
    assert_eq!(state.settle_version, SETTLE_VERSION);
    assert_eq!(hash(&state), VERSION_1_GOLDEN, "{:#x}", hash(&state));
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

/// The settled board after each move of the seeded 9-node random game at
/// version 1, as recorded on macOS. A mismatch on another platform names
/// the first move that settles differently there; a mismatch after a
/// change to the settle means version 1 output changed, so bump
/// SETTLE_VERSION and record the new values.
const VERSION_1_SEEDED_GAME: &[u64] = &[
    0x7bb01be736536569,
    0x7a06061ba002b013,
    0xa9fe3221b9b96035,
    0x475be4ad2b1b4a3e,
    0xa19c5ada51285ec,
    0x98127e76221c6345,
    0x3d62a2f5e2c6e0bc,
    0x56516138c9de2925,
    0x759c44b36b8bcc5a,
];

#[test]
fn version_1_settles_the_seeded_game_the_same_on_every_platform() {
    let picks = seeded_picks(9, 40);
    let mut state = GameState::with_board_size(9, 1000);
    state.current_player = Player::Human;
    let mut hashes = Vec::new();
    for pick in &picks {
        if state.face.is_game_over() {
            break;
        }
        if let Some(mov) = random_move(&state, *pick) {
            if let protocol::Outcome::Applied { .. } = protocol::apply(&mut state, mov) {
                hashes.push(hash(&state));
            }
        }
    }
    let printed: Vec<String> = hashes.iter().map(|h| format!("{h:#x}")).collect();
    let first = hashes
        .iter()
        .zip(VERSION_1_SEEDED_GAME)
        .position(|(got, want)| got != want);
    assert_eq!(
        hashes.as_slice(),
        VERSION_1_SEEDED_GAME,
        "first differing move {first:?}; got {}",
        printed.join(" ")
    );
}
