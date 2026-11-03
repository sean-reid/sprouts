use super::*;
use crate::replay;
use crate::types::Player;
use proptest::prelude::*;
use std::path::Path;

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

fn bits(state: &GameState) -> Vec<u64> {
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
fn every_fixture_settles_without_changing_the_model_or_losing_clearance() {
    for fixture in fixtures() {
        let mut state = fixture.start();
        for (index, recorded) in fixture.moves.iter().enumerate() {
            let mov = crate::types::Move::new(
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
            );
            crate::protocol::check(&state, &mov)
                .unwrap_or_else(|e| panic!("move {index} rejected: {}", e.message));
            state.apply_move(mov).unwrap();
            let (before, _) = least_clearance(&state);
            let drawn = state.clone();
            settle(&mut state);
            let (after, _) = least_clearance(&state);
            assert!(
                after + 1e-9 >= before,
                "move {index}: least clearance fell from {before:.3} to {after:.3}"
            );
            assert!(same_face_model(&state), "move {index} changed the model");
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
    }
}

#[test]
fn settling_twice_moves_nothing_beyond_the_fit_tolerance() {
    for fixture in fixtures() {
        let mut state = fixture.start();
        for recorded in &fixture.moves {
            let mov = crate::types::Move::new(
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
            );
            state.apply_move(mov).unwrap();
            settle(&mut state);
            let once = state.clone();
            settle(&mut state);
            let moved = drift(&once, &state);
            assert!(
                moved <= curve::tolerance(state.board_size),
                "second settle moved {moved:.3} after {} moves of a {} node game",
                once.lines.len(),
                fixture.initial_nodes
            );
        }
    }
}

/// A chord bent through a control point, or a loop round a node, from
/// the indices a random game picks.
fn random_move(state: &GameState, pick: (usize, usize, f64, f64)) -> Option<crate::types::Move> {
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
    let mov = crate::types::Move::new(
        a,
        b,
        polyline,
        new_node,
        state.current_player,
        state.board_size,
    );
    crate::protocol::check(state, &mov).ok()?;
    Some(mov)
}

fn play_random(nodes: usize, picks: &[(usize, usize, f64, f64)]) -> (GameState, usize) {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
    let mut played = 0;
    for pick in picks {
        if state.face.is_game_over() {
            break;
        }
        if let Some(mov) = random_move(&state, *pick) {
            state.apply_move(mov).unwrap();
            settle(&mut state);
            played += 1;
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
        prop_assert!(played > 0);
        prop_assert_eq!(played, again);
        prop_assert_eq!(bits(&first), bits(&second));
        prop_assert!(same_face_model(&first));
    }
}

#[test]
fn a_fresh_board_and_a_clear_chord_are_left_alone() {
    let mut state = GameState::new(4);
    let before = bits(&state);
    settle(&mut state);
    assert_eq!(bits(&state), before);
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
    let before = bits(&state);
    settle(&mut state);
    assert_eq!(bits(&state), before);
    assert_eq!(state.lines[0].polyline, polyline);
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
    assert!(on_curve < 1e-6);
    assert!(same_face_model(&state));
    let shift = MAX_SHIFT * MIN_NODE_SPACING;
    assert!(
        state.nodes[2]
            .position
            .distance_to(&Point::new(500.0, 512.0))
            <= shift
    );
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

/// No node or control point moves more than MAX_SHIFT node spacings in
/// one settle; the drift over a whole game is printed for the record.
#[test]
fn nothing_moves_more_than_the_shift_in_one_settle() {
    for fixture in all_fixtures() {
        let mut state = fixture.start();
        let bound = MAX_SHIFT * MIN_NODE_SPACING * scale(state.board_size);
        let mut drift: f64 = 0.0;
        let mut first_seen: Vec<Point> = Vec::new();
        for recorded in &fixture.moves {
            let mov = fixture_move(&state, recorded);
            state.apply_move(mov).unwrap();
            let before: Vec<Point> = state.nodes.iter().map(|n| n.position).collect();
            let controls: Vec<Vec<Cubic>> =
                state.lines.iter().map(|l| l.segments.clone()).collect();
            if first_seen.len() < before.len() {
                first_seen.extend_from_slice(&before[first_seen.len()..]);
            }
            settle(&mut state);
            for (line, was) in state.lines.iter().zip(&controls) {
                for (c, w) in line.segments.iter().zip(was) {
                    for (p, q) in [(c.p0, w.p0), (c.p1, w.p1), (c.p2, w.p2), (c.p3, w.p3)] {
                        let moved = p.distance_to(&q);
                        assert!(
                            moved <= bound + 1e-6,
                            "{} nodes: a control point of line {} moved {moved:.2} in one settle",
                            fixture.initial_nodes,
                            line.id
                        );
                    }
                }
            }
            for (node, was) in state.nodes.iter().zip(&before) {
                let moved = node.position.distance_to(was);
                assert!(
                    moved <= bound + 1e-6,
                    "{} nodes: node {} moved {moved:.2} in one settle",
                    fixture.initial_nodes,
                    node.id
                );
                drift = drift.max(node.position.distance_to(&first_seen[node.id]));
            }
        }
        println!(
            "{} nodes, board {}: a node drifted at most {drift:.1} units over {} moves",
            fixture.initial_nodes,
            fixture.board_size,
            fixture.moves.len()
        );
    }
}

/// Undo puts back the geometry the undone move was drawn on, so the
/// stroke that followed re-applies as it did.
#[test]
fn undo_restores_the_board_the_next_move_was_drawn_on() {
    for fixture in all_fixtures() {
        let mut state = fixture.start();
        let mut boards = vec![bits(&state)];
        for recorded in &fixture.moves {
            let mov = fixture_move(&state, recorded);
            state.apply_move(mov).unwrap();
            settle(&mut state);
            boards.push(bits(&state));
        }
        assert_eq!(state.settles.len(), fixture.moves.len());
        let finished = boards.last().unwrap().clone();
        while crate::protocol::undo(&mut state) {
            boards.pop();
            assert_eq!(
                bits(&state),
                *boards.last().unwrap(),
                "{} nodes: undo to {} moves left a different board",
                fixture.initial_nodes,
                state.lines.len()
            );
        }
        assert!(state.settles.is_empty());
        for recorded in &fixture.moves {
            let mov = fixture_move(&state, recorded);
            crate::protocol::check(&state, &mov).unwrap();
            state.apply_move(mov).unwrap();
            settle(&mut state);
        }
        assert_eq!(bits(&state), finished);
    }
}
