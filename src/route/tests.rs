use super::sweep::{self, cramped, draws};
use super::*;
use crate::face::Move as FaceMove;
use crate::geometry::closest_point_on_polyline;
use crate::protocol;
use crate::types::{ai_node_clearance, node_disc, Player, MIN_NODE_SPACING_AI};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

fn p(x: f64, y: f64) -> Point {
    Point::new(x, y)
}

fn chord(a: NodeId, b: NodeId) -> FaceMove {
    FaceMove {
        a,
        at_a: None,
        b,
        at_b: None,
        enclosed: Vec::new(),
    }
}

fn straight(a: Point, b: Point) -> Vec<Point> {
    (0..=10)
        .map(|i| {
            let t = i as f64 / 10.0;
            p(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
        })
        .collect()
}

/// Three nodes in a row at y = 500: 0 at x = 200, 1 at 500, 2 at 800.
fn row() -> GameState {
    let mut state = GameState::new(3);
    for (node, x) in state.nodes.iter_mut().zip([200.0, 500.0, 800.0]) {
        node.position = p(x, 500.0);
    }
    state.current_player = Player::AI;
    state
}

fn listed(state: &GameState, a: NodeId, b: NodeId) -> Vec<FaceMove> {
    state
        .face
        .moves()
        .into_iter()
        .filter(|m| (m.a, m.b) == (a.min(b), a.max(b)))
        .collect()
}

fn closest_pair(state: &GameState) -> f64 {
    let mut best = f64::INFINITY;
    for (i, a) in state.nodes.iter().enumerate() {
        for b in &state.nodes[i + 1..] {
            best = best.min(a.position.distance_to(&b.position));
        }
    }
    best
}

#[test]
fn a_chord_on_an_empty_board_runs_straight_between_the_nodes() {
    let mut state = GameState::new(4);
    state.current_player = Player::AI;
    let mov = route(&state, &chord(0, 1)).unwrap();
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    let a = state.nodes[0].position;
    let b = state.nodes[1].position;
    assert_eq!(mov.polyline[0], a);
    assert_eq!(*mov.polyline.last().unwrap(), b);
    let flat = mov.flatten(1.0);
    for q in &flat {
        let (_, off) = crate::geometry::closest_point_on_segment(&a, &b, q);
        assert!(off < 2.0, "strayed {off} from the chord at {q:?}");
    }
    state.apply_move(mov).unwrap();
}

#[test]
fn a_chord_bends_around_a_node_in_its_way() {
    let state = row();
    let mov = route(&state, &chord(0, 2)).unwrap();
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    let middle = state.nodes[1].position;
    let clearance = curve::nearest(&mov.segments, &middle).unwrap().distance;
    assert!(clearance >= MIN_PATH_CLEARANCE_AI, "{clearance}");
    assert!(
        clearance <= NODE_ROOM * MIN_NODE_SPACING_AI + MIN_PATH_CLEARANCE_AI,
        "{clearance}"
    );
}

#[test]
fn a_loop_enclosing_one_node_walls_it_off() {
    let mut state = row();
    let moves = listed(&state, 0, 0);
    let around_one = moves
        .iter()
        .find(|m| m.enclosed == vec![1] || m.enclosed == vec![2])
        .expect("a loop around one neighbour");
    let mov = route(&state, around_one).unwrap();
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    state.apply_move(mov).unwrap();
    let face = &state.face;
    assert_ne!(
        face.corner_face(1, None).unwrap(),
        face.corner_face(2, None).unwrap()
    );
    assert_eq!(listed(&state, 1, 2), Vec::new());
}

#[test]
fn a_loop_enclosing_nothing_is_a_small_circle_clear_of_everything() {
    let mut state = row();
    let empty = listed(&state, 0, 0)
        .into_iter()
        .find(|m| m.enclosed.is_empty())
        .unwrap();
    let mov = route(&state, &empty).unwrap();
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    let a = state.nodes[0].position;
    let reach = mov
        .flatten(1.0)
        .iter()
        .map(|q| q.distance_to(&a))
        .fold(0.0, f64::max);
    let disc = node_disc(state.board_size);
    assert!(reach >= 3.0 * disc, "{reach}");
    assert!(reach <= 5.0 * disc, "{reach}");
    assert!(mov.new_node_pos.distance_to(&a) >= 2.0 * disc);
    state.apply_move(mov).unwrap();
    let face = &state.face;
    assert_eq!(
        face.corner_face(1, None).unwrap(),
        face.corner_face(2, None).unwrap()
    );
    assert_eq!(listed(&state, 1, 2).len(), 1);
}

/// Nodes 0 and 1 sit 40 apart between two lines that run `gap` apart.
fn corridor(board: usize, gap: f64) -> GameState {
    let s = scale(board);
    let mut state = GameState::with_board_size(6, board);
    let mid = board as f64 / 2.0;
    let spots = [
        p(mid - 20.0 * s, mid),
        p(mid + 20.0 * s, mid),
        p(200.0 * s, mid - gap / 2.0),
        p(800.0 * s, mid - gap / 2.0),
        p(200.0 * s, mid + gap / 2.0),
        p(800.0 * s, mid + gap / 2.0),
    ];
    for (node, at) in state.nodes.iter_mut().zip(spots) {
        node.position = at;
    }
    for (from, to) in [(2, 3), (4, 5)] {
        let a = state.nodes[from].position;
        let b = state.nodes[to].position;
        let line = straight(a, b);
        let near_start = line[1];
        state
            .apply_move(Move::new(from, to, line, near_start, Player::Human, board))
            .unwrap();
    }
    state.current_player = Player::AI;
    state
}

#[test]
fn a_corridor_one_clearance_wide_is_passable() {
    let state = corridor(1000, MIN_PATH_CLEARANCE_AI);
    let mov = route(&state, &chord(0, 1)).unwrap();
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    let mid = 500.0;
    for q in mov.flatten(0.5) {
        assert!((q.y - mid).abs() < MIN_PATH_CLEARANCE_AI / 2.0, "{q:?}");
    }
}

#[test]
fn a_mobile_sized_corridor_is_passable_too() {
    let state = corridor(500, MIN_PATH_CLEARANCE_AI * scale(500));
    let mov = route(&state, &chord(0, 1)).unwrap();
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    for q in mov.flatten(0.5) {
        assert!(
            (q.y - 250.0).abs() < MIN_PATH_CLEARANCE_AI * scale(500) / 2.0,
            "{q:?}"
        );
    }
}

#[test]
fn the_ai_opens_with_a_move_the_normal_validator_accepts() {
    for nodes in [4, 5, 6] {
        for board in [1000, 500] {
            let mut state = GameState::with_board_size(nodes, board);
            state.current_player = Player::AI;
            let moves = state.face.search_moves();
            let order = crate::ai::rank(&state, &moves);
            let (Some(mov), failures) = crate::ai::draw(&state, &moves, &order) else {
                panic!("{nodes} nodes at {board}: no opening move drawn");
            };
            assert_eq!(failures, 0, "{nodes} nodes at {board}");
            assert_eq!(
                validate_ai_move(&state, &mov),
                Ok(()),
                "{nodes} nodes at {board}"
            );
            let untouched = state
                .nodes
                .iter()
                .filter(|n| n.id != mov.from_node && n.id != mov.to_node)
                .map(|n| closest_point_on_polyline(&mov.polyline, &n.position).1)
                .fold(f64::INFINITY, f64::min);
            assert!(
                untouched >= ai_node_clearance(board),
                "{nodes} nodes at {board}: line {untouched:.1} from a node"
            );
            state.apply_move(mov).unwrap();
            let spacing = MIN_NODE_SPACING_AI * scale(board);
            assert!(
                closest_pair(&state) >= spacing,
                "{nodes} nodes at {board}: {}",
                closest_pair(&state)
            );
        }
    }
}

/// Share of listed moves that may fail to route for a reason other than
/// room over the random games, checked after every game once `SAMPLE`
/// routes have been tried; the measured share is in the PR and the bench
/// report, most of it lines that cannot get the node clearance past a
/// neighbour.
const ROUTE_FAILURE_CEILING: f64 = 0.2;
const SAMPLE: usize = 200;
/// Routes tried, and failed for reasons other than room, over every case
/// of the property test so far.
static TALLY: std::sync::Mutex<sweep::Tally> = std::sync::Mutex::new(sweep::Tally {
    tried: 0,
    failed: 0,
    kinds: std::collections::BTreeMap::new(),
});

/// Random games to play; an unoptimised build takes some twenty seconds
/// over each. The games are the same on every run, so a result here is
/// reproducible and the aggregate below is a fixed number.
#[cfg(debug_assertions)]
const CASES: u32 = 4;
#[cfg(not(debug_assertions))]
const CASES: u32 = 24;
const SEED: u64 = 7;

proptest! {
    #![proptest_config(Config {
        cases: CASES,
        rng_seed: RngSeed::Fixed(SEED),
        // A failing game is reported as it is; shrinking its picks replays
        // dozens of games and finds nothing simpler.
        max_shrink_iters: 0,
        ..Config::default()
    })]

    /// Random games where every move is drawn by the router: a random search
    /// move each turn, the list the AI ranks, falling back through the
    /// others when it cannot be drawn. A position with nothing drawable has
    /// failed on room alone, and the moves that fail otherwise stay rare.
    /// A failing position prints as a bench fixture.
    #[test]
    fn every_listed_move_routes(nodes in 4usize..=12, picks in prop::collection::vec(any::<u32>(), 36)) {
        let tally = sweep::game(nodes, &picks);
        prop_assert!(tally.is_ok(), "{}", tally.as_ref().unwrap_err());
        let mut total = TALLY.lock().unwrap();
        total.add(&tally.unwrap());
        eprintln!("routes failed {} of {}: {:?}", total.failed, total.tried, total.kinds);
        prop_assert!(
            total.tried < SAMPLE || total.failed as f64 <= ROUTE_FAILURE_CEILING * total.tried as f64,
            "{} of {} routes failed for reasons other than room: {:?}",
            total.failed, total.tried, total.kinds
        );
    }
}

/// Routes over the recorded positions that may still fail for a reason
/// other than room. The positions were picked because the router failed
/// in them, so this is the count to drive down, not a rate to hold.
const HARD_FAILURE_ALLOWANCE: usize = 40;

/// Positions from random games where the router once failed. Every
/// position keeps a move the router can draw, and the moves that fail for
/// a reason other than room stay within the allowance, each one listed.
#[test]
fn recorded_positions_keep_a_drawable_move() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/routing");
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    assert!(!paths.is_empty());
    let mut hard = Vec::new();
    let mut tried = 0usize;
    for path in &paths {
        let fixture = crate::replay::load(path).unwrap();
        let mut state = fixture.start();
        state.current_player = Player::AI;
        for m in &fixture.moves {
            let polyline: Vec<Point> = m.polyline.iter().map(|q| p(q[0], q[1])).collect();
            let mov = Move::new(
                m.from,
                m.to,
                polyline,
                p(m.new_node[0], m.new_node[1]),
                Player::AI,
                fixture.board_size,
            );
            state.apply_move(mov).unwrap();
            state.current_player = Player::AI;
        }
        let moves = state.face.search_moves();
        let mut drawn = 0;
        let mut reasons = Vec::new();
        for mv in &moves {
            match draws(&state, mv) {
                Ok(_) => drawn += 1,
                Err(why) => {
                    if !cramped(&why) {
                        hard.push(format!("{}: {mv:?} -> {why:?}", path.display()));
                    }
                    reasons.push(why);
                }
            }
        }
        tried += moves.len();
        assert!(
            drawn > 0 || reasons.iter().all(sweep::walled_in),
            "{}: no listed move routes, {reasons:?}",
            path.display()
        );
    }
    assert!(
        hard.len() <= HARD_FAILURE_ALLOWANCE,
        "{} of {tried} routes failed for reasons other than room:\n{}",
        hard.len(),
        hard.join("\n")
    );
}

/// Every move of every fixture under fixtures/games routed from its face
/// move in the position it was played, as text: the bits of each
/// flattening point and of the new node, or the failure.
#[cfg(not(debug_assertions))]
fn routed_fixtures() -> String {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/games");
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    let mut out = String::new();
    for path in &paths {
        let fixture = crate::replay::load(path).unwrap();
        let mut state = fixture.start();
        out.push_str(&format!("{}\n", path.display()));
        for (index, m) in fixture.moves.iter().enumerate() {
            let polyline: Vec<Point> = m.polyline.iter().map(|q| p(q[0], q[1])).collect();
            let mov = Move::new(
                m.from,
                m.to,
                polyline,
                p(m.new_node[0], m.new_node[1]),
                state.current_player,
                fixture.board_size,
            );
            let face = embedding::embed(&state, &mov).unwrap();
            match route(&state, &face) {
                Ok(drawn) => {
                    let tolerance = curve::tolerance(state.board_size);
                    let bits: Vec<String> = drawn
                        .flatten(tolerance)
                        .iter()
                        .chain(std::iter::once(&drawn.new_node_pos))
                        .map(|q| format!("{:x},{:x}", q.x.to_bits(), q.y.to_bits()))
                        .collect();
                    out.push_str(&format!("{index} {face:?} {}\n", bits.join(" ")));
                }
                Err(why) => out.push_str(&format!("{index} {face:?} {why:?}\n")),
            }
            state.apply_move(mov).unwrap();
        }
    }
    out
}

/// Names the file a child process writes its `routed_fixtures` to.
#[cfg(not(debug_assertions))]
const ROUTE_DUMP: &str = "SPROUTS_ROUTE_DUMP";

/// A link carries an AI move as its face move and the visitor's machine
/// draws it again, so the router has to give the same curve for the same
/// input in any process: here, this one and two more.
#[test]
#[cfg(not(debug_assertions))]
fn routing_gives_the_same_curves_in_another_process() {
    if let Ok(path) = std::env::var(ROUTE_DUMP) {
        std::fs::write(path, routed_fixtures()).unwrap();
        return;
    }
    let here = routed_fixtures();
    assert!(here.lines().any(|l| l.contains(',')), "nothing routed");
    let exe = std::env::current_exe().unwrap();
    for run in 0..2 {
        let dump =
            std::env::temp_dir().join(format!("sprouts-routes-{}-{run}.txt", std::process::id()));
        let status = std::process::Command::new(&exe)
            .args([
                "--exact",
                "route::tests::routing_gives_the_same_curves_in_another_process",
            ])
            .env(ROUTE_DUMP, &dump)
            .status()
            .unwrap();
        assert!(status.success());
        let there = std::fs::read_to_string(&dump).unwrap();
        let _ = std::fs::remove_file(&dump);
        if there != here {
            let (ours, theirs) = here
                .lines()
                .zip(there.lines())
                .find(|(a, b)| a != b)
                .unwrap_or(("", ""));
            panic!("run {run} routed differently:\n{ours}\n{theirs}");
        }
    }
}

/// Every forfeit in the seeded games with its diagnosis, for working the
/// router's remaining failures down in one pass: `cargo test --release
/// survey_forfeits -- --ignored --nocapture`.
#[test]
#[ignore]
fn survey_forfeits() {
    let report = sweep::survey(SEED, CASES as usize);
    eprintln!("{report}");
}
