use super::sweep::{self, draws};
use super::*;
use crate::exact::Fnv;
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

/// The wall clearance the router erodes by.
fn eroded_by(board: usize) -> f64 {
    MIN_PATH_CLEARANCE_AI * scale(board) + curve::tolerance(board)
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
        assert!(off < 3.0, "strayed {off} from the chord at {q:?}");
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
    assert!(clearance >= ai_node_clearance(1000), "{clearance}");
    assert!(
        clearance <= NODE_ROOM * MIN_NODE_SPACING_AI + ai_node_clearance(1000),
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

/// After node 0 loops round node 1, a chord from 0 to 2 has to go round
/// the loop and keeps the line clearance from it the whole way.
#[test]
fn a_chord_keeps_its_clearance_from_a_loop_it_rounds() {
    let mut state = row();
    let around_one = listed(&state, 0, 0)
        .into_iter()
        .find(|m| m.enclosed.len() == 1)
        .expect("a loop around one node");
    let loop_move = route(&state, &around_one).unwrap();
    state.apply_move(loop_move).unwrap();
    state.current_player = Player::AI;
    let loop_flat = state.lines[0].flat.clone();
    // The chord goes to the node the loop leaves outside.
    let inside = crate::geometry::point_in_polygon(&loop_flat, &state.nodes[1].position);
    let other = if inside { 2 } else { 1 };
    let moves = listed(&state, 0, other);
    assert!(!moves.is_empty());
    let mov = route(&state, &moves[0]).unwrap_or_else(|e| panic!("{e:?}: {:?}", moves[0]));
    assert_eq!(protocol::check(&state, &mov), Ok(()));
    let a = state.nodes[0].position;
    let nearest = mov
        .flatten(0.5)
        .iter()
        .filter(|q| q.distance_to(&a) > 3.0 * MIN_PATH_CLEARANCE_AI)
        .map(|q| closest_point_on_polyline(&loop_flat, q).1)
        .fold(f64::INFINITY, f64::min);
    assert!(nearest >= MIN_PATH_CLEARANCE_AI, "{nearest}");
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
    assert!(
        reach >= 3.0 * disc - curve::tolerance(state.board_size),
        "{reach}"
    );
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

/// A passage narrower than two clearances is closed; the same passage one
/// clearance wider is open, and the line runs down its middle.
#[test]
fn a_passage_narrower_than_two_clearances_has_no_room() {
    for board in [1000, 500] {
        let c = eroded_by(board);
        let closed = corridor(board, 2.0 * c - 1.0);
        assert!(
            matches!(route(&closed, &chord(0, 1)), Err(Unroutable::Room)),
            "{board}"
        );
        let open = corridor(board, 3.0 * c - 1.0);
        let mov = route(&open, &chord(0, 1)).unwrap_or_else(|e| panic!("{board}: {e:?}"));
        assert_eq!(protocol::check(&open, &mov), Ok(()));
        let mid = board as f64 / 2.0;
        for q in mov.flatten(0.5) {
            assert!((q.y - mid).abs() < c / 2.0, "{board}: {q:?}");
        }
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
            let (Ok(mov), failures) = crate::ai::draw(&state, &moves, &order) else {
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

/// Routes tried over every case of the property test so far, by outcome.
static TALLY: std::sync::Mutex<sweep::Tally> = std::sync::Mutex::new(sweep::Tally {
    tried: 0,
    room: 0,
    no_spot: 0,
    shape: std::collections::BTreeMap::new(),
    topology: 0,
    sides: 0,
    bug: 0,
    bugs: std::collections::BTreeMap::new(),
});

/// The same over the human-rules games.
static HUMAN_TALLY: std::sync::Mutex<sweep::Tally> = std::sync::Mutex::new(sweep::Tally {
    tried: 0,
    room: 0,
    no_spot: 0,
    shape: std::collections::BTreeMap::new(),
    topology: 0,
    sides: 0,
    bug: 0,
    bugs: std::collections::BTreeMap::new(),
});

/// Random games to play; an unoptimised build takes some twenty seconds
/// over each. The games are the same on every run, so a result here is
/// reproducible and the aggregate below is a fixed number.
#[cfg(debug_assertions)]
const CASES: u32 = 4;
#[cfg(not(debug_assertions))]
const CASES: u32 = 24;
const SEED: u64 = 7;
/// Read-back mismatches the seeded sweep shows as the AI and as the
/// human. The sweep is seeded and platform-exact, so these are the
/// measured values, and a router change that moves them is recorded here.
const AI_TOPOLOGY_CEILING: usize = 4;
const HUMAN_TOPOLOGY_CEILING: usize = 2;

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
    /// others when it cannot be drawn. No route may fail as a bug; a
    /// failing position prints as a bench fixture.
    #[test]
    fn no_listed_move_fails_as_a_bug(nodes in 4usize..=12, picks in prop::collection::vec(any::<u32>(), 36)) {
        let tally = sweep::game(nodes, &picks, Player::AI);
        prop_assert!(tally.is_ok(), "{}", tally.as_ref().unwrap_err());
        let mut total = TALLY.lock().unwrap();
        total.add(&tally.unwrap());
        eprintln!("{total}");
        prop_assert_eq!(total.bug, 0);
        prop_assert!(total.topology <= AI_TOPOLOGY_CEILING, "{}", total);
    }

    /// The same games drawn at the human clearances, which routeMove and
    /// applyFaceMove use for the player at the keyboard.
    #[test]
    fn no_listed_move_fails_as_a_bug_for_the_human(nodes in 4usize..=12, picks in prop::collection::vec(any::<u32>(), 36)) {
        let tally = sweep::game(nodes, &picks, Player::Human);
        prop_assert!(tally.is_ok(), "{}", tally.as_ref().unwrap_err());
        let mut total = HUMAN_TALLY.lock().unwrap();
        total.add(&tally.unwrap());
        eprintln!("human {total}");
        prop_assert_eq!(total.bug, 0);
        prop_assert!(total.topology <= HUMAN_TOPOLOGY_CEILING, "{}", total);
    }
}

/// Positions from random games where an earlier router failed. Every
/// position keeps a move the router can draw, and no route fails as a bug.
#[test]
fn recorded_positions_keep_a_drawable_move() {
    let paths = fixture_paths("fixtures/routing");
    assert!(!paths.is_empty());
    let mut tally = sweep::Tally::default();
    let mut undrawn = Vec::new();
    for path in &paths {
        let state = position(path);
        let moves = state.face.search_moves();
        let mut drawn = 0;
        for mv in &moves {
            tally.tried += 1;
            match draws(&state, mv) {
                Ok(_) => drawn += 1,
                Err(Unroutable::Bug(what)) => {
                    panic!("{}: {mv:?} failed as a bug: {what}", path.display())
                }
                Err(why) => tally.count(&why),
            }
        }
        eprintln!(
            "{}: {drawn} of {} moves drawn",
            path.file_name().unwrap().to_string_lossy(),
            moves.len()
        );
        if drawn == 0 {
            undrawn.push(path.display().to_string());
        }
    }
    eprintln!("{tally}");
    assert!(undrawn.is_empty(), "no listed move routes in {undrawn:?}");
}

/// The position after every move of a fixture, with the AI to move.
fn position(path: &std::path::Path) -> GameState {
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
    state
}

/// The JSON files under `dir`, sorted.
fn fixture_paths(dir: &str) -> Vec<std::path::PathBuf> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(dir);
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    paths.sort();
    paths
}

/// The bits of every point of a drawn move's flattening and of its new
/// node.
#[cfg(not(debug_assertions))]
fn bits(state: &GameState, drawn: &Move) -> String {
    let tolerance = curve::tolerance(state.board_size);
    let bits: Vec<String> = drawn
        .flatten(tolerance)
        .iter()
        .chain(std::iter::once(&drawn.new_node_pos))
        .map(|q| format!("{:x},{:x}", q.x.to_bits(), q.y.to_bits()))
        .collect();
    bits.join(" ")
}

/// Every move of every fixture under fixtures/games routed from its face
/// move in the position it was played, then every search move of every
/// position under fixtures/routing, as text: the bits of each flattening
/// point and of the new node, or the failure.
#[cfg(not(debug_assertions))]
fn routed_fixtures() -> String {
    let mut out = String::new();
    for path in fixture_paths("fixtures/games") {
        let fixture = crate::replay::load(&path).unwrap();
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
                Ok(drawn) => out.push_str(&format!("{index} {face:?} {}\n", bits(&state, &drawn))),
                Err(why) => out.push_str(&format!("{index} {face:?} {why:?}\n")),
            }
            state.apply_move(mov).unwrap();
        }
    }
    for path in fixture_paths("fixtures/routing") {
        let state = position(&path);
        out.push_str(&format!("{}\n", path.display()));
        for mv in state.face.search_moves() {
            match route(&state, &mv) {
                Ok(drawn) => out.push_str(&format!("{mv:?} {}\n", bits(&state, &drawn))),
                Err(why) => out.push_str(&format!("{mv:?} {why:?}\n")),
            }
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

/// A game the recorder played: the record as the page saves it, the
/// face move the link carries for each move the router redraws, and the
/// board after the last move settled.
struct SharedGame {
    fixture: crate::replay::Fixture,
    topology: Vec<Option<protocol::FaceMove>>,
    board: Vec<Point>,
}

/// Plays a game of `moves` moves on `nodes` nodes at the current settle,
/// the human moving first: human lines are routed at twice the human
/// clearances, so the rounded strokes of a link still pass, and then
/// kinked by more than the redraw tolerance, so they travel as geometry;
/// AI lines are the first search move the router draws. Every move goes
/// in the way the page sends it. A game the AI cannot finish is an error
/// naming the move.
fn shared_game(nodes: usize, moves: usize, pick: usize) -> Result<SharedGame, String> {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
    // The layout a record carries is where the nodes started; the settle
    // moves them as the game goes on.
    let layout: Vec<[f64; 2]> = state
        .nodes
        .iter()
        .map(|n| [n.position.x, n.position.y])
        .collect();
    let kink = 4.0 * curve::tolerance(state.board_size);
    let human = rules(&state);
    let slack = Rules {
        clearance: 2.0 * human.clearance,
        node_clearance: 2.0 * human.node_clearance,
        fallback: 2.0 * human.fallback,
        ..human
    };
    let mut record = Vec::new();
    for index in 0..moves {
        let wire = if state.current_player == Player::Human {
            // Long chords first, as a hand plays across a fresh board; a
            // loop's stroke costs the link more.
            let mut listed = state.face.search_moves();
            let span = |m: &FaceMove| {
                state.nodes[m.a]
                    .position
                    .distance_to(&state.nodes[m.b].position)
            };
            listed.sort_by(|x, y| {
                (x.a == x.b)
                    .cmp(&(y.a == y.b))
                    .then(span(y).total_cmp(&span(x)))
            });
            let n = listed.len();
            listed
                .iter()
                .cycle()
                .skip(pick % n)
                .take(n)
                .find_map(|face| {
                    let routed = Router::new()
                        .route_at(&state, face, &slack, Parity::Search)
                        .ok()?;
                    // One smooth bulge along the length, so the stroke
                    // still fits in a few cubics and differs from the
                    // router's line.
                    let mut along = vec![0.0];
                    for pair in routed.polyline.windows(2) {
                        along.push(along.last().unwrap() + pair[0].distance_to(&pair[1]));
                    }
                    let length = *along.last().unwrap();
                    let bent: Vec<Point> = routed
                        .polyline
                        .iter()
                        .zip(&along)
                        .map(|(q, s)| {
                            let t = (std::f64::consts::PI * s / length).sin();
                            Point::new(q.x + kink * t, q.y + kink * t)
                        })
                        .collect();
                    let wire = protocol::Move {
                        from_node: face.a,
                        to_node: face.b,
                        polyline: bent,
                        new_node_pos: routed.new_node_pos,
                        segments: None,
                    };
                    let mov = protocol::enter(wire.clone(), &state).ok()?;
                    protocol::check(&state, &mov).ok()?;
                    let drawn = crate::embedding::embed(&state, &mov).ok()?;
                    // A stroke the router happens to redraw would travel
                    // as topology, so the game takes the next chord.
                    (drawn.enclosed.len() == face.enclosed.len()
                        && !protocol::redraws(&state, face, &mov))
                    .then_some(wire)
                })
                .ok_or_else(|| format!("no human move at move {index}"))?
        } else {
            // The first search move that draws, routed without a clock, so
            // the record is the same on every run under any load.
            let moves = state.face.search_moves();
            let mut last = Err(Unroutable::Room);
            for mv in &moves {
                last = draws(&state, mv);
                if last.is_ok() {
                    break;
                }
            }
            match last {
                Ok(mov) => protocol::Move::from(mov),
                Err(why) => return Err(format!("no AI move drawn at move {index}: {why:?}")),
            }
        };
        match protocol::apply(&mut state, wire) {
            protocol::Outcome::Applied { mov, .. } => {
                record.push(crate::replay::FixtureMove::from_applied(&mov))
            }
            protocol::Outcome::Rejected { rejection } => {
                return Err(format!("move {index} rejected: {}", rejection.message))
            }
        }
    }
    let fixture = crate::replay::Fixture {
        version: crate::replay::FIXTURE_VERSION,
        initial_nodes: state.initial_node_count,
        board_size: state.board_size,
        nodes: Some(layout),
        settle_version: state.settle_version,
        moves: record,
    };
    let topology = protocol::classify(protocol::Classification {
        initial_nodes: fixture.initial_nodes,
        board_size: fixture.board_size,
        first_player: Player::Human,
        nodes: None,
        settle_version: fixture.settle_version,
        moves: fixture.moves.iter().map(|m| m.wire()).collect(),
        known: vec![None; fixture.moves.len()],
    })
    .unwrap()
    .topology;
    Ok(SharedGame {
        fixture,
        topology,
        board: state.nodes.iter().map(|n| n.position).collect(),
    })
}

/// Writes a shared game to `path` in the saved record shape.
fn record_shared_game(nodes: usize, moves: usize, pick: usize, path: &std::path::Path) {
    let game = shared_game(nodes, moves, pick).unwrap();
    let mut json = serde_json::to_value(&game.fixture).unwrap();
    json["firstPlayer"] = serde_json::json!("human");
    json["cursor"] = serde_json::json!(moves);
    std::fs::write(path, json.to_string()).unwrap();
}

/// How far a visitor's replay of a shared game lands from the sharer's:
/// the routed node of each topology move and the settled board at the
/// end, both measured on the link's rounded coordinates, which the settle
/// keeps on the unit grid.
#[derive(Debug, Default)]
struct Drift {
    topology: usize,
    refused: Option<usize>,
    node: f64,
    board: f64,
    /// The first move an index link's replay refuses, and the bytes the
    /// link's moves take.
    index_refused: Option<usize>,
    index_bytes: usize,
}

/// Replays a shared game the way a visitor's engine replays its link and
/// measures the drift.
fn rehearse_shared_game(game: &SharedGame) -> Drift {
    let fixture = &game.fixture;
    let quantum = scale(fixture.board_size);
    let round = |q: Point| crate::exact::snap_to_unit(q, quantum);
    let moves = fixture
        .moves
        .iter()
        .zip(&game.topology)
        .map(|(m, face)| {
            let wire = m.wire();
            match face {
                Some(face) => protocol::LinkMove::Routed {
                    face: face.clone(),
                    at: round(wire.new_node_pos),
                },
                None => {
                    // The link carries the cubics; the visitor's page
                    // flattens them for the stroke.
                    let segments: Vec<curve::Cubic> = wire
                        .segments
                        .as_deref()
                        .expect("a settled record carries its spline")
                        .iter()
                        .map(|c| curve::Cubic {
                            p0: round(c.p0),
                            p1: round(c.p1),
                            p2: round(c.p2),
                            p3: round(c.p3),
                        })
                        .collect();
                    let polyline = curve::flatten(&segments, 1.5 * quantum);
                    protocol::LinkMove::Drawn {
                        mov: protocol::Move {
                            from_node: wire.from_node,
                            to_node: wire.to_node,
                            polyline,
                            new_node_pos: round(wire.new_node_pos),
                            segments: Some(segments),
                        },
                    }
                }
            }
        })
        .collect();
    let rehearsed = protocol::rehearse(protocol::Rehearsal {
        router_version: protocol::ROUTER_VERSION,
        settle_version: fixture.settle_version,
        list_version: protocol::MOVE_LIST_VERSION,
        initial_nodes: fixture.initial_nodes,
        board_size: fixture.board_size,
        first_player: Player::Human,
        nodes: fixture
            .nodes
            .as_ref()
            .map(|nodes| nodes.iter().map(|q| round(p(q[0], q[1]))).collect()),
        moves,
    })
    .unwrap();
    let indexed = protocol::index_moves(protocol::Classification {
        initial_nodes: fixture.initial_nodes,
        board_size: fixture.board_size,
        first_player: Player::Human,
        nodes: None,
        settle_version: fixture.settle_version,
        moves: fixture.moves.iter().map(|m| m.wire()).collect(),
        known: vec![None; fixture.moves.len()],
    });
    let (indexed, index_refused) = match indexed {
        Ok(protocol::Indexed {
            moves,
            refused: None,
        }) => {
            let by_index = protocol::rehearse(protocol::Rehearsal {
                router_version: protocol::ROUTER_VERSION,
                settle_version: fixture.settle_version,
                list_version: protocol::MOVE_LIST_VERSION,
                initial_nodes: fixture.initial_nodes,
                board_size: fixture.board_size,
                first_player: Player::Human,
                nodes: None,
                moves: moves
                    .iter()
                    .map(|m| protocol::LinkMove::Indexed {
                        index: m.index,
                        along: m.along,
                    })
                    .collect(),
            })
            .unwrap();
            assert_eq!(
                by_index.refused, None,
                "indexing passed but the replay refused"
            );
            (moves, None)
        }
        Ok(protocol::Indexed {
            refused: Some(at), ..
        }) => {
            eprintln!("  indexing: the regenerated board refuses move {at}");
            (Vec::new(), Some(at))
        }
        Err(why) => panic!("indexing: {why}"),
    };
    let varint =
        |v: usize| 1 + (usize::BITS - v.max(1).leading_zeros()).saturating_sub(1) as usize / 7;
    let mut drift = Drift {
        topology: game.topology.iter().flatten().count(),
        refused: rehearsed.refused,
        index_refused,
        index_bytes: indexed
            .iter()
            .map(|m| varint(m.index * 2) + usize::from(m.along > 0))
            .sum(),
        ..Drift::default()
    };
    for (m, node) in fixture.moves.iter().zip(&rehearsed.nodes) {
        if let Some(node) = node {
            let recorded = p(m.new_node[0], m.new_node[1]);
            drift.node = drift.node.max(node.distance_to(&recorded));
        }
    }
    if rehearsed.refused.is_none() {
        assert_eq!(rehearsed.settled.len(), game.board.len());
        for (a, b) in rehearsed.settled.iter().zip(&game.board) {
            drift.board = drift.board.max(a.distance_to(b));
        }
    }
    drift
}

/// The games the recorder can play: eight moves on four nodes for picks
/// under 9, ten moves on nine nodes from 9 up; 0 and 36 are the committed
/// recordings.
fn shared_game_at(pick: usize) -> Result<SharedGame, String> {
    if pick < 9 {
        shared_game(4, 8, pick)
    } else {
        shared_game(9, 10, pick)
    }
}

/// A visitor's engine regenerates the sharer's board exactly: both engines
/// hold the same cubics, the same face moves and the same router, so
/// every AI move travels as topology and every routed node and settled
/// position comes back as recorded. Picks 0 and 36 are the committed
/// recordings.
#[test]
#[cfg(not(debug_assertions))]
fn a_shared_game_replays_to_the_same_board() {
    // Pick 1 is out: the recorder cannot apply its move 3, a chord the
    // router draws within a unit of its own spoke, which the unit-grid
    // snap on entry turns into a crossing.
    for pick in [0, 2, 3, 4, 9, 10, 11, 12, 17, 36] {
        let game = shared_game_at(pick).unwrap();
        let drift = rehearse_shared_game(&game);
        assert_eq!(
            drift.topology,
            game.fixture.moves.len() / 2,
            "pick {pick}: an AI move travels as geometry"
        );
        assert_eq!(drift.refused, None, "pick {pick}");
        assert_eq!(drift.node, 0.0, "pick {pick}: {drift:?}");
        assert_eq!(drift.board, 0.0, "pick {pick}: {drift:?}");
    }
}

/// The drift table over the 21 picks the recorder can play:
/// `cargo test --release measure_shared_game_drift -- --ignored --nocapture`.
#[test]
#[ignore]
fn measure_shared_game_drift() {
    eprintln!("pick nodes moves topology refused node_drift board_drift index_refused index_bytes");
    for pick in 0..21 {
        let game = match shared_game_at(pick) {
            Ok(game) => game,
            Err(why) => {
                eprintln!("{pick} not recorded: {why}");
                continue;
            }
        };
        let drift = rehearse_shared_game(&game);
        eprintln!(
            "{pick} {} {} {} {} {:.3} {:.3} {} {}",
            game.fixture.initial_nodes,
            game.fixture.moves.len(),
            drift.topology,
            drift.refused.map_or("none".to_string(), |i| i.to_string()),
            drift.node,
            drift.board,
            drift
                .index_refused
                .map_or("none".to_string(), |i| i.to_string()),
            drift.index_bytes
        );
    }
}

/// Rewrites the four shared-game fixtures tests/e2e/replay-links.spec.ts
/// shares, for when the router's output changes:
/// `cargo test --release record_shared_games -- --ignored`. The pick,
/// from SPROUTS_PICK_4, SPROUTS_PICK_9, SPROUTS_PICK_12 and
/// SPROUTS_PICK_24 when set, is
/// which chord the human opens with. The record is deterministic while
/// the AI's draw stays inside its routing budget.
#[test]
#[ignore]
fn record_shared_games() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/saved");
    let pick = |var: &str, default: usize| {
        std::env::var(var)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    };
    record_shared_game(
        4,
        8,
        pick("SPROUTS_PICK_4", 0),
        &dir.join("eight-moves-4n-1000.json"),
    );
    record_shared_game(
        9,
        10,
        pick("SPROUTS_PICK_9", 36),
        &dir.join("ten-moves-9n-1000.json"),
    );
    record_shared_game(
        12,
        30,
        pick("SPROUTS_PICK_12", 5),
        &dir.join("thirty-moves-12n-1000.json"),
    );
    record_shared_game(
        24,
        10,
        pick("SPROUTS_PICK_24", 0),
        &dir.join("ten-moves-24n-1000.json"),
    );
}

/// Records every position of the bench's seeded AI games where a route
/// reads back as another move, under fixtures/topology, and prints the
/// moves: `cargo test --release record_topology_positions -- --ignored
/// --nocapture`.
#[test]
#[ignore]
fn record_topology_positions() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/topology");
    std::fs::create_dir_all(&dir).unwrap();
    for entry in std::fs::read_dir(&dir).unwrap().filter_map(Result::ok) {
        std::fs::remove_file(entry.path()).unwrap();
    }
    let mut written = 0;
    for (game, (nodes, picks)) in sweep::games(7, 24).into_iter().enumerate() {
        let mut state = GameState::with_board_size(nodes, 1000);
        state.current_player = Player::AI;
        for &pick in &picks {
            let moves = state.face.search_moves();
            if moves.is_empty() {
                break;
            }
            let first = pick as usize % moves.len();
            let mut drawn = None;
            let mut saved = false;
            for k in (0..moves.len()).map(|k| (first + k) % moves.len()) {
                match draws(&state, &moves[k]) {
                    Ok(mov) => {
                        drawn = Some(mov);
                        break;
                    }
                    Err(Unroutable::Topology) if !saved => {
                        let name = format!("game{game}-{nodes}n-{}m.json", state.lines.len());
                        std::fs::write(dir.join(&name), sweep::fixture_json(&state)).unwrap();
                        eprintln!("{name}: {:?}", moves[k]);
                        written += 1;
                        saved = true;
                    }
                    Err(_) => {}
                }
            }
            let Some(mov) = drawn else { break };
            state.apply_move(mov).unwrap();
            state.current_player = Player::AI;
        }
    }
    eprintln!("{written} positions written");
}

/// Read-back mismatches the four recorded positions show; the measured
/// value, since the positions and the router are exact.
const RECORDED_TOPOLOGY: usize = 1;

/// Positions where a route read back as another move when recorded. No
/// route fails as a bug, and the mismatches do not grow.
#[test]
fn recorded_topology_positions_stay_bounded() {
    let paths = fixture_paths("fixtures/topology");
    assert!(!paths.is_empty());
    let mut tally = sweep::Tally::default();
    for path in &paths {
        let state = position(path);
        for mv in state.face.search_moves() {
            tally.tried += 1;
            match draws(&state, &mv) {
                Ok(_) => {}
                Err(Unroutable::Bug(what)) => {
                    panic!("{}: {mv:?} failed as a bug: {what}", path.display())
                }
                Err(why) => tally.count(&why),
            }
        }
    }
    eprintln!("{tally}");
    assert!(tally.topology <= RECORDED_TOPOLOGY, "{tally}");
}

/// The first seeded sweep game played out six moves by the recorder's
/// rule, a position the hand-made fixtures do not reach.
fn sweep_position() -> GameState {
    let (nodes, picks) = sweep::games(SEED, 1).remove(0);
    // Sweep positions are recorded as drawn, with no settle.
    let mut state = GameState::with_nodes_at(crate::types::opening_layout(nodes, 1000), 1000, 0);
    state.current_player = Player::AI;
    for pick in picks.iter().take(6) {
        let moves = state.face.search_moves();
        if moves.is_empty() {
            break;
        }
        let first = *pick as usize % moves.len();
        let drawn = (0..moves.len())
            .map(|k| (first + k) % moves.len())
            .find_map(|k| draws(&state, &moves[k]).ok());
        let Some(mov) = drawn else { break };
        state.apply_move(mov).unwrap();
        state.current_player = Player::AI;
    }
    state
}

/// Hashes of each routing stage over every search move of the first
/// `ROUTING_HASH_FIXTURES` routing fixtures and one seeded sweep game, in
/// path order: arena vertices, taut path, relaxed path, fitted cubics,
/// and the flattening with the node or the refusal. A platform that
/// disagrees shows where it diverged.
fn routing_hashes() -> [u64; 5] {
    let mut stages: [Fnv; 5] = Default::default();
    for state in hash_fixture_paths().iter().map(|path| position(path)) {
        let want = rules(&state);
        for mv in state.face.search_moves() {
            if mv.a != mv.b || !mv.enclosed.is_empty() {
                if let Ok((arena, taut)) =
                    plan(&mut Router::new(), &state, &mv, &want, Parity::Search)
                {
                    stages[0].points(&arena.vertex_points());
                    stages[1].points(&taut);
                    if let Some(relaxed) = relax(&state, &mv, &want, &arena, &taut) {
                        stages[2].points(&relaxed);
                    }
                }
            }
            if let Ok(drawn) = route(&state, &mv) {
                for c in &drawn.segments {
                    stages[3].points(&[c.p0, c.p1, c.p2, c.p3]);
                }
            }
        }
        hash_routes(&mut stages[4], &state);
    }
    stages.map(|f| f.0)
}

/// Prints one hash per routed search move of the hash fixtures, "plan/move
/// hex" as the wasm build's routeHashes lists them, for finding the move a
/// platform routes apart: `cargo test --release print_route_hashes --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn print_route_hashes() {
    for (index, path) in hash_fixture_paths().iter().enumerate() {
        let state = position(path);
        for (at, mv) in state.face.search_moves().iter().enumerate() {
            let mut fnv = Fnv::default();
            super::hash_route(&mut fnv, &state, mv);
            eprintln!("{index}/{at} {} {mv:?}", fnv.hex());
        }
    }
}

/// The first `ROUTING_HASH_FIXTURES` routing fixtures and the sweep
/// fixture, in the order the hashes take them.
fn hash_fixture_paths() -> Vec<std::path::PathBuf> {
    fixture_paths("fixtures/routing")
        .into_iter()
        .take(ROUTING_HASH_FIXTURES)
        .chain(std::iter::once(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(SWEEP_FIXTURE),
        ))
        .collect()
}

/// Routing fixtures the hash covers; six keep the debug run short.
const ROUTING_HASH_FIXTURES: usize = 6;
/// The first seeded sweep game after six moves, recorded by
/// `record_sweep_position` so the wasm test hashes the same position.
const SWEEP_FIXTURE: &str = "fixtures/routing/sweep-6n-6m.json";

/// The final routing hash a Mac records; the Linux runner and the wasm
/// build (tests/unit/route-hash.test.ts) have to produce the same one,
/// since a share link is redrawn on the visitor's device.
const ROUTING_HASH: u64 = 0xa39a_4017_2306_5eef;

#[test]
fn routing_hashes_the_same_on_every_platform() {
    let [arena, taut, relaxed, cubics, flat] = routing_hashes();
    assert_eq!(
        flat, ROUTING_HASH,
        "routing hashes: arena {arena:#018x} taut {taut:#018x} relaxed {relaxed:#018x} cubics {cubics:#018x} flat {flat:#018x}"
    );
}

/// A routed move stored as its polyline and node refits to the same bytes
/// through Move::new, which is how a record and a share link rebuild it.
#[test]
fn a_routed_move_refits_to_itself() {
    let mut checked = 0;
    for path in fixture_paths("fixtures/routing")
        .into_iter()
        .take(ROUTING_HASH_FIXTURES)
    {
        let state = position(&path);
        for mv in state.face.search_moves() {
            let Ok(drawn) = route(&state, &mv) else {
                continue;
            };
            let again = Move::new(
                drawn.from_node,
                drawn.to_node,
                drawn.polyline.clone(),
                drawn.new_node_pos,
                drawn.player,
                state.board_size,
            );
            assert_eq!(again.segments, drawn.segments, "{}: {mv:?}", path.display());
            assert_eq!(
                again.new_node_pos,
                drawn.new_node_pos,
                "{}: {mv:?}",
                path.display()
            );
            checked += 1;
        }
    }
    assert!(checked > 50, "{checked} routes checked");
}

/// Writes `SWEEP_FIXTURE`: `cargo test --release record_sweep_position --
/// --ignored`.
#[test]
#[ignore]
fn record_sweep_position() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(SWEEP_FIXTURE);
    std::fs::write(path, sweep::fixture_json(&sweep_position())).unwrap();
}

/// The recorded sweep position replays to the position it was recorded
/// from, node for node and segment for segment, so the fixture hashes
/// what the recorder saw.
#[test]
fn the_sweep_fixture_replays_to_its_position() {
    let live = sweep_position();
    let loaded = position(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(SWEEP_FIXTURE));
    assert_eq!(live.nodes.len(), loaded.nodes.len());
    for (a, b) in live.nodes.iter().zip(&loaded.nodes) {
        assert_eq!(a.position, b.position);
    }
    assert_eq!(live.lines.len(), loaded.lines.len());
    for (a, b) in live.lines.iter().zip(&loaded.lines) {
        assert_eq!(a.segments, b.segments, "line {}", a.id);
        assert_eq!(a.new_node_pos, b.new_node_pos, "line {}", a.id);
    }
}

/// A settled 4-node position after five moves with the AI to move, where
/// node 3 has one spoke and a bare loop on it.
const LOOP_FIXTURE: &str = "fixtures/routing/loop-4n-5m.json";

#[test]
fn the_loop_fixture_lists_and_draws_the_loop_on_node_3() {
    let state = position(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(LOOP_FIXTURE));
    assert_eq!(state.nodes[3].position, p(713.0, 778.0));
    assert_eq!(state.face.rotation(3).len(), 1);
    let bare_loop = state
        .face
        .search_moves()
        .into_iter()
        .find(|mv| mv.a == 3 && mv.b == 3 && mv.enclosed.is_empty())
        .expect("the loop on node 3 is listed");
    assert_eq!(bare_loop.at_a, Some(8));
    route(&state, &bare_loop).unwrap();
}

/// Two ends closer than two guard radii, the entry already inside the
/// exit's guard: node 3 sits 7 units from node 0 on the outside of a wedge
/// of 0's own lines, and the chord from 0's corner inside the wedge has to
/// leave the exit's guard, go round the wedge and come back. Both chords
/// route.
#[test]
fn a_chord_between_ends_inside_each_others_guard_routes_round_a_detour() {
    let mut state = GameState::new(4);
    let (a, c1, c2) = (p(500.0, 500.0), p(300.0, 300.0), p(300.0, 700.0));
    for (node, at) in state.nodes.iter_mut().zip([a, c1, c2, p(800.0, 500.0)]) {
        node.position = at;
    }
    state.current_player = Player::Human;
    for (to, end) in [(1, c1), (2, c2)] {
        let mid = p((a.x + end.x) / 2.0, (a.y + end.y) / 2.0);
        state
            .apply_move(Move::new(0, to, straight(a, end), mid, Player::Human, 1000))
            .unwrap();
        state.current_player = Player::Human;
    }
    state.nodes[3].position = p(507.0, 500.0);
    let want = rules(&state);
    let guard = want.clearance + curve::tolerance(state.board_size);
    assert!(a.distance_to(&state.nodes[3].position) < 2.0 * guard);
    let moves = listed(&state, 0, 3);
    assert_eq!(moves.len(), 2);
    let routed: Vec<Result<f64, Unroutable>> = moves
        .iter()
        .map(|mv| {
            route(&state, mv).map(|drawn| {
                let path = drawn.polyline;
                path.windows(2).map(|w| w[0].distance_to(&w[1])).sum()
            })
        })
        .collect();
    // The corner facing node 3 has a 7-unit chord with no room for a node;
    // the corner inside the wedge goes round it.
    assert!(
        routed.contains(&Err(Unroutable::Shape("short"))),
        "{routed:?}"
    );
    assert!(
        routed.iter().any(|r| r.as_ref().is_ok_and(|&l| l > 400.0)),
        "{routed:?}"
    );
    // Once the detour has left node 0's guard it stays out of it, except
    // where node 3's guard overlaps, which is the exit's.
    let b = state.nodes[3].position;
    let detour = moves
        .iter()
        .find_map(|mv| route(&state, mv).ok())
        .expect("the detour routes");
    let left = detour
        .polyline
        .iter()
        .position(|q| q.distance_to(&a) > guard)
        .expect("the detour leaves node 0's guard");
    for q in &detour.polyline[left..] {
        assert!(
            q.distance_to(&a) > guard || q.distance_to(&b) <= guard,
            "{q:?} is back inside node 0's guard"
        );
    }
    // A loop has one end, so it starts inside its own guard as before. The
    // loops round node 3 from the wedge's far ends find no room, since node
    // 3's disc overlaps node 0's, and say so rather than fail as a bug.
    let loops: Vec<FaceMove> = state
        .face
        .moves()
        .into_iter()
        .filter(|m| m.a == m.b && m.enclosed == vec![3])
        .collect();
    assert_eq!(loops.len(), 2);
    for mv in &loops {
        assert!(matches!(route(&state, mv), Err(Unroutable::Room)), "{mv:?}");
    }
}

/// The guard rule on the bits alone: a chord that has left the exit's
/// guard may pass a triangle inside both guards on its way, and from there
/// may not go on into the entry's guard alone.
#[test]
fn a_path_through_an_overlap_may_not_slide_into_the_entry_guard() {
    use corridor::step_bits;
    let (entry, exit) = (1u8, 2u8);
    let both = entry | exit;
    // A chord sets out through its own guard, then free ground.
    assert_eq!(step_bits(entry, 0, exit, true), Some(true));
    // Into an overlap mid-route is allowed, and commits it to finishing.
    assert_eq!(step_bits(0, both, exit, true), Some(true));
    assert_eq!(step_bits(both, entry, exit, true), None);
    assert_eq!(step_bits(both, 0, exit, true), None);
    assert_eq!(step_bits(both, exit, exit, true), Some(true));
    // The entry's guard alone is never entered from outside it.
    assert_eq!(step_bits(0, entry, exit, true), None);
    assert_eq!(step_bits(exit, both, exit, true), Some(true));
    // A loop, or a chord starting inside the overlap, leaves the exit's
    // guard once and is then out.
    assert_eq!(step_bits(both, entry, exit, false), Some(true));
    assert_eq!(step_bits(exit, 0, exit, false), Some(true));
    assert_eq!(step_bits(0, exit, exit, true), Some(true));
}

/// Two legs through one vertex in opposite directions are set a quarter
/// clearance apart, each to its own left; a vertex revisited the same way
/// stays; a vertex three legs pinch at moves all three.
#[test]
fn split_pinches_parts_opposite_legs_and_leaves_parallel_ones() {
    let v = p(10.0, 0.0);
    let pinch = vec![p(0.0, 0.0), v, p(20.0, 5.0), p(20.0, -5.0), v, p(0.0, 0.0)];
    let split = split_pinches(pinch.clone(), 2.0, |_| true);
    assert_eq!(split[0], pinch[0]);
    assert_eq!(split[5], pinch[5]);
    assert!((split[1].distance_to(&v) - 2.0).abs() < 1e-9);
    assert!((split[4].distance_to(&v) - 2.0).abs() < 1e-9);
    assert!(split[1].y < 0.0 && split[4].y > 0.0, "{split:?}");

    let same_way = vec![
        p(-10.0, -10.0),
        p(0.0, 0.0),
        v,
        p(20.0, 0.0),
        p(20.0, 10.0),
        p(-10.0, -5.0),
        p(0.0, 0.0),
        v,
        p(20.0, -10.0),
    ];
    assert_eq!(split_pinches(same_way.clone(), 2.0, |_| true), same_way);

    let triple = vec![
        p(-5.0, 0.0),
        v,
        p(20.0, 0.0),
        v,
        p(0.0, 0.0),
        v,
        p(20.0, 10.0),
    ];
    let split = split_pinches(triple, 2.0, |_| true);
    for k in [1, 3, 5] {
        assert!((split[k].distance_to(&v) - 2.0).abs() < 1e-9, "{split:?}");
    }
    assert!(split[1].y < 0.0 && split[3].y > 0.0 && split[5].y < 0.0);
}

/// A hairpin at a vertex a second leg passes is parted too: the hairpin
/// backs out the way it came and the other leg moves to its left.
#[test]
fn split_pinches_parts_a_hairpin_from_a_pass_through_its_vertex() {
    let v = p(10.0, 0.0);
    let path = vec![
        p(0.0, 0.0),
        v,
        p(0.0, 0.0),
        p(10.0, -10.0),
        v,
        p(10.0, 10.0),
    ];
    let split = split_pinches(path, 2.0, |_| true);
    assert!((split[1].distance_to(&v) - 2.0).abs() < 1e-9, "{split:?}");
    assert!((split[4].distance_to(&v) - 2.0).abs() < 1e-9, "{split:?}");
    assert!(
        split[1].x < v.x && (split[1].y - v.y).abs() < 1e-9,
        "{split:?}"
    );
    assert!(split[4].distance_to(&split[1]) > 2.0, "{split:?}");
}

/// A leg whose left is not free stays on the vertex while the other leg
/// still parts from it, and legs with no free left both stay put.
#[test]
fn split_pinches_keeps_the_displaced_points_free() {
    let v = p(10.0, 0.0);
    let pinch = vec![p(0.0, 0.0), v, p(20.0, 5.0), p(20.0, -5.0), v, p(0.0, 0.0)];
    let split = split_pinches(pinch.clone(), 2.0, |q| q.y >= 0.0);
    assert_eq!(split[1], v, "{split:?}");
    assert!((split[4].distance_to(&v) - 2.0).abs() < 1e-9 && split[4].y > 0.0);
    let stuck = split_pinches(pinch.clone(), 2.0, |q| q.distance_to(&v) < 1.0);
    assert_eq!(stuck, pinch);
}

/// A pinch both of whose legs stay on their vertex, as when neither side
/// is free, still goes through the rest of the route: the loop on node 2
/// round node 3 is drawn from its taut path with the pinch left in, and
/// comes out as the move or as a shape refusal, never a bug.
#[test]
fn a_pinch_left_coincident_still_routes_or_is_refused_as_a_shape() {
    let mut state = GameState::new(7);
    state.current_player = Player::Human;
    let mv = FaceMove {
        a: 2,
        at_a: None,
        b: 2,
        at_b: None,
        enclosed: vec![3],
    };
    let want = rules(&state);
    let (arena, taut) = taut_path(&mut Router::new(), &state, &mv, &want, Parity::Search).unwrap();
    let n = taut.len();
    assert!(
        (1..n - 1).any(|i| (i + 1..n - 1).any(|j| taut[i].distance_to(&taut[j]) < 1e-9)),
        "the taut path pinches"
    );
    let path = settle(taut).unwrap();
    let drawn = match relax(&state, &mv, &want, &arena, &path) {
        Some(relaxed) => finish(&state, &mv, relaxed).or_else(|_| finish(&state, &mv, path)),
        None => finish(&state, &mv, path),
    };
    assert!(
        matches!(drawn, Ok(_) | Err(Unroutable::Shape(_))),
        "{:?}",
        drawn.map(|m| m.polyline.len())
    );
}

/// Two lines leaving one node 11 degrees apart make a wedge whose offset
/// curves meet at an exact mitre ten clearances out. Every point within
/// the clearance of both walls is outside the eroded region, the mouth
/// of the wedge is inside it, and the chords across the mouth draw.
#[test]
fn a_sharp_wedge_is_eroded_to_its_mitre_and_routes_across() {
    let mut state = GameState::new(3);
    let (a, b, c) = (p(200.0, 500.0), p(800.0, 440.0), p(800.0, 560.0));
    for (node, at) in state.nodes.iter_mut().zip([a, b, c]) {
        node.position = at;
    }
    state.current_player = Player::AI;
    for (to, end) in [(1, b), (2, c)] {
        let mid = p((a.x + end.x) / 2.0, (a.y + end.y) / 2.0);
        state
            .apply_move(Move::new(0, to, straight(a, end), mid, Player::AI, 1000))
            .unwrap();
        state.current_player = Player::AI;
    }
    let want = rules(&state);
    let clearance = want.clearance + curve::tolerance(state.board_size);
    let moves = listed(&state, 1, 2);
    assert!(!moves.is_empty());
    let face = state.face.corner_face(1, moves[0].at_a).unwrap();
    let arena = erode::Arena::build(
        &state,
        face,
        1,
        2,
        clearance,
        want.node_clearance + curve::tolerance(state.board_size),
        want.forgiven,
    )
    .unwrap();
    let walls = [(a, b), (a, c)];
    let within_both = |q: Point| {
        walls
            .iter()
            .all(|(u, v)| crate::geometry::closest_point_on_segment(u, v, &q).1 < clearance)
    };
    let disc = want.node_clearance + 2.0 * curve::tolerance(state.board_size);
    let wall_distance = |q: Point| {
        walls
            .iter()
            .map(|(u, v)| crate::geometry::closest_point_on_segment(u, v, &q).1)
            .fold(f64::INFINITY, f64::min)
    };
    let inside_wedge = |q: Point| {
        cross(a, b, q) > 0.0 && cross(a, c, q) < 0.0 || cross(a, b, q) < 0.0 && cross(a, c, q) > 0.0
    };
    let off_nodes = |q: Point| {
        state
            .nodes
            .iter()
            .all(|n| n.position.distance_to(&q) > disc)
    };
    let (mut blocked, mut open) = (0, 0);
    for x in (200..800).step_by(2) {
        for y in (440..=560).step_by(2) {
            let q = p(x as f64, y as f64);
            if !inside_wedge(q) {
                continue;
            }
            assert!(arena.contains(q), "{q:?} is outside the triangulation");
            if within_both(q) {
                assert!(
                    !arena.free_at(q),
                    "{q:?} is free within {clearance} of both walls"
                );
                blocked += 1;
            } else if wall_distance(q) > clearance + 1.0 && off_nodes(q) {
                assert!(
                    arena.free_at(q),
                    "{q:?} is eroded {} from the nearest wall",
                    wall_distance(q)
                );
                open += 1;
            }
        }
    }
    assert!(
        blocked > 50 && open > 100,
        "{blocked} blocked and {open} open points checked"
    );
    for mv in &moves {
        route(&state, mv).unwrap_or_else(|e| panic!("{e:?}: {mv:?}"));
    }
}

/// A straight line from node 0 to node 1 with node 3 on it: a shortcut
/// into node 3 from above is accepted through the corner that faces up
/// and refused through the one that faces down, and the other way round
/// from below.
#[test]
fn a_shortcut_into_an_end_has_to_arrive_through_its_corner() {
    let mut state = row();
    let (a, b) = (state.nodes[0].position, state.nodes[1].position);
    let mid = p((a.x + b.x) / 2.0, 500.0);
    state
        .apply_move(Move::new(0, 1, straight(a, b), mid, Player::AI, 1000))
        .unwrap();
    state.current_player = Player::AI;
    let want = rules(&state);
    let c = want.clearance + curve::tolerance(state.board_size);
    let mv = listed(&state, 0, 3)
        .into_iter()
        .next()
        .expect("a chord from 0 to its new node");
    let face = state.face.corner_face(mv.a, mv.at_a).unwrap();
    let arena = erode::Arena::build(
        &state,
        face,
        mv.a,
        mv.b,
        c,
        want.node_clearance + curve::tolerance(state.board_size),
        want.forgiven,
    )
    .unwrap();
    let entry = arena.entries(&state, mv.a, mv.at_a).unwrap();
    let exit = arena.entries(&state, mv.b, mv.at_b).unwrap();
    let cuts = arena
        .cuts(&state, &mv, face, &corridor::blocked(&arena, &entry, &exit))
        .unwrap();
    let corners: Vec<erode::Entry> = state
        .face
        .rotation(3)
        .iter()
        .map(|&h| arena.entries(&state, 3, Some(h)).unwrap())
        .collect();
    assert_eq!(corners.len(), 2);
    let above = p(mid.x - 23.7, mid.y - 48.1);
    let below = p(mid.x - 23.7, mid.y + 48.1);
    let arrives = |corner: &erode::Entry, from: Point| {
        arena
            .segment_parity(&cuts, from, mid, None, Some(corner))
            .is_some()
    };
    assert!(arrives(&corners[0], above) || arrives(&corners[1], above));
    assert!(arrives(&corners[0], below) || arrives(&corners[1], below));
    // The same segments end inside node 3's guard, so with no end allowed
    // the guard rule refuses them.
    assert!(arena
        .segment_parity(&cuts, above, mid, None, None)
        .is_none());
    assert!(arena
        .segment_parity(&cuts, below, mid, None, None)
        .is_none());
    assert_ne!(arrives(&corners[0], above), arrives(&corners[1], above));
    assert_ne!(arrives(&corners[0], below), arrives(&corners[1], below));
    assert_ne!(arrives(&corners[0], above), arrives(&corners[0], below));
}

/// The flattenings of every line at node `v`, the node's own spokes.
fn spokes_at(state: &GameState, v: NodeId) -> Vec<Vec<Point>> {
    state
        .face
        .rotation(v)
        .iter()
        .map(|&h| crate::embedding::half_edge_path(state, h))
        .collect()
}

/// Every position of the seeded AI sweep games plus the routing
/// fixtures, with the AI to move.
fn probe_positions() -> Vec<GameState> {
    let mut out: Vec<GameState> = fixture_paths("fixtures/routing")
        .iter()
        .map(|path| position(path))
        .collect();
    for (nodes, picks) in sweep::games(SEED, CASES as usize) {
        let mut state =
            GameState::with_nodes_at(crate::types::opening_layout(nodes, 1000), 1000, 0);
        state.current_player = Player::AI;
        for &pick in &picks {
            let moves = state.face.search_moves();
            if moves.is_empty() {
                break;
            }
            let first = pick as usize % moves.len();
            let drawn = (0..moves.len())
                .map(|k| (first + k) % moves.len())
                .find_map(|k| draws(&state, &moves[k]).ok());
            let Some(mov) = drawn else { break };
            out.push(state.clone());
            state.apply_move(mov).unwrap();
            state.current_player = Player::AI;
        }
    }
    out
}

/// Measures the relaxation at the ends over every search move of the
/// probe positions: how many routes leave the first or last interior
/// point where the taut path put it while the next point moved, split by
/// whether that end has spokes, and how close the first point sits to
/// the end's own spokes: `cargo test --release probe_end_pins -- --ignored
/// --nocapture`.
#[test]
#[ignore]
fn probe_end_pins() {
    let mut routes = 0;
    let mut relaxed_routes = 0;
    let mut pinned = [0usize; 2];
    let mut ends_seen = [0usize; 2];
    let mut gaps: Vec<f64> = Vec::new();
    for state in probe_positions() {
        let want = rules(&state);
        for mv in state.face.search_moves() {
            if mv.a == mv.b && mv.enclosed.is_empty() {
                continue;
            }
            routes += 1;
            let Ok((arena, taut)) = plan(&mut Router::new(), &state, &mv, &want, Parity::Search)
            else {
                continue;
            };
            let Some(relaxed) = relax(&state, &mv, &want, &arena, &taut) else {
                continue;
            };
            relaxed_routes += 1;
            let before =
                clearance::resample(&taut, clearance::SPACING * super::eroded_by(&state, &want));
            assert_eq!(before.len(), relaxed.len());
            let n = relaxed.len();
            if n < 5 {
                continue;
            }
            let same = |i: usize| before[i].distance_to(&relaxed[i]) < 1e-9;
            for (end, first, second) in [(mv.a, 1, 2), (mv.b, n - 2, n - 3)] {
                let spokes = spokes_at(&state, end);
                let has = usize::from(!spokes.is_empty());
                ends_seen[has] += 1;
                if same(first) && !same(second) {
                    pinned[has] += 1;
                }
                if !spokes.is_empty() {
                    let gap = spokes
                        .iter()
                        .map(|s| closest_point_on_polyline(s, &relaxed[first]).1)
                        .fold(f64::INFINITY, f64::min);
                    gaps.push(gap / want.clearance);
                }
            }
        }
    }
    gaps.sort_by(f64::total_cmp);
    let pct = |q: f64| gaps[((gaps.len() - 1) as f64 * q) as usize];
    eprintln!(
        "{routes} routes planned, {relaxed_routes} relaxed; ends with spokes {}, pinned {}; bare ends {}, pinned {}",
        ends_seen[1], pinned[1], ends_seen[0], pinned[0]
    );
    eprintln!(
        "first point to own spoke, in line clearances: min {:.3} p10 {:.3} median {:.3} p90 {:.3} over {} ends",
        pct(0.0),
        pct(0.1),
        pct(0.5),
        pct(0.9),
        gaps.len()
    );
}

/// Half-angles of the corner between node 0's spokes in the spokes
/// fixtures, in degrees, with the rise of nodes 1 and 3 over their 200
/// unit run (the tangent, rounded to whole units so no platform's trig
/// reaches the position).
const SPOKES_CORNERS: [(usize, f64); 4] = [(20, 73.0), (30, 115.0), (40, 168.0), (50, 238.0)];

fn spokes_fixture(half_angle: usize) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("fixtures/routing/spokes-{half_angle}-4n-2m.json"))
}

/// Node 0 at the right with straight lines to nodes 1 and 3, which sit
/// `rise` above and below it 200 to its left, so its last move, to node 2
/// in between, leaves through the corner between its two spokes. The
/// lines are drawn straight rather than routed so the corner at node 0
/// has the angle the rise names.
fn spokes_position(rise: f64) -> GameState {
    let nodes = vec![
        p(700.0, 500.0),
        p(500.0, 500.0 - rise),
        p(250.0, 500.0),
        p(500.0, 500.0 + rise),
    ];
    let mut state = GameState::with_nodes_at(nodes, 1000, 0);
    state.current_player = Player::AI;
    for b in [1, 3] {
        let (from, to) = (state.nodes[0].position, state.nodes[b].position);
        let stroke = straight(from, to);
        let mid = stroke[5];
        state
            .apply_move(Move::new(0, b, stroke, mid, Player::AI, 1000))
            .unwrap();
        state.current_player = Player::AI;
    }
    state
}

/// Writes the spokes fixtures: `cargo test --release record_spokes_positions
/// -- --ignored`.
#[test]
#[ignore]
fn record_spokes_positions() {
    for (half_angle, rise) in SPOKES_CORNERS {
        std::fs::write(
            spokes_fixture(half_angle),
            sweep::fixture_json(&spokes_position(rise)),
        )
        .unwrap();
    }
}

/// The chord from node 0 to node 2 through the corner between node 0's
/// spokes in `state`, the spokes fixture with `rise`, and the test for a
/// point inside that corner: heading left, within the corner's half-angle
/// of the chord to node 2. The chord is told from the one round the back
/// by where its taut path leaves at the human clearances.
fn corner_chord(state: &GameState, rise: f64) -> (FaceMove, impl Fn(&Point) -> bool) {
    assert_eq!(spokes_at(state, 0).len(), 2);
    let end = state.nodes[0].position;
    let between =
        move |q: &Point| q.x < end.x && (q.y - end.y).abs() < (end.x - q.x) * rise / 200.0;
    let mut human = state.clone();
    human.current_player = Player::Human;
    let want = rules(&human);
    let mut moves = listed(state, 0, 2);
    moves.retain(|m| {
        plan(&mut Router::new(), &human, m, &want, Parity::Search)
            .is_ok_and(|(_, taut)| between(&taut[1]))
    });
    assert_eq!(moves.len(), 1, "one chord through the corner: {moves:?}");
    (moves.remove(0), between)
}

/// The chord through the corner between node 0's spokes draws down to
/// the corner the fit margin allows at the forgiven radius: the AI draws
/// it at 30 degrees and refuses it at 20 as Room (its floor is 22, where
/// the margin fits between the spokes 8 units out), and the human draws
/// every one (its floor is 17.5, the margin 10 units out). Each drawn
/// curve stays in the corner over the first node room and is a clearance
/// off both spokes at the end of it.
#[test]
fn a_chord_into_a_narrow_corner_draws_down_to_the_margin_floor() {
    let ai_draws = [false, true, true, true];
    for ((half_angle, rise), ai) in SPOKES_CORNERS.into_iter().zip(ai_draws) {
        let mut state = recorded_spokes_position(half_angle, rise);
        let (mv, between) = corner_chord(&state, rise);
        for (player, drawable) in [(Player::AI, ai), (Player::Human, true)] {
            state.current_player = player;
            let want = rules(&state);
            let verdict = route(&state, &mv);
            assert_eq!(
                verdict.is_ok(),
                drawable,
                "{half_angle} degrees as {player:?}: {verdict:?}"
            );
            if drawable {
                assert_curve_leaves_spokes(&state, &mv, &want, &between, true);
            } else {
                assert_eq!(verdict.unwrap_err(), Unroutable::Room);
            }
        }
    }
}

/// The 50 degree spokes fixture's chord through the corner, with the
/// recorded position, the rules and the test for a point in the corner.
fn spokes_case() -> (GameState, FaceMove, Rules, impl Fn(&Point) -> bool) {
    let (half_angle, rise) = SPOKES_CORNERS[3];
    let state = recorded_spokes_position(half_angle, rise);
    let want = rules(&state);
    let (mv, between) = corner_chord(&state, rise);
    (state, mv, want, between)
}

/// The spokes fixture for `half_angle`, checked against the position it
/// was recorded from.
fn recorded_spokes_position(half_angle: usize, rise: f64) -> GameState {
    let state = position(&spokes_fixture(half_angle));
    assert_eq!(
        bits_of(&spokes_position(rise)),
        bits_of(&state),
        "the {half_angle} degree fixture is the recorded position"
    );
    state
}

/// Least distance from `q` to a spoke of node 0.
fn spoke_gap(state: &GameState, q: &Point) -> f64 {
    spokes_at(state, 0)
        .iter()
        .map(|s| closest_point_on_polyline(s, q).1)
        .fold(f64::INFINITY, f64::min)
}

/// Relaxes `mv` and checks the interior point next to node 0 (the first
/// when `at_start`, else the last) moved from where the taut path put it,
/// stayed in the corner and sits the line clearance from both spokes.
fn assert_relaxed_point_leaves_spokes(
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    between: &dyn Fn(&Point) -> bool,
    at_start: bool,
) {
    let end = state.nodes[0].position;
    let (arena, taut) = plan(&mut Router::new(), state, mv, want, Parity::Search).unwrap();
    let relaxed = relax(state, mv, want, &arena, &taut).unwrap();
    let before = clearance::resample(&taut, clearance::SPACING * super::eroded_by(state, want));
    let n = relaxed.len();
    let (end_at, i) = if at_start { (0, 1) } else { (n - 1, n - 2) };
    assert_eq!(relaxed[end_at], end);
    assert!(
        before[i].distance_to(&relaxed[i]) > 1e-9,
        "the point next to the end did not move"
    );
    assert!(
        between(&relaxed[i]),
        "the point next to the end left the corner: {:?}",
        relaxed[i]
    );
    assert!(
        spoke_gap(state, &relaxed[i]) >= want.clearance,
        "point next to the end {:.2} from a spoke, clearance {:.2}",
        spoke_gap(state, &relaxed[i]),
        want.clearance
    );
}

/// Checks the drawn curve for `mv` stays in node 0's corner over the node
/// room nearest it (its first when `at_start`, else its last) and is a
/// clearance off both spokes one node room out.
fn assert_curve_leaves_spokes(
    state: &GameState,
    mv: &FaceMove,
    want: &Rules,
    between: &dyn Fn(&Point) -> bool,
    at_start: bool,
) {
    let drawn = route(state, mv).unwrap();
    let mut flat = drawn.flatten(curve::tolerance(state.board_size));
    if !at_start {
        flat.reverse();
    }
    assert_eq!(flat[0], state.nodes[0].position);
    let room = NODE_ROOM * want.spacing;
    let mut along = 0.0;
    let mut checked = 0;
    let mut last = flat[0];
    // The curve sampled about every unit of arc length over the first
    // node room, so a straight route still gives points to check.
    'curve: for w in flat.windows(2) {
        let length = w[0].distance_to(&w[1]);
        let steps = length.ceil().max(1.0) as usize;
        for k in 1..=steps {
            let t = k as f64 / steps as f64;
            last = p(
                w[0].x + (w[1].x - w[0].x) * t,
                w[0].y + (w[1].y - w[0].y) * t,
            );
            if along + length * t > room {
                break 'curve;
            }
            assert!(between(&last), "{last:?} left the corner");
            checked += 1;
        }
        along += length;
    }
    assert!(checked >= 20, "{checked} points checked");
    assert!(
        spoke_gap(state, &last) >= want.clearance,
        "{:.2} from a spoke one node room out",
        spoke_gap(state, &last)
    );
}

/// A route out of an end node with two spokes leaves between them instead
/// of hugging one: the relaxation moves the first point after the end to
/// the line clearance from both spokes, and the drawn curve stays in the
/// corner, a clearance off both spokes by the time it is a node room out.
#[test]
fn a_route_out_of_an_end_with_two_spokes_leaves_between_them() {
    let (state, mv, want, between) = spokes_case();
    assert_relaxed_point_leaves_spokes(&state, &mv, &want, &between, true);
    assert_curve_leaves_spokes(&state, &mv, &want, &between, true);
}

/// The same move routed the other way, into the spoked node: the last
/// point before the end moves and the curve arrives between the spokes.
#[test]
fn a_route_into_an_end_with_two_spokes_arrives_between_them() {
    let (state, mv, want, between) = spokes_case();
    let reversed = FaceMove {
        a: mv.b,
        at_a: mv.at_b,
        b: mv.a,
        at_b: mv.at_a,
        enclosed: mv.enclosed.clone(),
    };
    assert_relaxed_point_leaves_spokes(&state, &reversed, &want, &between, false);
    assert_curve_leaves_spokes(&state, &reversed, &want, &between, false);
}

/// Every coordinate of a position, for comparing a fixture with the
/// position it was recorded from.
fn bits_of(state: &GameState) -> Vec<u64> {
    crate::settle::golden::bits(state)
}

/// Records forty picks of each shared game and prints how many AI moves
/// of each classify as topology, the first thing
/// tests/e2e/replay-links.spec.ts needs of a recording:
/// `cargo test --release probe_shared_game_picks -- --ignored --nocapture`.
#[test]
#[ignore]
fn probe_shared_game_picks() {
    for (nodes, moves) in [(4, 8), (9, 10)] {
        let (mut yes, mut of, mut games) = (0, 0, 0);
        for pick in 0..40 {
            match shared_game(nodes, moves, pick) {
                Ok(game) => {
                    let topology = game.topology.iter().flatten().count();
                    eprintln!(
                        "{nodes} nodes pick {pick}: {topology} of {} AI moves redraw",
                        moves / 2
                    );
                    yes += topology;
                    of += moves / 2;
                    games += 1;
                }
                Err(why) => eprintln!("{nodes} nodes pick {pick}: {why}"),
            }
        }
        eprintln!("{nodes} nodes: {yes} of {of} AI moves redraw over {games} games");
    }
}

/// The far runs of `path` for ends at `ends` and radius 8: every point
/// of a run lies outside the radius, and every cut (a run end that is
/// not an end of the path) lies on it within 1e-6.
fn far_runs(path: &[Point], ends: &[Point]) -> Vec<Vec<Point>> {
    let runs = erode::beyond(path, ends, 8.0);
    let nearest = |q: &Point| {
        ends.iter()
            .map(|e| e.distance_to(q))
            .fold(f64::INFINITY, f64::min)
    };
    for run in &runs {
        assert!(run.len() >= 2, "a run has two points: {run:?}");
        for q in run {
            assert!(nearest(q) >= 8.0 - 1e-6, "{q:?} lies inside the radius");
        }
        for cut in [run[0], run[run.len() - 1]] {
            if Some(&cut) != path.first() && Some(&cut) != path.last() {
                assert!(
                    (nearest(&cut) - 8.0).abs() < 1e-6,
                    "cut {cut:?} is {} from the radius",
                    nearest(&cut) - 8.0
                );
            }
        }
    }
    runs
}

/// A straight spoke between the move's two ends has no far part when
/// the two forgiven discs cover it, and a 17 unit one keeps the unit
/// between them, cut on the segment since both vertices lie inside.
#[test]
fn a_spoke_between_the_ends_is_cut_on_its_segment() {
    let a = p(100.0, 100.0);
    assert!(far_runs(&[a, p(115.0, 100.0)], &[a, p(115.0, 100.0)]).is_empty());
    let b = p(117.0, 100.0);
    assert_eq!(
        far_runs(&[a, b], &[a, b]),
        vec![vec![p(108.0, 100.0), p(109.0, 100.0)]]
    );
}

/// A loop back to its own node is cut at both ends by the one owner.
#[test]
fn a_loop_spoke_is_cut_where_it_leaves_and_where_it_returns() {
    let a = p(100.0, 100.0);
    let path = [a, p(120.0, 100.0), p(120.0, 120.0), p(100.0, 120.0), a];
    let runs = far_runs(&path, &[a]);
    assert_eq!(runs.len(), 1);
    let run = &runs[0];
    assert_eq!(run.len(), 5);
    assert!(run[0].distance_to(&p(108.0, 100.0)) < 1e-6);
    assert_eq!(&run[1..4], &path[1..4]);
    assert!(run[4].distance_to(&p(100.0, 108.0)) < 1e-6);
}

/// A spoke that leaves the radius, bends back inside it and leaves again
/// gives two runs, so the stretch inside carries no margin, as the
/// validator forgives a crossing there.
#[test]
fn a_spoke_that_comes_back_inside_gives_two_far_runs() {
    let a = p(100.0, 100.0);
    let path = [
        a,
        p(120.0, 100.0),
        p(120.0, 106.0),
        p(102.0, 106.0),
        p(102.0, 130.0),
    ];
    let runs = far_runs(&path, &[a]);
    assert_eq!(runs.len(), 2);
    assert!(runs[0][0].distance_to(&p(108.0, 100.0)) < 1e-6);
    assert_eq!(&runs[0][1..3], &path[1..3]);
    // The y = 106 segment meets the radius where x - 100 = sqrt(64 - 36),
    // the x = 102 segment where y - 100 = sqrt(64 - 4).
    let back_in = runs[0].last().unwrap();
    assert!(
        back_in.distance_to(&p(100.0 + 28f64.sqrt(), 106.0)) < 1e-6,
        "{back_in:?}"
    );
    let out_again = runs[1][0];
    assert!(
        out_again.distance_to(&p(102.0, 100.0 + 60f64.sqrt())) < 1e-6,
        "{out_again:?}"
    );
    assert_eq!(runs[1][1], path[4]);
}

/// A spoke to another node is cut once, at its own end, and keeps that
/// node's end.
#[test]
fn a_spoke_to_another_node_keeps_its_far_end() {
    let a = p(100.0, 100.0);
    let path = [a, p(150.0, 100.0), p(200.0, 140.0)];
    let runs = far_runs(&path, &[a]);
    assert_eq!(runs.len(), 1);
    assert!(runs[0][0].distance_to(&p(108.0, 100.0)) < 1e-6);
    assert_eq!(&runs[0][1..], &path[1..]);
}

/// A chord drawn alongside a straight line between its own two ends: the
/// shared spoke is a two point polyline with both vertices inside the
/// ends' forgiven radius, and a cut that looked at vertices alone left it
/// with no far part, so the path ran along it and the read-back refused
/// the heading as PathAlongLine.
#[test]
fn a_chord_beside_a_straight_line_between_its_ends_leaves_the_line() {
    let state = position(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures/routing/corridor-4n-2m.json"),
    );
    let mv = FaceMove {
        a: 0,
        at_a: Some(4),
        b: 5,
        at_b: Some(6),
        enclosed: Vec::new(),
    };
    let line = crate::embedding::half_edge_path(&state, 4);
    assert_eq!(line.len(), 2, "the spoke between the ends is straight");
    let drawn = route(&state, &mv).unwrap();
    let flat = drawn.flatten(curve::tolerance(state.board_size));
    let (far_from_line, total) = flat.iter().fold((0usize, 0usize), |(far, n), q| {
        let gap = closest_point_on_polyline(&line, q).1;
        (far + usize::from(gap > curve::tolerance(1000)), n + 1)
    });
    assert!(
        far_from_line * 2 > total,
        "{far_from_line} of {total} points off the line"
    );
}

/// How long indexing a game takes natively, which boot pays when a link
/// is in the URL: `cargo test --release probe_index_time -- --ignored
/// --nocapture`.
#[test]
#[ignore]
fn probe_index_time() {
    for (nodes, moves, pick) in [(12, 30, 1), (24, 10, 0), (24, 20, 0)] {
        let game = match shared_game(nodes, moves, pick) {
            Ok(game) => game,
            Err(why) => {
                eprintln!("PROBE {nodes}n {moves}m: {why}");
                continue;
            }
        };
        let plan = || protocol::Classification {
            initial_nodes: game.fixture.initial_nodes,
            board_size: game.fixture.board_size,
            first_player: Player::Human,
            nodes: None,
            settle_version: game.fixture.settle_version,
            moves: game.fixture.moves.iter().map(|m| m.wire()).collect(),
            known: vec![None; game.fixture.moves.len()],
        };
        let start = std::time::Instant::now();
        let outcome = protocol::index_moves(plan());
        eprintln!(
            "PROBE {nodes}n {moves}m: index_moves {:?} in {} ms",
            outcome
                .as_ref()
                .map(|i| i.moves.len())
                .map_err(|e| e.clone()),
            start.elapsed().as_millis()
        );
    }
}
