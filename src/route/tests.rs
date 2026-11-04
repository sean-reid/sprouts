use super::sweep::{self, draws};
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
    let other = if around_one.enclosed == vec![1] { 2 } else { 1 };
    let loop_move = route(&state, &around_one).unwrap();
    state.apply_move(loop_move).unwrap();
    state.current_player = Player::AI;
    let loop_flat = state.lines[0].flat.clone();
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

/// Routes tried over every case of the property test so far, by outcome.
static TALLY: std::sync::Mutex<sweep::Tally> = std::sync::Mutex::new(sweep::Tally {
    tried: 0,
    room: 0,
    no_spot: 0,
    no_leg: 0,
    shape: std::collections::BTreeMap::new(),
    topology: 0,
    bug: 0,
    bugs: std::collections::BTreeMap::new(),
});

/// The same over the human-rules games.
static HUMAN_TALLY: std::sync::Mutex<sweep::Tally> = std::sync::Mutex::new(sweep::Tally {
    tried: 0,
    room: 0,
    no_spot: 0,
    no_leg: 0,
    shape: std::collections::BTreeMap::new(),
    topology: 0,
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
/// Most read-back mismatches the seeded sweep and the recorded topology
/// positions may show; the release sweep reads 3.
const TOPOLOGY_CEILING: usize = 8;

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
        prop_assert!(total.topology <= TOPOLOGY_CEILING, "{}", total);
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

/// Writes a game of `moves` moves on `nodes` nodes to `path` in the saved
/// record shape, the human moving first: human lines are routed at twice
/// the human clearances, so the rounded strokes of a link still pass, and
/// then kinked by more than the redraw tolerance, so they travel as
/// geometry; AI lines are the first search move the router draws.
fn record_shared_game(nodes: usize, moves: usize, pick: usize, path: &std::path::Path) {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
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
        let mov = if state.current_player == Player::Human {
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
                    let routed = route_at(&state, face, &slack).ok()?;
                    // One kink in the middle, so the thinned stroke stays
                    // short and still differs from the router's line.
                    let n = routed.polyline.len();
                    let bent: Vec<Point> = routed
                        .polyline
                        .iter()
                        .enumerate()
                        .map(|(i, q)| {
                            let t = 1.0 - (2.0 * i as f64 / (n - 1) as f64 - 1.0).abs();
                            Point::new(q.x + kink * t, q.y + kink * t)
                        })
                        .collect();
                    let mov = Move::new(
                        face.a,
                        face.b,
                        bent,
                        routed.new_node_pos,
                        Player::Human,
                        state.board_size,
                    );
                    protocol::check(&state, &mov).ok()?;
                    let drawn = crate::embedding::embed(&state, &mov).ok()?;
                    (drawn.enclosed.len() == face.enclosed.len()).then_some(mov)
                })
                .unwrap_or_else(|| panic!("no human move at move {index}"))
        } else {
            // The first search move that draws, so the record is the same
            // on every run; the ranked choice depends on the wall clock.
            let moves = state.face.search_moves();
            let order: Vec<usize> = (0..moves.len()).collect();
            match crate::ai::draw(&state, &moves, &order) {
                (Some(mov), _) => mov,
                (None, _) => panic!("no AI move draws at move {index}"),
            }
        };
        record.push(serde_json::json!({
            "from": mov.from_node,
            "to": mov.to_node,
            "polyline": mov.polyline.iter().map(|q| [q.x, q.y]).collect::<Vec<_>>(),
            "newNode": [mov.new_node_pos.x, mov.new_node_pos.y],
        }));
        state.apply_move(mov).unwrap();
    }
    let nodes: Vec<[f64; 2]> = state.nodes[..state.initial_node_count]
        .iter()
        .map(|n| [n.position.x, n.position.y])
        .collect();
    let json = serde_json::json!({
        "version": 1,
        "initialNodes": state.initial_node_count,
        "boardSize": state.board_size,
        "nodes": nodes,
        "moves": record,
        "firstPlayer": "human",
        "cursor": moves,
    });
    std::fs::write(path, json.to_string()).unwrap();
}

/// Rewrites the two shared-game fixtures that tests/e2e/replay-links.spec.ts
/// carries as topology, for when the router's output changes:
/// `cargo test --release record_shared_games -- --ignored`. The pick,
/// from SPROUTS_PICK_4 and SPROUTS_PICK_9 when set, is which chord the
/// human opens with; the spec needs games whose AI moves the visitor's
/// rounded board reroutes within a unit, which most picks do not give, so
/// a new router may need another pick. The record is deterministic.
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
        pick("SPROUTS_PICK_4", 11),
        &dir.join("eight-moves-4n-1000.json"),
    );
    record_shared_game(
        9,
        10,
        pick("SPROUTS_PICK_9", 11),
        &dir.join("ten-moves-9n-1000.json"),
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

/// Read-back mismatches the four recorded positions showed when recorded,
/// every one a chord from a node to the node on its own spoke through the
/// corner beside that spoke.
const RECORDED_TOPOLOGY: usize = 9;

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

/// Hashes of each routing stage over every search move of the first
/// `ROUTING_HASH_FIXTURES` routing fixtures, in path order: arena
/// vertices, taut path, relaxed path, fitted cubics, and the flattening
/// with the node. A platform that disagrees shows where it diverged.
fn routing_hashes() -> [u64; 5] {
    let mut stages: [Fnv; 5] = Default::default();
    for path in fixture_paths("fixtures/routing")
        .into_iter()
        .take(ROUTING_HASH_FIXTURES)
    {
        let state = position(&path);
        let want = rules(&state);
        for mv in state.face.search_moves() {
            if mv.a != mv.b || !mv.enclosed.is_empty() {
                if let Ok((arena, taut)) = plan(&state, &mv, &want) {
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

/// Routing fixtures the hash covers; five keep the debug run short.
const ROUTING_HASH_FIXTURES: usize = 5;

/// The final routing hash a Mac records; the Linux runner and the wasm
/// build (tests/unit/route-hash.test.ts) have to produce the same one,
/// since a share link is redrawn on the visitor's device.
const ROUTING_HASH: u64 = 0x1ddc_ba1e_f5d3_5c8c;

#[test]
fn routing_hashes_the_same_on_every_platform() {
    let [arena, taut, relaxed, cubics, flat] = routing_hashes();
    assert_eq!(
        flat, ROUTING_HASH,
        "routing hashes: arena {arena:#018x} taut {taut:#018x} relaxed {relaxed:#018x} cubics {cubics:#018x} flat {flat:#018x}"
    );
}
