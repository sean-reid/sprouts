//! Replays recorded games natively and times each engine phase per move.

use crate::ai;
#[cfg(feature = "bench")]
pub use crate::ai::nimber_stats;
use crate::protocol;
#[cfg(feature = "bench")]
pub use crate::route::sweep::{sweep, Tally};
use crate::settle;
pub use crate::types::Player;
use crate::types::{opening_layout, GameState, Move, Point};
use crate::validation::Rejection;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::hint::black_box;
use std::path::Path;
use std::time::{Duration, Instant};

pub const FIXTURE_VERSION: u32 = 1;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct Fixture {
    pub version: u32,
    pub initial_nodes: usize,
    pub board_size: usize,
    /// Where the starting nodes were; the default layout when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nodes: Option<Vec<[f64; 2]>>,
    /// The settle the game was played under; absent means none ran.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub settle_version: u8,
    pub moves: Vec<FixtureMove>,
}

fn is_zero(version: &u8) -> bool {
    *version == 0
}

impl Fixture {
    /// The board before the first move; `parse` has checked the positions
    /// match the count.
    pub fn start(&self) -> GameState {
        protocol::start(
            self.initial_nodes,
            self.board_size,
            Player::Human,
            self.nodes
                .as_ref()
                .map(|nodes| nodes.iter().map(|p| Point::new(p[0], p[1])).collect()),
            self.settle_version,
        )
        .expect("a parsed fixture's positions match its count")
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct FixtureMove {
    pub from: usize,
    pub to: usize,
    pub polyline: Vec<[f64; 2]>,
    pub new_node: [f64; 2],
}

impl FixtureMove {
    fn from_move(mov: &Move) -> Self {
        Self {
            from: mov.from_node,
            to: mov.to_node,
            polyline: mov.polyline.iter().map(|p| [p.x, p.y]).collect(),
            new_node: [mov.new_node_pos.x, mov.new_node_pos.y],
        }
    }

    fn to_move(&self, player: Player, board_size: usize) -> Move {
        Move::new(
            self.from,
            self.to,
            self.polyline
                .iter()
                .map(|p| Point::new(p[0], p[1]))
                .collect(),
            Point::new(self.new_node[0], self.new_node[1]),
            player,
            board_size,
        )
    }
}

pub const PHASE_COUNT: usize = 6;

pub const PHASES: [&str; PHASE_COUNT] = [
    "face_update",
    "enumeration",
    "routing",
    "search",
    "validation",
    "settle",
];

#[derive(Debug, Clone, Copy, Default)]
pub struct Phases {
    /// Applying the recorded move to the position, face model included.
    pub face_update: Duration,
    pub enumeration: Duration,
    /// Drawing the move the search picked, with any fallbacks.
    pub routing: Duration,
    pub search: Duration,
    pub validation: Duration,
    /// Relaxing the board around the recorded move once it is in.
    pub settle: Duration,
}

impl Phases {
    pub fn as_array(&self) -> [Duration; PHASE_COUNT] {
        [
            self.face_update,
            self.enumeration,
            self.routing,
            self.search,
            self.validation,
            self.settle,
        ]
    }

    pub fn total(&self) -> Duration {
        self.as_array().iter().sum()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MoveTiming {
    pub index: usize,
    pub player: Player,
    /// The position after the last move, where no move is validated.
    pub terminal: bool,
    /// What the face model says of the position.
    pub game_over: bool,
    /// Moves the search preferred that could not be drawn here.
    pub routing_failures: usize,
    /// No listed move could be drawn, though the position had some.
    pub undrawn: bool,
    /// Least clearance on the board as a share of its target, after the
    /// recorded move went in and again once the board settled.
    pub clearance: Option<(f64, f64)>,
    pub phases: Phases,
}

#[derive(Debug, Clone)]
pub struct Replay {
    pub timings: Vec<MoveTiming>,
    pub failure: Option<String>,
}

impl Replay {
    pub fn routing_failures(&self) -> usize {
        self.timings.iter().map(|t| t.routing_failures).sum()
    }

    /// Positions where the AI had moves and could draw none of them.
    pub fn undrawn(&self) -> usize {
        self.timings.iter().filter(|t| t.undrawn).count()
    }
}

pub fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

/// Slowest run of each phase across the replay, in milliseconds.
pub fn max_per_phase(timings: &[MoveTiming]) -> [f64; PHASE_COUNT] {
    let mut max = [0.0f64; PHASE_COUNT];
    for t in timings {
        for (slot, d) in max.iter_mut().zip(t.phases.as_array()) {
            *slot = slot.max(ms(d));
        }
    }
    max
}

/// One line per phase whose max is over its budget.
pub fn breaches(
    name: &str,
    max: &[f64; PHASE_COUNT],
    budgets: &BTreeMap<String, f64>,
) -> Vec<String> {
    PHASES
        .iter()
        .zip(max)
        .filter_map(|(phase, v)| {
            let budget = budgets.get(*phase)?;
            (v > budget).then(|| format!("{name}: {phase} {v:.2} ms over budget {budget:.2} ms"))
        })
        .collect()
}

pub fn parse(json: &str) -> Result<Fixture, String> {
    let fixture: Fixture = serde_json::from_str(json).map_err(|e| e.to_string())?;
    if fixture.version != FIXTURE_VERSION {
        return Err(format!(
            "fixture version {} is not {}",
            fixture.version, FIXTURE_VERSION
        ));
    }
    if let Some(nodes) = &fixture.nodes {
        if nodes.len() != fixture.initial_nodes {
            return Err(format!(
                "{} starting positions for {} nodes",
                nodes.len(),
                fixture.initial_nodes
            ));
        }
    }
    Ok(fixture)
}

pub fn load(path: &Path) -> Result<Fixture, String> {
    let json = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path.display(), e))?;
    parse(&json).map_err(|e| format!("{}: {}", path.display(), e))
}

pub fn write(fixture: &Fixture, path: &Path) -> Result<(), String> {
    let json = serde_json::to_string(fixture).map_err(|e| e.to_string())?;
    std::fs::write(path, json).map_err(|e| format!("{}: {}", path.display(), e))
}

fn validate_for(state: &GameState, mov: &Move) -> Result<(), Rejection> {
    protocol::check(state, mov)
}

/// The board after every recorded move, each checked as the page would.
#[cfg(test)]
pub(crate) fn play(fixture: &Fixture) -> GameState {
    play_as(fixture, Player::Human)
}

/// `play` with `first` making the first move.
#[cfg(test)]
pub(crate) fn play_as(fixture: &Fixture, first: Player) -> GameState {
    let nodes = fixture
        .nodes
        .as_ref()
        .map(|nodes| nodes.iter().map(|p| Point::new(p[0], p[1])).collect());
    let mut state =
        protocol::start(fixture.initial_nodes, fixture.board_size, first, nodes).unwrap();
    for (index, recorded) in fixture.moves.iter().enumerate() {
        let mov = recorded.to_move(state.current_player, state.board_size);
        if let Err(e) = validate_for(&state, &mov) {
            panic!("move {index} rejected: {}", e.message);
        }
        state.apply_move(mov).unwrap();
    }
    state
}

/// Runs every phase of a turn at each position, including the one after the
/// last move, and applies each recorded move in turn.
pub fn replay(fixture: &Fixture) -> Replay {
    let mut state = fixture.start();
    let mut timings = Vec::with_capacity(fixture.moves.len() + 1);
    let mut failure = None;

    for index in 0..=fixture.moves.len() {
        let mut phases = Phases::default();

        let t = Instant::now();
        let candidates = state.face.search_moves();
        phases.enumeration = t.elapsed();
        let game_over = candidates.is_empty();

        let t = Instant::now();
        let order = ai::rank(&state, &candidates);
        phases.search = t.elapsed();

        let t = Instant::now();
        let (routing_failures, undrawn, out_of_time) = if candidates.is_empty() {
            (0, false, false)
        } else {
            let (drawn, failures) = ai::draw(&state, &candidates, &order);
            let drawn = black_box(drawn);
            (
                failures,
                drawn.is_err(),
                matches!(drawn, Err(ai::Undrawn::OutOfTime)),
            )
        };
        phases.routing = t.elapsed();
        if out_of_time {
            failure = Some(format!("move {index}: the draw ran out of time"));
            break;
        }

        let recorded = fixture.moves.get(index);
        let verdict = recorded.map(|recorded| {
            let t = Instant::now();
            let mov = recorded.to_move(state.current_player, state.board_size);
            let verdict = validate_for(&state, &mov);
            phases.validation = t.elapsed();
            (mov, verdict)
        });

        let mut clearance = None;
        let applied = match verdict {
            Some((mov, Ok(()))) => {
                let t = Instant::now();
                let applied = state.apply_move(mov);
                phases.face_update = t.elapsed();
                if applied.is_ok() {
                    let (before, _) = settle::least_clearance(&state);
                    let mut settled = state.clone();
                    settled.settle_version = protocol::SETTLE_VERSION;
                    let t = Instant::now();
                    settle::settle(&mut settled);
                    phases.settle = t.elapsed();
                    clearance = Some((before, settle::least_clearance(&settled).0));
                    if state.settle_version > 0 {
                        state = settled;
                    }
                }
                applied.map_err(|e| format!("move {index} rejected: {}", e.message))
            }
            Some((_, Err(e))) => Err(format!("move {index} rejected: {}", e.message)),
            None => Ok(()),
        };

        timings.push(MoveTiming {
            index,
            player: if recorded.is_some() {
                state
                    .move_history
                    .last()
                    .map_or(state.current_player, |m| m.player)
            } else {
                state.current_player
            },
            terminal: recorded.is_none(),
            game_over,
            routing_failures,
            undrawn,
            clearance,
            phases,
        });

        if let Err(e) = applied {
            failure = Some(e);
            break;
        }
    }

    Replay { timings, failure }
}

/// Positions the search charged, summed over the fixture's positions with a
/// choice to make, when deepening to each depth from 1 to `max_depth`
/// without a time limit.
#[cfg(feature = "bench")]
pub fn nodes_per_depth(fixture: &Fixture, max_depth: usize) -> Vec<usize> {
    let mut state = fixture.start();
    let mut totals = vec![0; max_depth];
    for recorded in &fixture.moves {
        let candidates = state.face.search_moves();
        if candidates.len() > 1 {
            for (slot, depth) in totals.iter_mut().zip(1..) {
                *slot += ai::nodes_to_depth(&state, &candidates, depth);
            }
        }
        let mov = recorded.to_move(state.current_player, state.board_size);
        if state.apply_move(mov).is_err() {
            break;
        }
    }
    totals
}

/// Applies each recorded move without the AI's phases and settles a copy
/// of each position under the current settle; returns the settle time per
/// move in milliseconds, for games too large for the full replay.
pub fn settle_times(fixture: &Fixture) -> Vec<f64> {
    let mut state = fixture.start();
    let mut times = Vec::with_capacity(fixture.moves.len());
    for recorded in &fixture.moves {
        let mov = recorded.to_move(state.current_player, state.board_size);
        if state.apply_move(mov).is_err() {
            break;
        }
        let mut settled = state.clone();
        settled.settle_version = protocol::SETTLE_VERSION;
        let t = Instant::now();
        settle::settle(&mut settled);
        times.push(ms(t.elapsed()));
        if state.settle_version > 0 {
            state = settled;
        }
    }
    times
}

/// Why a self-play game stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stop {
    GameOver,
    /// No listed move could be drawn.
    Undrawn,
    /// The drawn move failed the validation the page applies to its side.
    Rejected,
}

/// Plays the AI against itself, holding each side to the validation the page
/// applies to that side.
pub fn record(initial_nodes: usize, board_size: usize) -> Fixture {
    let (moves, _) = self_play(initial_nodes, board_size, true);
    Fixture {
        version: FIXTURE_VERSION,
        initial_nodes,
        board_size,
        settle_version: protocol::SETTLE_VERSION,
        nodes: Some(
            opening_layout(initial_nodes, board_size)
                .iter()
                .map(|p| [p.x, p.y])
                .collect(),
        ),
        moves,
    }
}

/// The AI against itself, settling the board after each move or not, with
/// the moves it made and why it stopped.
pub fn self_play(
    initial_nodes: usize,
    board_size: usize,
    settled: bool,
) -> (Vec<FixtureMove>, Stop) {
    let mut state = GameState::with_board_size(initial_nodes, board_size);
    if !settled {
        state.settle_version = 0;
    }
    let mut moves = Vec::new();
    loop {
        let mov = match next_accepted_move(&mut state) {
            Ok(mov) => mov,
            Err(stop) => return (moves, stop),
        };
        moves.push(FixtureMove::from_move(&mov));
        state.apply_move(mov).expect("an accepted move applies");
        settle::settle(&mut state);
    }
}

/// The AI's move for the position, or why there is none: the game is
/// over, the AI could not draw a move, or its move fails the validation
/// the page would apply. The last two are reported on stderr since they
/// end a recording early. A draw that stops at its time limit is an error,
/// not a position to record.
fn next_accepted_move(state: &mut GameState) -> Result<Move, Stop> {
    let mov = match ai::choose(state) {
        ai::Reply::Move(mov) => mov,
        ai::Reply::GameOver => return Err(Stop::GameOver),
        ai::Reply::Undrawn(ai::Undrawn::Exhausted) => {
            eprintln!(
                "recording stopped after {} moves: every listed move was refused",
                state.move_history.len()
            );
            return Err(Stop::Undrawn);
        }
        ai::Reply::Undrawn(ai::Undrawn::OutOfTime) => panic!(
            "the draw ran out of time after {} moves; the recording would differ between runs",
            state.move_history.len()
        ),
    };
    match validate_for(state, &mov) {
        Ok(()) => Ok(mov),
        Err(e) => {
            eprintln!(
                "recording stopped after {} moves: the AI's move was rejected: {}",
                state.move_history.len(),
                e.message
            );
            Err(Stop::Rejected)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures_dir() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/games")
    }

    #[test]
    fn human_export_replays_every_move() {
        let fixture = load(&fixtures_dir().join("human-4n-1000.json")).unwrap();
        assert_eq!(fixture.initial_nodes, 4);
        assert_eq!(fixture.board_size, 1000);
        let run = replay(&fixture);
        assert_eq!(run.failure, None);
        assert_eq!(run.timings.len(), fixture.moves.len() + 1);
        assert_eq!(run.timings[0].player, Player::Human);
        assert_eq!(run.timings[1].player, Player::AI);
        assert!(run.timings[2].terminal);
        let max = max_per_phase(&run.timings);
        assert!(max.iter().all(|v| *v > 0.0), "{max:?}");
        assert_eq!(run.undrawn(), 0);
    }

    #[test]
    fn saved_game_from_the_page_loads_as_a_fixture() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/saved/first-exchange.json");
        let raw: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        assert_eq!(raw["firstPlayer"], "human");
        assert_eq!(raw["cursor"], 2);
        let fixture = load(&path).unwrap();
        assert_eq!(fixture.moves.len(), 2);
        let run = replay(&fixture);
        assert_eq!(run.failure, None);
        assert!(run.timings[2].terminal);
    }

    /// The board after every recorded move, each checked as the page would.
    fn play(fixture: &Fixture) -> GameState {
        let mut state = fixture.start();
        for (index, recorded) in fixture.moves.iter().enumerate() {
            let mov = recorded.to_move(state.current_player, state.board_size);
            if let Err(e) = validate_for(&state, &mov) {
                panic!("move {index} rejected: {}", e.message);
            }
            state.apply_move(mov).unwrap();
            settle::settle(&mut state);
        }
        state
    }

    /// Recorded on the live site on 2026-11-03, where the page declared the
    /// human the winner with nodes 8 and 12 still joinable.
    #[test]
    fn a_false_game_over_from_the_site_is_still_live() {
        let fixture = load(&fixtures_dir().join("false-game-over-4n-1000.json")).unwrap();
        assert_eq!(fixture.moves.len(), 9);
        let run = replay(&fixture);
        assert_eq!(run.failure, None);
        let last = run.timings.last().unwrap();
        assert!(last.terminal);
        assert!(!last.game_over);
        assert!(!run.timings[0].game_over);

        let state = play(&fixture);
        assert_eq!(state.face.degree(8), 2);
        assert_eq!(state.face.degree(12), 2);
        let moves = state.face.search_moves();
        assert_eq!(moves.len(), 1);
        assert_eq!((moves[0].a, moves[0].b), (8, 12));
        assert!(moves[0].enclosed.is_empty());
    }

    /// The one move left runs between nodes 8 and 12 inside the spiral; the
    /// AI has to draw it.
    #[test]
    fn the_ai_plays_the_last_move_of_the_false_game_over() {
        let fixture = load(&fixtures_dir().join("false-game-over-4n-1000.json")).unwrap();
        let mut state = play(&fixture);
        assert!(!state.face.is_game_over());
        assert_eq!(state.current_player, Player::AI);
        let moves = state.face.search_moves();
        let order = ai::rank(&state, &moves);
        let (Ok(mov), failures) = ai::draw(&state, &moves, &order) else {
            panic!("the AI draws the last move");
        };
        assert_eq!(failures, 0);
        assert_eq!(
            (mov.from_node, mov.to_node),
            (8, 12),
            "{}",
            serde_json::to_string(&FixtureMove::from_move(&mov)).unwrap()
        );
        assert_eq!(validate_for(&state, &mov), Ok(()));
        state.apply_move(mov).unwrap();
        assert!(state.face.is_game_over());
        let run = replay(&fixture);
        assert_eq!(run.failure, None);
        assert_eq!(run.undrawn(), 0);
    }

    #[test]
    fn exported_game_replays() {
        let Ok(path) = std::env::var("SPROUTS_FIXTURE") else {
            return;
        };
        let fixture = load(Path::new(&path)).unwrap();
        assert!(fixture.moves.len() >= 2);
        let run = replay(&fixture);
        assert_eq!(run.failure, None);
    }

    #[test]
    fn rejection_keeps_the_timings_so_far() {
        let mut fixture = load(&fixtures_dir().join("human-4n-1000.json")).unwrap();
        let start = fixture.moves[1].polyline[0];
        fixture.moves[1].polyline = vec![start, start];
        let run = replay(&fixture);
        assert!(run
            .failure
            .as_deref()
            .unwrap()
            .starts_with("move 1 rejected"));
        assert_eq!(run.timings.len(), 2);
        assert!(!run.timings[1].terminal);
    }

    #[test]
    fn self_play_records_a_game_that_replays() {
        let fixture = record(4, 500);
        assert!(fixture.moves.len() >= 2);
        let run = replay(&fixture);
        assert_eq!(run.failure, None);
        assert_eq!(run.timings.len(), fixture.moves.len() + 1);
    }

    #[test]
    fn breaches_name_only_phases_over_budget() {
        let budgets: BTreeMap<String, f64> = PHASES.iter().map(|p| (p.to_string(), 10.0)).collect();
        let max = [5.0, 10.0, 10.5, 0.0, 200.0, 1.0];
        let hits = breaches("g", &max, &budgets);
        assert_eq!(hits.len(), 2);
        assert!(hits[0].starts_with("g: routing 10.50 ms over budget 10.00 ms"));
        assert!(hits[1].starts_with("g: validation 200.00 ms"));
    }

    #[test]
    fn a_fixture_without_positions_starts_on_the_default_layout() {
        let bare = parse(r#"{"version":1,"initialNodes":3,"boardSize":500,"moves":[]}"#).unwrap();
        assert_eq!(bare.nodes, None);
        let state = bare.start();
        let expected = opening_layout(3, 500);
        assert_eq!(state.nodes.len(), 3);
        assert_eq!(state.nodes[2].position, expected[2]);
        assert!(!serde_json::to_string(&bare).unwrap().contains("nodes"));
    }

    #[test]
    fn recorded_positions_place_the_starting_nodes() {
        let fixture = load(&fixtures_dir().join("human-4n-1000.json")).unwrap();
        let nodes = fixture.nodes.as_ref().unwrap();
        assert_eq!(nodes.len(), 4);
        let state = fixture.start();
        assert_eq!(
            state.nodes[0].position,
            Point::new(nodes[0][0], nodes[0][1])
        );
        assert_ne!(state.nodes[0].position, opening_layout(4, 1000)[0]);
        let recorded = record(4, 500);
        assert_eq!(recorded.nodes.unwrap().len(), 4);
    }

    #[test]
    fn rejects_positions_that_do_not_match_the_count() {
        let json =
            r#"{"version":1,"initialNodes":3,"boardSize":500,"nodes":[[1,2],[3,4]],"moves":[]}"#;
        assert!(parse(json)
            .unwrap_err()
            .contains("2 starting positions for 3 nodes"));
    }

    #[test]
    fn rejects_other_versions() {
        let json = r#"{"version":2,"initialNodes":4,"boardSize":1000,"moves":[]}"#;
        assert!(parse(json).unwrap_err().contains("version 2"));
    }

    #[test]
    fn fixture_round_trips_through_json() {
        let fixture = Fixture {
            version: FIXTURE_VERSION,
            initial_nodes: 4,
            board_size: 500,
            nodes: None,
            settle_version: 0,
            moves: vec![FixtureMove {
                from: 0,
                to: 1,
                polyline: vec![[310.0, 250.0], [280.0, 280.0], [250.0, 310.0]],
                new_node: [280.0, 280.0],
            }],
        };
        let json = serde_json::to_string(&fixture).unwrap();
        assert!(json.contains("\"initialNodes\":4"));
        assert_eq!(parse(&json).unwrap(), fixture);
    }
}
