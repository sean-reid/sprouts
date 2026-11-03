//! Replays recorded games natively and times each engine phase per move.

use crate::ai;
use crate::components::label_components;
use crate::morphology::generate_skeleton;
use crate::node_classifier::classify_nodes;
pub use crate::types::Player;
use crate::types::{GameState, Move, Point};
use crate::validation::{validate_ai_move, validate_move};
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
    pub moves: Vec<FixtureMove>,
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

    fn to_move(&self, player: Player) -> Move {
        Move {
            from_node: self.from,
            to_node: self.to,
            polyline: self
                .polyline
                .iter()
                .map(|p| Point::new(p[0], p[1]))
                .collect(),
            new_node_pos: Point::new(self.new_node[0], self.new_node[1]),
            player,
        }
    }
}

pub const PHASES: [&str; 5] = [
    "skeleton",
    "classification",
    "generation",
    "search",
    "validation",
];

#[derive(Debug, Clone, Copy, Default)]
pub struct Phases {
    pub skeleton: Duration,
    pub classification: Duration,
    pub generation: Duration,
    pub search: Duration,
    pub validation: Duration,
}

impl Phases {
    pub fn as_array(&self) -> [Duration; 5] {
        [
            self.skeleton,
            self.classification,
            self.generation,
            self.search,
            self.validation,
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
    pub phases: Phases,
}

pub fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

/// Slowest run of each phase across the replay, in milliseconds.
pub fn max_per_phase(timings: &[MoveTiming]) -> [f64; 5] {
    let mut max = [0.0f64; 5];
    for t in timings {
        for (slot, d) in max.iter_mut().zip(t.phases.as_array()) {
            *slot = slot.max(ms(d));
        }
    }
    max
}

/// One line per phase whose max is over its budget.
pub fn breaches(name: &str, max: &[f64; 5], budgets: &BTreeMap<String, f64>) -> Vec<String> {
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

fn validate_for(state: &GameState, mov: &Move) -> Result<(), String> {
    match state.current_player {
        Player::Human => validate_move(state, mov),
        Player::AI => validate_ai_move(state, mov),
    }
}

/// Runs every phase of a turn at the position before each move, then applies it.
pub fn replay(fixture: &Fixture) -> Result<Vec<MoveTiming>, String> {
    let mut state = GameState::with_board_size(fixture.initial_nodes, fixture.board_size);
    let mut timings = Vec::with_capacity(fixture.moves.len());

    for (index, recorded) in fixture.moves.iter().enumerate() {
        let mut phases = Phases::default();

        let t = Instant::now();
        let (skeleton, distance_transform) = generate_skeleton(&state);
        state.skeleton_cache.skeleton = skeleton;
        state.skeleton_cache.distance_transform = distance_transform;
        let (labels, components) = label_components(&state.skeleton_cache.skeleton, &state);
        state.skeleton_cache.component_labels = labels;
        state.skeleton_cache.components = components;
        state.skeleton_cache.is_valid = true;
        phases.skeleton = t.elapsed();

        let t = Instant::now();
        let classification = classify_nodes(&mut state);
        phases.classification = t.elapsed();

        let t = Instant::now();
        let moves = ai::generate_concrete_moves_from_classification(&mut state, &classification);
        phases.generation = t.elapsed();

        let t = Instant::now();
        if moves.len() > 1 {
            black_box(ai::search_best_index(&state, &classification, &moves));
        }
        phases.search = t.elapsed();

        let mov = recorded.to_move(state.current_player);
        let t = Instant::now();
        let verdict = validate_for(&state, &mov);
        phases.validation = t.elapsed();
        verdict.map_err(|e| format!("move {} rejected: {}", index, e))?;

        timings.push(MoveTiming {
            index,
            player: state.current_player,
            phases,
        });
        state.apply_move(mov);
    }

    Ok(timings)
}

/// Plays the AI against itself, holding each side to the validation the page
/// applies to that side.
pub fn record(initial_nodes: usize, board_size: usize) -> Fixture {
    let mut state = GameState::with_board_size(initial_nodes, board_size);
    let mut moves = Vec::new();
    while let Some(mov) = next_accepted_move(&mut state) {
        moves.push(FixtureMove::from_move(&mov));
        state.apply_move(mov);
    }
    Fixture {
        version: FIXTURE_VERSION,
        initial_nodes,
        board_size,
        moves,
    }
}

fn next_accepted_move(state: &mut GameState) -> Option<Move> {
    let preferred = ai::select_ai_move(state)?;
    if validate_for(state, &preferred).is_ok() {
        return Some(preferred);
    }
    let classification = classify_nodes(state);
    let mut accepted: Vec<Move> =
        ai::generate_concrete_moves_from_classification(state, &classification)
            .into_iter()
            .filter(|m| validate_for(state, m).is_ok())
            .collect();
    if accepted.len() > 1 {
        let best = ai::search_best_index(state, &classification, &accepted);
        accepted.swap(0, best);
    }
    accepted.into_iter().next()
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
        let timings = replay(&fixture).unwrap();
        assert_eq!(timings.len(), fixture.moves.len());
        assert_eq!(timings[0].player, Player::Human);
        assert_eq!(timings[1].player, Player::AI);
        let max = max_per_phase(&timings);
        assert!(max[0] > 0.0 && max[1] > 0.0 && max[2] > 0.0 && max[4] > 0.0);
    }

    #[test]
    fn self_play_records_a_game_that_replays() {
        let fixture = record(4, 500);
        assert!(fixture.moves.len() >= 2);
        let timings = replay(&fixture).unwrap();
        assert_eq!(timings.len(), fixture.moves.len());
    }

    #[test]
    fn breaches_name_only_phases_over_budget() {
        let budgets: BTreeMap<String, f64> = PHASES.iter().map(|p| (p.to_string(), 10.0)).collect();
        let max = [5.0, 10.0, 10.5, 0.0, 200.0];
        let hits = breaches("g", &max, &budgets);
        assert_eq!(hits.len(), 2);
        assert!(hits[0].starts_with("g: generation 10.50 ms over budget 10.00 ms"));
        assert!(hits[1].starts_with("g: validation 200.00 ms"));
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
