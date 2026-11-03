use serde::{Deserialize, Serialize};
use tsify::Tsify;

use crate::game_over;
use crate::node_classifier;
use crate::types::{self, GameState, Line, Node, Player, Point};
use crate::validation::{self, Rejection};
use crate::{ai, geometry};

/// A move as proposed to the engine. The mover is whoever is to play, so a
/// recorded game is the list of these in order plus the opening node count.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Move {
    pub from_node: usize,
    pub to_node: usize,
    pub polyline: Vec<Point>,
    pub new_node_pos: Point,
}

impl Move {
    fn by(self, player: Player) -> types::Move {
        types::Move {
            from_node: self.from_node,
            to_node: self.to_node,
            polyline: self.polyline,
            new_node_pos: self.new_node_pos,
            player,
        }
    }
}

impl From<types::Move> for Move {
    fn from(mov: types::Move) -> Self {
        Self {
            from_node: mov.from_node,
            to_node: mov.to_node,
            polyline: mov.polyline,
            new_node_pos: mov.new_node_pos,
        }
    }
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Snapshot {
    pub board_size: usize,
    pub initial_nodes: usize,
    pub nodes: Vec<Node>,
    pub lines: Vec<Line>,
    pub current_player: Player,
    pub active_nodes: Vec<usize>,
    pub game_over: bool,
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum Outcome {
    Applied { snapshot: Snapshot },
    Rejected { rejection: Rejection },
}

#[derive(Debug, Clone, Deserialize, Tsify)]
pub struct PlacementQuery {
    pub path: Vec<Point>,
    pub at: Point,
}

#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
pub struct Placement {
    pub position: Point,
    pub valid: bool,
}

pub fn start(initial_nodes: usize, board_size: usize, first_player: Player) -> GameState {
    let mut state = GameState::with_board_size(initial_nodes, board_size);
    state.current_player = first_player;
    state
}

pub fn snapshot(state: &mut GameState) -> Snapshot {
    let classification = node_classifier::classify_nodes(state);
    let game_over = game_over::is_game_over(state, &classification);
    let mut active_nodes: Vec<usize> = classification.active_nodes.into_iter().collect();
    active_nodes.sort_unstable();
    Snapshot {
        board_size: state.board_size,
        initial_nodes: state.initial_node_count,
        nodes: state.nodes.clone(),
        lines: state.lines.clone(),
        current_player: state.current_player,
        active_nodes,
        game_over,
    }
}

// The AI generator keeps a move that passes either check, so apply must too.
pub fn check(state: &GameState, mov: &types::Move) -> Result<(), Rejection> {
    match state.current_player {
        Player::AI => validation::validate_ai_move(state, mov)
            .or_else(|_| validation::validate_ai_move_tight(state, mov)),
        Player::Human => validation::validate_move(state, mov),
    }
}

pub fn apply(state: &mut GameState, mov: Move) -> Outcome {
    let mov = mov.by(state.current_player);
    match check(state, &mov) {
        Ok(()) => {
            state.apply_move(mov);
            Outcome::Applied {
                snapshot: snapshot(state),
            }
        }
        Err(rejection) => Outcome::Rejected { rejection },
    }
}

pub fn ai_move(state: &mut GameState) -> Option<Move> {
    ai::select_ai_move(state).map(Move::from)
}

pub fn placement(state: &GameState, query: PlacementQuery) -> Placement {
    let (position, _) = geometry::closest_point_on_polyline(&query.path, &query.at);
    let valid = validation::validate_new_node_placement(
        &query.path,
        &position,
        &state.nodes,
        state.board_size,
    )
    .is_ok();
    Placement { position, valid }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn straight(a: Point, b: Point, steps: usize) -> Vec<Point> {
        (0..=steps)
            .map(|i| {
                let t = i as f64 / steps as f64;
                Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
            })
            .collect()
    }

    #[test]
    fn move_wire_format_is_stable() {
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline: vec![Point::new(1.0, 2.0), Point::new(3.0, 4.0)],
            new_node_pos: Point::new(2.0, 3.0),
        };
        let wire = serde_json::to_value(&mov).unwrap();
        assert_eq!(
            wire,
            json!({
                "fromNode": 0,
                "toNode": 1,
                "polyline": [{ "x": 1.0, "y": 2.0 }, { "x": 3.0, "y": 4.0 }],
                "newNodePos": { "x": 2.0, "y": 3.0 },
            })
        );
        assert_eq!(serde_json::from_value::<Move>(wire).unwrap(), mov);
    }

    #[test]
    fn snapshot_of_a_fresh_board() {
        let mut state = GameState::new(4);
        let snap = snapshot(&mut state);
        assert_eq!(snap.nodes.len(), 4);
        assert!(snap.lines.is_empty());
        assert_eq!(snap.active_nodes, vec![0, 1, 2, 3]);
        assert_eq!(snap.current_player, Player::Human);
        assert!(!snap.game_over);
        let wire = serde_json::to_value(&snap).unwrap();
        assert_eq!(wire["currentPlayer"], json!("human"));
        assert_eq!(wire["nodes"][0]["connectionCount"], json!(0));
        assert_eq!(wire["boardSize"], json!(1000));
        assert_eq!(wire["initialNodes"], json!(4));
    }

    #[test]
    fn apply_reports_the_rejection_with_its_location() {
        let mut state = GameState::new(4);
        let a = state.nodes[0].position;
        let mov = Move {
            from_node: 0,
            to_node: 0,
            polyline: vec![a, a],
            new_node_pos: a,
        };
        let wire = serde_json::to_value(apply(&mut state, mov)).unwrap();
        assert_eq!(wire["kind"], json!("rejected"));
        assert_eq!(wire["rejection"]["kind"], json!("pathTooShort"));
        assert_eq!(
            wire["rejection"]["at"],
            json!({ "kind": "segments", "first": 0, "last": 0 })
        );
        assert!(!wire["rejection"]["message"].as_str().unwrap().is_empty());
        assert!(state.lines.is_empty());
    }

    #[test]
    fn the_ai_opening_on_a_full_size_board_is_accepted() {
        let mut state = start(4, 1000, Player::AI);
        let mov = ai_move(&mut state).expect("the AI has an opening move");
        match apply(&mut state, mov) {
            Outcome::Applied { snapshot } => assert_eq!(snapshot.current_player, Player::Human),
            Outcome::Rejected { rejection } => panic!("opening rejected: {}", rejection.message),
        }
    }

    #[test]
    fn the_ai_can_open_and_the_turn_passes_to_the_human() {
        let mut state = start(4, 1000, Player::AI);
        assert_eq!(snapshot(&mut state).current_player, Player::AI);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let polyline = straight(a, b, 10);
        let new_node_pos = polyline[5];
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline,
            new_node_pos,
        };
        match apply(&mut state, mov) {
            Outcome::Applied { snapshot } => {
                assert_eq!(snapshot.current_player, Player::Human);
                assert_eq!(snapshot.lines[0].player, Player::AI);
            }
            Outcome::Rejected { rejection } => panic!("opening rejected: {}", rejection.message),
        }
        assert!(state.undo_move());
        assert_eq!(state.current_player, Player::AI);
    }

    #[test]
    fn apply_returns_the_board_after_the_move() {
        let mut state = GameState::new(4);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let polyline = straight(a, b, 10);
        let new_node_pos = polyline[5];
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline,
            new_node_pos,
        };
        let wire = serde_json::to_value(apply(&mut state, mov)).unwrap();
        assert_eq!(wire["kind"], json!("applied"));
        let snap = &wire["snapshot"];
        assert_eq!(snap["currentPlayer"], json!("ai"));
        assert_eq!(snap["lines"].as_array().unwrap().len(), 1);
        assert_eq!(snap["lines"][0]["player"], json!("human"));
        assert_eq!(snap["nodes"].as_array().unwrap().len(), 5);
    }

    #[test]
    fn placement_on_an_existing_node_is_invalid() {
        let state = GameState::new(4);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let path = straight(a, b, 10);
        let placed = placement(&state, PlacementQuery { path, at: a });
        assert!(!placed.valid);
        assert_eq!(placed.position, a);
    }

    #[derive(Serialize, Deserialize)]
    #[serde(rename_all = "camelCase")]
    struct Recorded {
        initial_nodes: usize,
        board_size: usize,
        moves: Vec<Move>,
    }

    fn replay(recorded: Recorded) -> GameState {
        let mut state = GameState::with_board_size(recorded.initial_nodes, recorded.board_size);
        for mov in recorded.moves {
            match apply(&mut state, mov) {
                Outcome::Applied { .. } => {}
                Outcome::Rejected { rejection } => {
                    panic!("replay rejected: {}", rejection.message)
                }
            }
        }
        state
    }

    #[test]
    fn a_recorded_game_replays_from_its_move_list() {
        let layout = GameState::new(4);
        let chord = |from: usize, to: usize| {
            let polyline = straight(layout.nodes[from].position, layout.nodes[to].position, 10);
            let new_node_pos = polyline[5];
            Move {
                from_node: from,
                to_node: to,
                polyline,
                new_node_pos,
            }
        };
        let fixture = serde_json::to_string(&Recorded {
            initial_nodes: 4,
            board_size: 1000,
            moves: vec![chord(0, 1), chord(2, 3)],
        })
        .unwrap();

        let state = replay(serde_json::from_str(&fixture).unwrap());
        assert_eq!(state.lines.len(), 2);
        assert_eq!(state.lines[1].player, Player::AI);
        assert_eq!(state.current_player, Player::Human);
    }

    #[test]
    fn placement_snaps_to_the_path() {
        let state = GameState::new(4);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let path = straight(a, b, 10);
        let off_path = Point::new(path[5].x + 7.0, path[5].y + 7.0);
        let placed = placement(&state, PlacementQuery { path, at: off_path });
        assert!(placed.valid);
        assert!(placed.position.distance_to(&off_path) < 10.0);
        assert_ne!(placed.position, off_path);
    }
}
