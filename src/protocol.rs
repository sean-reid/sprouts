use serde::{Deserialize, Serialize};
use tsify::Tsify;

use crate::ai;
use crate::curve::{self, Cubic};
use crate::embedding;
use crate::geometry;
use crate::types::{self, scale, GameState, Line, Node, Player, Point, MIN_NODE_SPACING};
use crate::validation::{self, Rejection};

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
    fn by(self, player: Player, board_size: usize) -> types::Move {
        types::Move::new(
            self.from_node,
            self.to_node,
            self.polyline,
            self.new_node_pos,
            player,
            board_size,
        )
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

/// The AI's move with the spline it commits as, so the page animates the
/// curve the board will show.
/// The AI's answer: a move, no move because the game is over, or no move
/// because none of the listed moves could be drawn while the game is open.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum AiReply {
    Move { mov: Move, segments: Vec<Cubic> },
    GameOver,
    Undrawn,
}

#[derive(Debug, Clone, Deserialize, Tsify)]
pub struct FitQuery {
    pub path: Vec<Point>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
pub struct Curve {
    pub segments: Vec<Cubic>,
}

#[derive(Debug, Clone, Deserialize, Tsify)]
pub struct PlacementQuery {
    pub segments: Vec<Cubic>,
    pub at: Point,
}

#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
pub struct Placement {
    pub position: Point,
    pub valid: bool,
}

#[derive(Debug, Clone, Deserialize, Tsify)]
pub struct AutoPlacementQuery {
    pub segments: Vec<Cubic>,
}

/// The best spot on a locked curve for its new node, or why there is none.
#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum AutoPlacement {
    Placed { placement: Placement },
    Rejected { rejection: Rejection },
}

/// Starting node positions a restored game brings with it.
#[derive(Debug, Clone, Deserialize, Tsify)]
pub struct Layout {
    pub nodes: Vec<Point>,
}

/// A fresh board on the recorded positions, or on the default layout when
/// none are given. Positions that do not match the count are an error.
pub fn start(
    initial_nodes: usize,
    board_size: usize,
    first_player: Player,
    nodes: Option<Vec<Point>>,
) -> Result<GameState, String> {
    let mut state = match nodes {
        Some(nodes) if nodes.len() == initial_nodes => GameState::with_nodes(nodes, board_size),
        Some(nodes) => {
            return Err(format!(
                "{} starting positions for {initial_nodes} nodes",
                nodes.len()
            ))
        }
        None => GameState::with_board_size(initial_nodes, board_size),
    };
    state.current_player = first_player;
    Ok(state)
}

/// A node is active when the face model gives it a legal move.
pub fn snapshot(state: &GameState) -> Snapshot {
    let active_nodes = state.face.active_nodes();
    Snapshot {
        board_size: state.board_size,
        initial_nodes: state.initial_node_count,
        nodes: state.nodes.clone(),
        lines: state.lines.clone(),
        current_player: state.current_player,
        game_over: active_nodes.is_empty(),
        active_nodes,
    }
}

pub fn check(state: &GameState, mov: &types::Move) -> Result<(), Rejection> {
    match state.current_player {
        Player::AI => validation::validate_ai_move(state, mov),
        Player::Human => validation::validate_move(state, mov),
    }?;
    embedding::embed(state, mov).map(drop)
}

pub fn apply(state: &mut GameState, mov: Move) -> Outcome {
    let mov = mov.by(state.current_player, state.board_size);
    match check(state, &mov).and_then(|()| state.apply_move(mov)) {
        Ok(()) => Outcome::Applied {
            snapshot: snapshot(state),
        },
        Err(rejection) => Outcome::Rejected { rejection },
    }
}

pub fn ai_move(state: &mut GameState) -> AiReply {
    match ai::select_ai_move(state) {
        ai::Reply::Move(mov) => AiReply::Move {
            segments: mov.segments.clone(),
            mov: Move::from(mov),
        },
        ai::Reply::GameOver => AiReply::GameOver,
        ai::Reply::Undrawn => AiReply::Undrawn,
    }
}

/// The spline a stroke commits as, so the page can show it before the move.
pub fn fit(state: &GameState, query: FitQuery) -> Curve {
    Curve {
        segments: curve::fit(&query.path, curve::tolerance(state.board_size)),
    }
}

pub fn placement(state: &GameState, query: PlacementQuery) -> Placement {
    let Some(hit) = curve::nearest(&query.segments, &query.at) else {
        return Placement {
            position: query.at,
            valid: false,
        };
    };
    let path = curve::flatten(&query.segments, curve::tolerance(state.board_size));
    let valid =
        validation::validate_new_node_placement(&path, &hit.point, &state.nodes, state.board_size)
            .is_ok();
    Placement {
        position: hit.point,
        valid,
    }
}

/// Candidates are spaced two curve tolerances apart along the flattening,
/// ranked by clearance, kept a node spacing in from every board edge, and
/// the first that validates wins.
pub fn auto_placement(state: &GameState, query: AutoPlacementQuery) -> AutoPlacement {
    let tolerance = curve::tolerance(state.board_size);
    let flat = curve::flatten(&query.segments, tolerance);
    let path = geometry::resample_polyline(&flat, tolerance * 2.0);
    let inset = MIN_NODE_SPACING * scale(state.board_size);
    let far = state.board_size as f64 - inset;
    let inside = |at: &Point| (inset..=far).contains(&at.x) && (inset..=far).contains(&at.y);
    let valid = |at: &Point| {
        validation::validate_new_node_placement(&flat, at, &state.nodes, state.board_size).is_ok()
    };
    let chosen = ai::rank_node_placements(&path, state)
        .into_iter()
        .filter_map(|at| curve::nearest(&query.segments, &at))
        .map(|hit| hit.point)
        .filter(inside)
        .find(valid);
    match chosen {
        Some(position) => AutoPlacement::Placed {
            placement: Placement {
                position,
                valid: true,
            },
        },
        None => {
            let centre = state.board_size as f64 / 2.0;
            let midpoint = path
                .get(path.len() / 2)
                .copied()
                .unwrap_or(Point::new(centre, centre));
            AutoPlacement::Rejected {
                rejection: Rejection::no_room(midpoint),
            }
        }
    }
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
        let state = GameState::new(4);
        let snap = snapshot(&state);
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

    /// One node: a loop, then the chord that closes it. Node 2 keeps a life
    /// but has nobody to reach, so it is not active either.
    #[test]
    fn snapshot_of_a_finished_game() {
        let mut state = start(1, 1000, Player::Human, None).unwrap();
        let a = state.nodes[0].position;
        let ring: Vec<Point> = (0..=48)
            .map(|i| {
                let t = i as f64 / 48.0 * std::f64::consts::TAU;
                Point::new(a.x + 60.0 - 60.0 * t.cos(), a.y + 60.0 * t.sin())
            })
            .collect();
        let far = ring[24];
        let Outcome::Applied { snapshot: snap } = apply(
            &mut state,
            Move {
                from_node: 0,
                to_node: 0,
                polyline: ring,
                new_node_pos: far,
            },
        ) else {
            panic!("the loop was rejected");
        };
        assert!(!snap.game_over);
        assert_eq!(snap.active_nodes, vec![0, 1]);
        // The chord is the AI's, so it leaves and arrives clear of the
        // ring's tangents rather than along them.
        let over = vec![
            a,
            Point::new(a.x - 40.0, a.y - 100.0),
            Point::new(far.x + 40.0, far.y - 100.0),
            far,
        ];
        let outcome = apply(
            &mut state,
            Move {
                from_node: 0,
                to_node: 1,
                polyline: over.clone(),
                new_node_pos: over[1],
            },
        );
        let Outcome::Applied { snapshot: snap } = outcome else {
            panic!("the closing chord was rejected: {outcome:?}");
        };
        assert!(snap.game_over);
        assert_eq!(snap.active_nodes, Vec::<usize>::new());
        assert_eq!(snap.nodes[2].connection_count, 2);
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
    fn start_takes_recorded_positions_and_refuses_a_count_mismatch() {
        let given = vec![Point::new(100.0, 100.0), Point::new(400.0, 100.0)];
        let placed = start(2, 500, Player::AI, Some(given.clone())).unwrap();
        assert_eq!(placed.nodes[1].position, given[1]);
        assert_eq!(placed.current_player, Player::AI);
        let refused = start(3, 500, Player::Human, Some(given))
            .err()
            .expect("a count mismatch is refused");
        assert!(refused.contains("2 starting positions for 3 nodes"));
        let laid_out = start(3, 500, Player::Human, None).unwrap();
        assert_eq!(laid_out.nodes[0].position, types::opening_layout(3, 500)[0]);
    }

    #[test]
    fn the_ai_opening_on_a_full_size_board_is_accepted() {
        let mut state = start(4, 1000, Player::AI, None).unwrap();
        let AiReply::Move { mov, .. } = ai_move(&mut state) else {
            panic!("the AI has an opening move");
        };
        match apply(&mut state, mov) {
            Outcome::Applied { snapshot } => assert_eq!(snapshot.current_player, Player::Human),
            Outcome::Rejected { rejection } => panic!("opening rejected: {}", rejection.message),
        }
    }

    #[test]
    fn the_ai_answers_a_human_opening_with_a_move_that_applies() {
        let mut state = start(4, 1000, Player::Human, None).unwrap();
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let polyline = straight(a, b, 20);
        let new_node_pos = polyline[10];
        let Outcome::Applied { .. } = apply(
            &mut state,
            Move {
                from_node: 0,
                to_node: 1,
                polyline,
                new_node_pos,
            },
        ) else {
            panic!("the opening chord was rejected");
        };
        let AiReply::Move { mov, .. } = ai_move(&mut state) else {
            panic!("the AI has a reply");
        };
        match apply(&mut state, mov) {
            Outcome::Applied { snapshot } => {
                assert_eq!(snapshot.current_player, Player::Human);
                assert_eq!(snapshot.lines.len(), 2);
            }
            Outcome::Rejected { rejection } => panic!("reply rejected: {}", rejection.message),
        }
    }

    #[test]
    fn the_ai_can_open_and_the_turn_passes_to_the_human() {
        let mut state = start(4, 1000, Player::AI, None).unwrap();
        assert_eq!(snapshot(&state).current_player, Player::AI);
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

    fn fitted(state: &GameState, path: Vec<Point>) -> Vec<Cubic> {
        fit(state, FitQuery { path }).segments
    }

    #[test]
    fn placement_on_an_existing_node_is_invalid() {
        let state = GameState::new(4);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let segments = fitted(&state, straight(a, b, 10));
        let placed = placement(&state, PlacementQuery { segments, at: a });
        assert!(!placed.valid);
        assert_eq!(placed.position, a);
    }

    #[derive(Serialize, Deserialize)]
    #[serde(rename_all = "camelCase")]
    struct Recorded {
        initial_nodes: usize,
        board_size: usize,
        nodes: Option<Vec<[f64; 2]>>,
        moves: Vec<Move>,
    }

    fn replay(recorded: Recorded) -> GameState {
        let nodes = recorded
            .nodes
            .map(|nodes| nodes.iter().map(|p| Point::new(p[0], p[1])).collect());
        let mut state = start(
            recorded.initial_nodes,
            recorded.board_size,
            Player::Human,
            nodes,
        )
        .unwrap();
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
    fn placement_with_no_curve_is_invalid() {
        let state = GameState::new(4);
        let at = Point::new(10.0, 10.0);
        let placed = placement(
            &state,
            PlacementQuery {
                segments: vec![],
                at,
            },
        );
        assert!(!placed.valid);
        assert_eq!(placed.position, at);
    }

    fn wavy(a: Point, b: Point) -> Vec<Point> {
        (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                let len = (dx * dx + dy * dy).sqrt();
                let bulge = 40.0 * (t * 3.0 * std::f64::consts::PI).sin();
                Point::new(
                    a.x + dx * t - dy / len * bulge,
                    a.y + dy * t + dx / len * bulge,
                )
            })
            .collect()
    }

    #[test]
    fn the_fit_shown_before_a_move_is_the_curve_the_move_commits() {
        let mut state = GameState::new(4);
        let path = wavy(state.nodes[0].position, state.nodes[1].position);
        let shown = fitted(&state, path.clone());
        assert!(shown.len() > 1);
        let placed = placement(
            &state,
            PlacementQuery {
                segments: shown.clone(),
                at: path[20],
            },
        );
        assert!(placed.valid);
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline: path,
            new_node_pos: placed.position,
        };
        let Outcome::Applied { snapshot } = apply(&mut state, mov) else {
            panic!("wavy chord rejected");
        };
        assert_eq!(snapshot.lines[0].segments, shown);
        assert_eq!(snapshot.lines[0].new_node_pos, placed.position);
        assert_eq!(snapshot.nodes[4].position, placed.position);
        let line = &state.lines[0];
        assert_eq!(line.flat, line.flatten(curve::tolerance(state.board_size)));
    }

    #[test]
    fn the_ai_reply_carries_the_segments_its_move_commits() {
        let mut state = GameState::with_board_size(4, 500);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let polyline = straight(a, b, 10);
        let new_node_pos = polyline[5];
        let opening = Move {
            from_node: 0,
            to_node: 1,
            polyline,
            new_node_pos,
        };
        assert!(matches!(
            apply(&mut state, opening),
            Outcome::Applied { .. }
        ));
        let AiReply::Move { mov, segments } = ai_move(&mut state) else {
            panic!("the AI has a move on an open board");
        };
        let Outcome::Applied { snapshot } = apply(&mut state, mov) else {
            panic!("the AI's own move was rejected");
        };
        assert_eq!(snapshot.lines.last().unwrap().segments, segments);
    }

    #[test]
    fn a_line_on_the_wire_carries_its_stroke_and_segments_but_not_the_flattening() {
        let mut state = GameState::new(4);
        let path = wavy(state.nodes[0].position, state.nodes[1].position);
        let new_node_pos = path[20];
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline: path,
            new_node_pos,
        };
        let wire = serde_json::to_value(apply(&mut state, mov)).unwrap();
        let line = &wire["snapshot"]["lines"][0];
        assert!(line.get("flat").is_none());
        assert_eq!(line["polyline"].as_array().unwrap().len(), 41);
        let segments = line["segments"].as_array().unwrap();
        assert!(segments.len() > 1);
        assert_eq!(
            segments[0]["p0"],
            serde_json::to_value(state.nodes[0].position).unwrap()
        );
        assert_eq!(
            segments[segments.len() - 1]["p3"],
            serde_json::to_value(state.nodes[1].position).unwrap()
        );
    }

    #[test]
    fn a_move_still_validates_after_a_round_trip_through_json() {
        let state = GameState::new(4);
        let path = wavy(state.nodes[0].position, state.nodes[1].position);
        let new_node_pos = path[20];
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline: path,
            new_node_pos,
        };
        let text = serde_json::to_string(&mov).unwrap();
        let back: Move = serde_json::from_str(&text).unwrap();
        let mut direct = state.clone();
        let mut replayed = state.clone();
        let (Outcome::Applied { snapshot: a }, Outcome::Applied { snapshot: b }) =
            (apply(&mut direct, mov), apply(&mut replayed, back))
        else {
            panic!("the chord was rejected");
        };
        // JSON parsing is only exact to the last bit with a serde_json
        // feature the app does not use, so the splines match to a micron.
        assert_eq!(a.lines[0].segments.len(), b.lines[0].segments.len());
        for (x, y) in a.lines[0].segments.iter().zip(&b.lines[0].segments) {
            for (px, py) in [(x.p0, y.p0), (x.p1, y.p1), (x.p2, y.p2), (x.p3, y.p3)] {
                assert!(px.distance_to(&py) < 1e-6);
            }
        }
        assert!(
            a.lines[0]
                .new_node_pos
                .distance_to(&b.lines[0].new_node_pos)
                < 1e-6
        );
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
            nodes: None,
            moves: vec![chord(0, 3), chord(1, 2)],
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
        let segments = fitted(&state, path);
        let placed = placement(
            &state,
            PlacementQuery {
                segments,
                at: off_path,
            },
        );
        assert!(placed.valid);
        assert!(placed.position.distance_to(&off_path) < 10.0);
        assert_ne!(placed.position, off_path);
    }

    fn suggested(state: &GameState, segments: Vec<Cubic>) -> AutoPlacement {
        auto_placement(state, AutoPlacementQuery { segments })
    }

    #[test]
    fn the_suggested_spot_on_an_open_chord_is_its_middle_and_valid() {
        let state = GameState::new(4);
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let segments = fitted(&state, straight(a, b, 10));
        let AutoPlacement::Placed { placement } = suggested(&state, segments.clone()) else {
            panic!("an open chord has room for a node");
        };
        assert!(placement.valid);
        let mid = Point::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
        assert!(placement.position.distance_to(&mid) < 2.0);
        let wire = serde_json::to_value(suggested(&state, segments)).unwrap();
        assert_eq!(wire["kind"], json!("placed"));
        assert_eq!(wire["placement"]["valid"], json!(true));
    }

    #[test]
    fn the_suggested_spot_stays_on_the_board_when_the_curve_leaves_it() {
        let mut state = GameState::new(4);
        state.nodes[0].position = Point::new(300.0, 60.0);
        state.nodes[1].position = Point::new(700.0, 60.0);
        let arc: Vec<Point> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                Point::new(
                    300.0 + t * 400.0,
                    60.0 - 100.0 * (t * std::f64::consts::PI).sin(),
                )
            })
            .collect();
        let segments = fitted(&state, arc);
        let AutoPlacement::Placed { placement } = suggested(&state, segments.clone()) else {
            panic!("the arc has room for a node inside the board");
        };
        assert!(placement.position.y >= MIN_NODE_SPACING);
        assert!(placement.valid);
        let check = placement_at(&state, segments, placement.position);
        assert!(check.valid);
    }

    fn placement_at(state: &GameState, segments: Vec<Cubic>, at: Point) -> Placement {
        placement(state, PlacementQuery { segments, at })
    }

    #[test]
    fn a_loop_hugging_its_node_has_no_room_and_is_rejected_at_its_midpoint() {
        let state = GameState::new(4);
        let a = state.nodes[0].position;
        let radius = 8.0;
        let loop_path: Vec<Point> = (0..=32)
            .map(|i| {
                let angle = std::f64::consts::PI + 2.0 * std::f64::consts::PI * i as f64 / 32.0;
                Point::new(
                    a.x + radius + radius * angle.cos(),
                    a.y + radius * angle.sin(),
                )
            })
            .collect();
        let segments = fitted(&state, loop_path);
        let AutoPlacement::Rejected { rejection } = suggested(&state, segments) else {
            panic!("every point of the loop is within node spacing of its node");
        };
        assert_eq!(rejection.kind, validation::RejectionKind::NoRoomForNode);
        let validation::BoardLocation::Point { at } = rejection.at else {
            panic!("the rejection points at the curve");
        };
        let far_side = Point::new(a.x + 2.0 * radius, a.y);
        assert!(at.distance_to(&far_side) < 4.0);
        let wire = serde_json::to_value(AutoPlacement::Rejected { rejection }).unwrap();
        assert_eq!(wire["kind"], json!("rejected"));
        assert_eq!(wire["rejection"]["kind"], json!("noRoomForNode"));
    }

    #[test]
    fn no_curve_has_no_room() {
        let state = GameState::new(4);
        assert!(matches!(
            suggested(&state, vec![]),
            AutoPlacement::Rejected { .. }
        ));
    }

    /// A pocket where the three listed moves are all too cramped to draw:
    /// the AI reports that it cannot draw, and the game stays open. The
    /// pocket serves only while the router cannot draw it; if routing
    /// improves, record a tighter one.
    #[test]
    fn an_undrawn_position_does_not_end_the_game() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures/undrawn/pocket-6n-1000.json");
        let raw: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let nodes: Vec<Point> = raw["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| Point::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
            .collect();
        let mut state = start(6, 1000, Player::AI, Some(nodes)).unwrap();
        for m in raw["moves"].as_array().unwrap() {
            let polyline: Vec<Point> = m["polyline"]
                .as_array()
                .unwrap()
                .iter()
                .map(|p| Point::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
                .collect();
            let new_node = Point::new(
                m["newNode"][0].as_f64().unwrap(),
                m["newNode"][1].as_f64().unwrap(),
            );
            let mov = types::Move::new(
                m["from"].as_u64().unwrap() as usize,
                m["to"].as_u64().unwrap() as usize,
                polyline,
                new_node,
                Player::AI,
                1000,
            );
            state.apply_move(mov).unwrap();
            state.current_player = Player::AI;
        }
        assert!(!state.face.search_moves().is_empty());
        assert!(matches!(ai_move(&mut state), AiReply::Undrawn));
        assert!(!snapshot(&state).game_over);
    }
}
