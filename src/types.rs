use serde::{Deserialize, Serialize};
use tsify::Tsify;

use crate::curve::{self, Cubic};
use crate::embedding;
use crate::face::FaceModel;
use crate::validation::Rejection;

pub const BOARD_SIZE: usize = 1000;

// All spatial constants are tuned for a 1000px board.
// Use scale(board_size) to adapt them to smaller boards (e.g. 500px mobile).
pub fn scale(board_size: usize) -> f64 {
    board_size as f64 / BOARD_SIZE as f64
}

// Validation thresholds (base values for 1000px board)
pub const MIN_PATH_LENGTH: f64 = 20.0;
pub const MIN_NODE_SPACING: f64 = 20.0;
pub const MIN_NODE_SPACING_AI: f64 = 15.0;
pub const MIN_PATH_CLEARANCE: f64 = 10.0;
pub const MIN_PATH_CLEARANCE_AI: f64 = 6.0;
pub const HUMAN_INTERSECTION_TOLERANCE: f64 = 30.0;
pub const AI_INTERSECTION_TOLERANCE: f64 = 8.0;

/// Radius of a node disc on the page, in CSS pixels.
pub const NODE_DISC_PX: f64 = 8.0;
/// Width of the board on the narrowest layout, the 375 px phone page, in
/// CSS pixels.
pub const NARROWEST_BOARD_PX: f64 = 340.0;

/// Radius of a node disc in board units on the narrowest layout, where the
/// discs are largest against the board.
pub fn node_disc(board_size: usize) -> f64 {
    NODE_DISC_PX * board_size as f64 / NARROWEST_BOARD_PX
}

/// Nearest an AI line comes to the centre of a node it does not touch: the
/// disc, plus the line clearance.
pub fn ai_node_clearance(board_size: usize) -> f64 {
    node_disc(board_size) + MIN_PATH_CLEARANCE_AI * scale(board_size)
}

/// Fraction of the board the outermost starting node sits from the centre.
pub const LAYOUT_REACH: f64 = 0.35;

/// Starting nodes on a sunflower spiral: node i at radius proportional to
/// the square root of i + 0.5 and i golden angles round, scaled so the last
/// node reaches `LAYOUT_REACH` of the board. Neighbours stay at least four
/// node spacings apart up to 24 nodes.
pub fn opening_layout(count: usize, board_size: usize) -> Vec<Point> {
    let golden_angle = std::f64::consts::PI * (3.0 - 5f64.sqrt());
    let centre = board_size as f64 / 2.0;
    let unit = LAYOUT_REACH * board_size as f64 / (count as f64 - 0.5).sqrt();
    (0..count)
        .map(|i| {
            let radius = unit * (i as f64 + 0.5).sqrt();
            let angle = i as f64 * golden_angle;
            crate::exact::snap_to_grid(Point::new(
                centre + radius * angle.cos(),
                centre + radius * angle.sin(),
            ))
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Tsify)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: &Point) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "lowercase")]
pub enum Player {
    Human,
    AI,
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Node {
    pub id: usize,
    pub position: Point,
    pub connection_count: u8,
}

impl Node {
    pub fn new(id: usize, position: Point) -> Self {
        Self {
            id,
            position: crate::exact::snap_to_grid(position),
            connection_count: 0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Line {
    pub id: usize,
    pub from_node: usize,
    pub to_node: usize,
    pub polyline: Vec<Point>,
    pub segments: Vec<Cubic>,
    // Flattening of segments at the board's curve tolerance, cached for the
    // intersection tests against later moves.
    #[serde(skip)]
    pub flat: Vec<Point>,
    pub new_node_pos: Point,
    pub player: Player,
}

impl Line {
    pub fn flatten(&self, tolerance: f64) -> Vec<Point> {
        curve::flatten(&self.segments, tolerance)
    }
}

#[derive(Debug, Clone)]
pub struct Move {
    pub from_node: usize,
    pub to_node: usize,
    pub polyline: Vec<Point>,
    pub segments: Vec<Cubic>,
    pub new_node_pos: Point,
    pub player: Player,
}

/// The node's spot on the curve, on the grid: a point already within a
/// grid step of the curve is kept, so a node that has been placed once
/// comes back from a record or a link as the same point; any other is the
/// nearest point of the curve.
fn on_curve(segments: &[Cubic], at: Point) -> Point {
    let hit = curve::nearest(segments, &at).filter(|hit| hit.distance > crate::exact::GRID);
    crate::exact::snap_to_grid(hit.map_or(at, |hit| hit.point))
}

impl Move {
    /// Fits the spline to the drawn polyline and puts the new node on it.
    pub fn new(
        from_node: usize,
        to_node: usize,
        polyline: Vec<Point>,
        new_node_pos: Point,
        player: Player,
        board_size: usize,
    ) -> Self {
        let segments = curve::fit(&polyline, curve::tolerance(board_size));
        let new_node_pos = on_curve(&segments, new_node_pos);
        Self {
            from_node,
            to_node,
            polyline,
            segments,
            new_node_pos,
            player,
        }
    }

    /// A move whose spline is already fit, with its node on that curve.
    pub fn from_segments(
        from_node: usize,
        to_node: usize,
        polyline: Vec<Point>,
        segments: Vec<Cubic>,
        new_node_pos: Point,
        player: Player,
    ) -> Self {
        let new_node_pos = on_curve(&segments, new_node_pos);
        Self {
            from_node,
            to_node,
            polyline,
            segments,
            new_node_pos,
            player,
        }
    }

    pub fn flatten(&self, tolerance: f64) -> Vec<Point> {
        curve::flatten(&self.segments, tolerance)
    }
}

#[derive(Clone)]
pub struct GameState {
    pub nodes: Vec<Node>,
    pub lines: Vec<Line>,
    pub face: FaceModel,
    pub move_history: Vec<Move>,
    pub current_player: Player,
    pub next_node_id: usize,
    pub next_line_id: usize,
    pub initial_node_count: usize,
    pub board_size: usize,
}

impl GameState {
    #[cfg(test)]
    pub fn new(initial_nodes: usize) -> Self {
        Self::with_board_size(initial_nodes, BOARD_SIZE)
    }

    pub fn with_board_size(initial_nodes: usize, board_size: usize) -> Self {
        Self::with_nodes(opening_layout(initial_nodes, board_size), board_size)
    }

    /// A fresh game whose starting nodes sit at `positions`.
    pub fn with_nodes(positions: Vec<Point>, board_size: usize) -> Self {
        let initial_nodes = positions.len();
        let nodes = positions
            .into_iter()
            .enumerate()
            .map(|(i, position)| Node::new(i, position))
            .collect();
        Self {
            nodes,
            lines: Vec::new(),
            face: FaceModel::new(initial_nodes),
            move_history: Vec::new(),
            current_player: Player::Human,
            next_node_id: initial_nodes,
            next_line_id: 0,
            initial_node_count: initial_nodes,
            board_size,
        }
    }

    pub fn find_node(&self, id: usize) -> Option<&Node> {
        self.nodes.iter().find(|n| n.id == id)
    }

    pub fn find_node_mut(&mut self, id: usize) -> Option<&mut Node> {
        self.nodes.iter_mut().find(|n| n.id == id)
    }

    /// Fails, changing nothing, when the line cannot be placed in a face.
    pub fn apply_move(&mut self, mov: Move) -> Result<(), Rejection> {
        let topology = embedding::embed(self, &mov)?;
        self.face
            .apply(&topology)
            .expect("the embedding checked the move");

        if mov.from_node == mov.to_node {
            // Self-loop: node gets +2 connections
            if let Some(node) = self.find_node_mut(mov.from_node) {
                node.connection_count += 2;
            }
        } else {
            if let Some(from_node) = self.nodes.iter_mut().find(|n| n.id == mov.from_node) {
                from_node.connection_count += 1;
            }
            if let Some(to_node) = self.nodes.iter_mut().find(|n| n.id == mov.to_node) {
                to_node.connection_count += 1;
            }
        }

        // Add new node — starts with 2 connections (to both line endpoints)
        let new_node_id = self.next_node_id;
        self.next_node_id += 1;
        let mut new_node = Node::new(new_node_id, mov.new_node_pos);
        new_node.connection_count = 2;
        self.nodes.push(new_node);

        // Add line
        let line = Line {
            id: self.next_line_id,
            from_node: mov.from_node,
            to_node: mov.to_node,
            polyline: mov.polyline.clone(),
            segments: mov.segments.clone(),
            flat: mov.flatten(curve::tolerance(self.board_size)),
            new_node_pos: mov.new_node_pos,
            player: mov.player,
        };
        self.next_line_id += 1;
        self.lines.push(line);

        // Record move
        self.move_history.push(mov);

        // Switch player
        self.current_player = match self.current_player {
            Player::Human => Player::AI,
            Player::AI => Player::Human,
        };

        Ok(())
    }

    /// O(1) undo: directly reverse the last move instead of replaying history.
    pub fn undo_move(&mut self) -> bool {
        let mov = match self.move_history.pop() {
            Some(m) => m,
            None => return false,
        };
        self.face.undo();

        // Remove the created node (always the last one added)
        self.nodes.pop();
        self.next_node_id -= 1;

        // Remove the line (always the last one added)
        self.lines.pop();
        self.next_line_id -= 1;

        // Restore connection counts
        if mov.from_node == mov.to_node {
            if let Some(node) = self.find_node_mut(mov.from_node) {
                node.connection_count -= 2;
            }
        } else {
            if let Some(node) = self.find_node_mut(mov.from_node) {
                node.connection_count -= 1;
            }
            if let Some(node) = self.find_node_mut(mov.to_node) {
                node.connection_count -= 1;
            }
        }

        // Switch player back
        self.current_player = match self.current_player {
            Player::Human => Player::AI,
            Player::AI => Player::Human,
        };

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_move(from: usize, to: usize) -> Move {
        Move::new(
            from,
            to,
            vec![
                Point::new(100.0, 100.0),
                Point::new(150.0, 150.0),
                Point::new(200.0, 200.0),
            ],
            Point::new(150.0, 150.0),
            Player::Human,
            BOARD_SIZE,
        )
    }

    /// Placing a node on a curve and placing it again where it landed
    /// gives the same point: a node already on the curve is kept.
    #[test]
    fn a_node_placed_on_its_curve_stays_put_when_placed_again() {
        let polyline: Vec<Point> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                Point::new(100.0 + 400.0 * t, 300.0 + 120.0 * (t * 3.0).sin())
            })
            .collect();
        let first = Move::new(
            0,
            1,
            polyline.clone(),
            Point::new(250.0, 450.0),
            Player::AI,
            1000,
        );
        assert!(first.new_node_pos.distance_to(&Point::new(250.0, 450.0)) > 1.0);
        let again = Move::new(0, 1, polyline, first.new_node_pos, Player::AI, 1000);
        assert_eq!(again.new_node_pos, first.new_node_pos);
        assert_eq!(again.segments, first.segments);
        let refit = Move::from_segments(
            0,
            1,
            again.polyline.clone(),
            again.segments.clone(),
            again.new_node_pos,
            Player::AI,
        );
        assert_eq!(refit.new_node_pos, first.new_node_pos);
    }

    // --- GameState::new ---

    #[test]
    fn new_correct_node_count() {
        let gs = GameState::new(3);
        assert_eq!(gs.nodes.len(), 3);
        assert_eq!(gs.initial_node_count, 3);
        assert_eq!(gs.next_node_id, 3);
    }

    #[test]
    fn new_all_zero_connections() {
        let gs = GameState::new(4);
        for node in &gs.nodes {
            assert_eq!(node.connection_count, 0);
        }
    }

    #[test]
    fn the_last_node_of_the_layout_reaches_the_set_fraction_of_the_board() {
        for board in [500, 1000] {
            let centre = Point::new(board as f64 / 2.0, board as f64 / 2.0);
            for count in 1..=24 {
                let layout = opening_layout(count, board);
                let reach = layout.last().unwrap().distance_to(&centre);
                assert!((reach - LAYOUT_REACH * board as f64).abs() < 1e-6);
                let widest = layout
                    .iter()
                    .map(|p| p.distance_to(&centre))
                    .fold(0.0, f64::max);
                assert_eq!(widest, reach);
            }
        }
    }

    #[test]
    fn layout_neighbours_keep_four_node_spacings_apart_up_to_24_nodes() {
        for board in [500, 1000] {
            let floor = 4.0 * MIN_NODE_SPACING * scale(board);
            for count in 2..=24 {
                let layout = opening_layout(count, board);
                let mut closest = f64::INFINITY;
                for (i, a) in layout.iter().enumerate() {
                    for b in &layout[i + 1..] {
                        closest = closest.min(a.distance_to(b));
                    }
                }
                assert!(closest >= floor, "{count} nodes on {board}: {closest:.1}");
            }
        }
    }

    #[test]
    fn with_board_size_lays_out_the_scaled_sunflower() {
        let gs = GameState::with_board_size(2, 500);
        assert_eq!(gs.board_size, 500);
        for (node, expected) in gs.nodes.iter().zip(opening_layout(2, 500)) {
            assert_eq!(node.position, expected);
            assert!(node.position.x > 0.0 && node.position.x < 500.0);
            assert!(node.position.y > 0.0 && node.position.y < 500.0);
        }
        let full = GameState::with_board_size(2, 1000);
        for (half, whole) in gs.nodes.iter().zip(&full.nodes) {
            assert!((half.position.x * 2.0 - whole.position.x).abs() < 1e-6);
            assert!((half.position.y * 2.0 - whole.position.y).abs() < 1e-6);
        }
    }

    #[test]
    fn with_nodes_takes_the_positions_given() {
        let positions = vec![Point::new(100.0, 100.0), Point::new(300.0, 120.0)];
        let gs = GameState::with_nodes(positions.clone(), 500);
        assert_eq!(gs.initial_node_count, 2);
        assert_eq!(gs.next_node_id, 2);
        assert_eq!(gs.nodes[1].position, positions[1]);
        assert_eq!(gs.nodes[1].id, 1);
    }

    // --- apply_move ---

    #[test]
    fn apply_move_increments_connections() {
        let mut gs = GameState::new(2);
        let mov = make_move(0, 1);
        gs.apply_move(mov).unwrap();
        assert_eq!(gs.find_node(0).unwrap().connection_count, 1);
        assert_eq!(gs.find_node(1).unwrap().connection_count, 1);
    }

    #[test]
    fn apply_move_adds_node_with_2_connections() {
        let mut gs = GameState::new(2);
        let mov = make_move(0, 1);
        gs.apply_move(mov).unwrap();
        assert_eq!(gs.nodes.len(), 3);
        let new_node = gs.find_node(2).unwrap();
        assert_eq!(new_node.connection_count, 2);
    }

    #[test]
    fn apply_move_adds_line() {
        let mut gs = GameState::new(2);
        let mov = make_move(0, 1);
        gs.apply_move(mov).unwrap();
        assert_eq!(gs.lines.len(), 1);
        assert_eq!(gs.lines[0].from_node, 0);
        assert_eq!(gs.lines[0].to_node, 1);
    }

    #[test]
    fn apply_move_switches_player() {
        let mut gs = GameState::new(2);
        assert_eq!(gs.current_player, Player::Human);
        gs.apply_move(make_move(0, 1)).unwrap();
        assert_eq!(gs.current_player, Player::AI);
    }

    #[test]
    fn apply_move_self_loop_adds_2() {
        let mut gs = GameState::new(2);
        let mov = Move::new(
            0,
            0,
            vec![
                Point::new(100.0, 100.0),
                Point::new(200.0, 200.0),
                Point::new(100.0, 100.0),
            ],
            Point::new(150.0, 150.0),
            Player::Human,
            BOARD_SIZE,
        );
        gs.apply_move(mov).unwrap();
        assert_eq!(gs.find_node(0).unwrap().connection_count, 2);
    }

    // --- undo_move ---

    #[test]
    fn undo_move_reverses_apply() {
        let mut gs = GameState::new(2);
        let orig_nodes: Vec<(usize, u8)> = gs
            .nodes
            .iter()
            .map(|n| (n.id, n.connection_count))
            .collect();

        gs.apply_move(make_move(0, 1)).unwrap();
        assert_eq!(gs.nodes.len(), 3);

        let undone = gs.undo_move();
        assert!(undone);
        assert_eq!(gs.face, FaceModel::new(2));
        assert_eq!(gs.nodes.len(), 2);
        assert_eq!(gs.lines.len(), 0);
        assert_eq!(gs.current_player, Player::Human);

        for (id, cc) in &orig_nodes {
            assert_eq!(gs.find_node(*id).unwrap().connection_count, *cc);
        }
    }

    #[test]
    fn undo_move_empty_history_returns_false() {
        let mut gs = GameState::new(2);
        assert!(!gs.undo_move());
    }

    #[test]
    fn repeated_apply_undo_identity() {
        let mut gs = GameState::new(3);
        let snapshot: Vec<(usize, u8)> = gs
            .nodes
            .iter()
            .map(|n| (n.id, n.connection_count))
            .collect();

        for _ in 0..5 {
            gs.apply_move(make_move(0, 1)).unwrap();
            gs.undo_move();
        }

        assert_eq!(gs.nodes.len(), snapshot.len());
        assert_eq!(gs.lines.len(), 0);
        assert_eq!(gs.current_player, Player::Human);
        for (id, cc) in &snapshot {
            assert_eq!(gs.find_node(*id).unwrap().connection_count, *cc);
        }
    }
}
