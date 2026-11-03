use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
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

// Morphology / pathfinding
pub const LINE_RASTER_WIDTH: f64 = 3.0;
pub const NODE_PROTECTION_RADIUS: i32 = 6;
pub const MAX_ASTAR_ITERATIONS: usize = 300_000;

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
            Point::new(centre + radius * angle.cos(), centre + radius * angle.sin())
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PixelCoord {
    pub x: i32,
    pub y: i32,
}

impl PixelCoord {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
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
            position,
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
        let new_node_pos =
            curve::nearest(&segments, &new_node_pos).map_or(new_node_pos, |hit| hit.point);
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
    pub skeleton_cache: SkeletonCache,
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
            skeleton_cache: SkeletonCache::new_with_size(board_size),
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

    pub fn lines_connected_to(&self, node_id: usize) -> Vec<&Line> {
        self.lines
            .iter()
            .filter(|line| line.from_node == node_id || line.to_node == node_id)
            .collect()
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

        self.skeleton_cache.invalidate();
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

        self.skeleton_cache.invalidate();
        true
    }
}

#[derive(Clone)]
pub struct SkeletonCache {
    pub skeleton: Grid<bool>,
    pub distance_transform: Grid<u8>,
    pub component_labels: Grid<u32>, // 0 = not skeleton, 1+ = component ID
    pub components: HashMap<usize, ComponentMetadata>,
    pub is_valid: bool,
}

impl SkeletonCache {
    pub fn new_with_size(board_size: usize) -> Self {
        Self {
            skeleton: Grid::new(board_size, board_size, false),
            distance_transform: Grid::new(board_size, board_size, 0),
            component_labels: Grid::new(board_size, board_size, 0),
            components: HashMap::new(),
            is_valid: false,
        }
    }

    pub fn invalidate(&mut self) {
        self.is_valid = false;
    }
}

#[derive(Debug, Clone)]
pub struct ComponentMetadata {
    pub pixel_count: usize,
    pub accessible_nodes: HashSet<usize>,
    pub bounding_box: BoundingBox,
    pub max_internal_distance: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct BoundingBox {
    pub min_x: i32,
    pub min_y: i32,
    pub max_x: i32,
    pub max_y: i32,
}

impl BoundingBox {
    pub fn new() -> Self {
        Self {
            min_x: i32::MAX,
            min_y: i32::MAX,
            max_x: i32::MIN,
            max_y: i32::MIN,
        }
    }

    pub fn update(&mut self, x: i32, y: i32) {
        self.min_x = self.min_x.min(x);
        self.min_y = self.min_y.min(y);
        self.max_x = self.max_x.max(x);
        self.max_y = self.max_y.max(y);
    }
}

#[derive(Clone)]
pub struct Grid<T> {
    pub width: usize,
    pub height: usize,
    pub data: Vec<T>,
}

impl<T: Clone> Grid<T> {
    pub fn new(width: usize, height: usize, default: T) -> Self {
        Self {
            width,
            height,
            data: vec![default; width * height],
        }
    }

    pub fn get(&self, x: i32, y: i32) -> Option<&T> {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            None
        } else {
            Some(&self.data[y as usize * self.width + x as usize])
        }
    }

    pub fn get_mut(&mut self, x: i32, y: i32) -> Option<&mut T> {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            None
        } else {
            Some(&mut self.data[y as usize * self.width + x as usize])
        }
    }

    pub fn set(&mut self, x: i32, y: i32, value: T) {
        if let Some(cell) = self.get_mut(x, y) {
            *cell = value;
        }
    }

    pub fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.width as i32 && y < self.height as i32
    }
}

impl Grid<bool> {
    pub fn invert(&self) -> Grid<bool> {
        let mut result = Grid::new(self.width, self.height, false);
        for i in 0..self.data.len() {
            result.data[i] = !self.data[i];
        }
        result
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

    // --- Grid ---

    #[test]
    fn grid_get_set() {
        let mut g: Grid<i32> = Grid::new(10, 10, 0);
        g.set(3, 4, 42);
        assert_eq!(*g.get(3, 4).unwrap(), 42);
        assert_eq!(*g.get(0, 0).unwrap(), 0);
    }

    #[test]
    fn grid_in_bounds() {
        let g: Grid<bool> = Grid::new(5, 5, false);
        assert!(g.in_bounds(0, 0));
        assert!(g.in_bounds(4, 4));
        assert!(!g.in_bounds(-1, 0));
        assert!(!g.in_bounds(5, 0));
        assert!(!g.in_bounds(0, 5));
    }

    #[test]
    fn grid_out_of_bounds_returns_none() {
        let g: Grid<i32> = Grid::new(5, 5, 0);
        assert!(g.get(-1, 0).is_none());
        assert!(g.get(5, 0).is_none());
    }

    #[test]
    fn grid_invert() {
        let mut g: Grid<bool> = Grid::new(3, 3, false);
        g.set(1, 1, true);
        let inv = g.invert();
        assert!(*inv.get(0, 0).unwrap());
        assert!(!*inv.get(1, 1).unwrap());
    }
}
