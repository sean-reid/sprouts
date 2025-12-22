use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone, Copy, PartialEq)]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Player {
    Human,
    AI,
}

#[derive(Debug, Clone)]
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

    pub fn is_active(&self) -> bool {
        self.connection_count < 3
    }
}

#[derive(Debug, Clone)]
pub struct Line {
    pub id: usize,
    pub from_node: usize,
    pub to_node: usize,
    pub polyline: Vec<Point>,
    pub new_node_pos: Point,
    pub player: Player,
}

#[derive(Debug, Clone)]
pub struct Move {
    pub from_node: usize,
    pub to_node: usize,
    pub polyline: Vec<Point>,
    pub new_node_pos: Point,
    pub player: Player,
}

#[derive(Clone)]
pub struct GameState {
    pub nodes: Vec<Node>,
    pub lines: Vec<Line>,
    pub skeleton_cache: SkeletonCache,
    pub move_history: Vec<Move>,
    pub current_player: Player,
    pub game_over: bool,
    pub next_node_id: usize,
    pub next_line_id: usize,
}

impl GameState {
    pub fn new(initial_nodes: usize) -> Self {
        let mut nodes = Vec::new();
        
        // Place initial nodes in a circle
        let center_x = 400.0;
        let center_y = 400.0;
        let radius = 100.0;
        
        for i in 0..initial_nodes {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (initial_nodes as f64);
            let x = center_x + radius * angle.cos();
            let y = center_y + radius * angle.sin();
            nodes.push(Node::new(i, Point::new(x, y)));
        }
        
        Self {
            nodes,
            lines: Vec::new(),
            skeleton_cache: SkeletonCache::new(),
            move_history: Vec::new(),
            current_player: Player::Human,
            game_over: false,
            next_node_id: initial_nodes,
            next_line_id: 0,
        }
    }

    pub fn lines_connected_to(&self, node_id: usize) -> Vec<&Line> {
        self.lines
            .iter()
            .filter(|line| line.from_node == node_id || line.to_node == node_id)
            .collect()
    }

    pub fn apply_move(&mut self, mov: Move) {
        // Increment connection counts
        if let Some(from_node) = self.nodes.iter_mut().find(|n| n.id == mov.from_node) {
            from_node.connection_count += 1;
        }
        if let Some(to_node) = self.nodes.iter_mut().find(|n| n.id == mov.to_node) {
            to_node.connection_count += 1;
        }

        // Add new node - CRITICAL: starts with 2 connections (to both line endpoints)
        let new_node_id = self.next_node_id;
        self.next_node_id += 1;
        let mut new_node = Node::new(new_node_id, mov.new_node_pos);
        new_node.connection_count = 2; // Connected to both ends of the line
        self.nodes.push(new_node);

        // Add line
        let line = Line {
            id: self.next_line_id,
            from_node: mov.from_node,
            to_node: mov.to_node,
            polyline: mov.polyline.clone(),
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

        // Invalidate skeleton cache
        self.skeleton_cache.invalidate();
    }
}

#[derive(Clone)]
pub struct SkeletonCache {
    pub skeleton: Grid<bool>,
    pub distance_transform: Grid<u8>,
    pub component_labels: HashMap<PixelCoord, usize>,
    pub components: HashMap<usize, ComponentMetadata>,
    pub is_valid: bool,
}

impl SkeletonCache {
    pub fn new() -> Self {
        Self {
            skeleton: Grid::new(800, 800, false),
            distance_transform: Grid::new(800, 800, 0),
            component_labels: HashMap::new(),
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
