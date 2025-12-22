mod ai;
mod components;
mod geometry;
mod morphology;
mod node_classifier;
mod pathfinding;
mod spatial_index;
mod types;
mod validation;

use wasm_bindgen::prelude::*;
use types::{GameState, Move, Player, Point};

#[wasm_bindgen]
pub struct SproutsGame {
    state: GameState,
}

#[wasm_bindgen]
impl SproutsGame {
    #[wasm_bindgen(constructor)]
    pub fn new(initial_nodes: usize) -> Self {
        Self {
            state: GameState::new(initial_nodes),
        }
    }

    /// Get current game state as Float64Array
    /// Format: [node_count, ...nodes (id, x, y, conn_count), line_count, ...lines (id, from, to, point_count, ...points, new_x, new_y, player)]
    #[wasm_bindgen]
    pub fn get_state(&self) -> Vec<f64> {
        let mut data = Vec::new();

        // Nodes
        data.push(self.state.nodes.len() as f64);
        for node in &self.state.nodes {
            data.push(node.id as f64);
            data.push(node.position.x);
            data.push(node.position.y);
            data.push(node.connection_count as f64);
        }

        // Lines
        data.push(self.state.lines.len() as f64);
        for line in &self.state.lines {
            data.push(line.id as f64);
            data.push(line.from_node as f64);
            data.push(line.to_node as f64);
            data.push(line.polyline.len() as f64);
            for point in &line.polyline {
                data.push(point.x);
                data.push(point.y);
            }
            data.push(line.new_node_pos.x);
            data.push(line.new_node_pos.y);
            data.push(match line.player {
                Player::Human => 0.0,
                Player::AI => 1.0,
            });
        }

        data
    }

    /// Get active nodes
    #[wasm_bindgen]
    pub fn get_active_nodes(&mut self) -> Vec<f64> {
        let classification = node_classifier::classify_nodes(&mut self.state);
        classification
            .active_nodes
            .into_iter()
            .map(|id| id as f64)
            .collect()
    }

    /// Check if a move is valid
    #[wasm_bindgen]
    pub fn validate_move_js(
        &self,
        from_node: usize,
        to_node: usize,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> bool {
        let mut polyline = Vec::new();
        for i in (0..path_data.len()).step_by(2) {
            polyline.push(Point::new(path_data[i], path_data[i + 1]));
        }

        let mov = Move {
            from_node,
            to_node,
            polyline,
            new_node_pos: Point::new(new_node_x, new_node_y),
            player: self.state.current_player,
        };

        validation::validate_move(&self.state, &mov).is_ok()
    }

    /// Get the validation error message for a move
    #[wasm_bindgen]
    pub fn get_move_error(
        &self,
        from_node: usize,
        to_node: usize,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> String {
        let mut polyline = Vec::new();
        for i in (0..path_data.len()).step_by(2) {
            polyline.push(Point::new(path_data[i], path_data[i + 1]));
        }

        let mov = Move {
            from_node,
            to_node,
            polyline,
            new_node_pos: Point::new(new_node_x, new_node_y),
            player: self.state.current_player,
        };

        match validation::validate_move(&self.state, &mov) {
            Ok(_) => "Valid".to_string(),
            Err(e) => e,
        }
    }

    /// Apply a move
    #[wasm_bindgen]
    pub fn apply_move(
        &mut self,
        from_node: usize,
        to_node: usize,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> bool {
        let mut polyline = Vec::new();
        for i in (0..path_data.len()).step_by(2) {
            polyline.push(Point::new(path_data[i], path_data[i + 1]));
        }

        let mov = Move {
            from_node,
            to_node,
            polyline,
            new_node_pos: Point::new(new_node_x, new_node_y),
            player: self.state.current_player,
        };

        if validation::validate_move(&self.state, &mov).is_ok() {
            self.state.apply_move(mov);
            true
        } else {
            false
        }
    }

    /// Get AI move
    #[wasm_bindgen]
    pub fn get_ai_move(&mut self) -> Vec<f64> {
        if let Some(mov) = ai::select_ai_move(&mut self.state) {
            let mut data = Vec::new();
            data.push(mov.from_node as f64);
            data.push(mov.to_node as f64);
            data.push(mov.polyline.len() as f64);
            for point in &mov.polyline {
                data.push(point.x);
                data.push(point.y);
            }
            data.push(mov.new_node_pos.x);
            data.push(mov.new_node_pos.y);
            data
        } else {
            vec![]
        }
    }

    /// Check if game is over
    #[wasm_bindgen]
    pub fn is_game_over(&mut self) -> bool {
        let classification = node_classifier::classify_nodes(&mut self.state);
        classification.active_nodes.is_empty()
            || (classification.legal_pairs.is_empty()
                && classification.self_loop_candidates.is_empty())
    }

    /// Get closest point on path
    #[wasm_bindgen]
    pub fn get_closest_point_on_path(&self, path_data: Vec<f64>, target_x: f64, target_y: f64) -> Vec<f64> {
        let mut polyline = Vec::new();
        for i in (0..path_data.len()).step_by(2) {
            polyline.push(Point::new(path_data[i], path_data[i + 1]));
        }

        let target = Point::new(target_x, target_y);
        let (closest, _) = geometry::closest_point_on_polyline(&polyline, &target);

        vec![closest.x, closest.y]
    }

    /// Validate new node placement
    #[wasm_bindgen]
    pub fn validate_placement(
        &self,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> bool {
        let mut polyline = Vec::new();
        for i in (0..path_data.len()).step_by(2) {
            polyline.push(Point::new(path_data[i], path_data[i + 1]));
        }

        let new_node_pos = Point::new(new_node_x, new_node_y);

        validation::validate_new_node_placement(&polyline, &new_node_pos, &self.state.nodes).is_ok()
    }

    /// Get debug skeleton data
    #[wasm_bindgen]
    pub fn get_skeleton_debug(&mut self) -> Vec<u8> {
        // Ensure skeleton is valid
        if !self.state.skeleton_cache.is_valid {
            let (skeleton, distance_transform) = morphology::generate_skeleton(&self.state);
            self.state.skeleton_cache.skeleton = skeleton;
            self.state.skeleton_cache.distance_transform = distance_transform;
            
            let (labels, components) =
                components::label_components(&self.state.skeleton_cache.skeleton, &self.state);
            self.state.skeleton_cache.component_labels = labels;
            self.state.skeleton_cache.components = components;
            
            self.state.skeleton_cache.is_valid = true;
        }

        // Convert skeleton to RGBA image data
        let skeleton = &self.state.skeleton_cache.skeleton;
        let mut data = vec![0u8; 800 * 800 * 4];

        for y in 0..800 {
            for x in 0..800 {
                let idx = (y * 800 + x) * 4;
                if skeleton.data[y * 800 + x] {
                    // White skeleton
                    data[idx] = 255;
                    data[idx + 1] = 255;
                    data[idx + 2] = 255;
                    data[idx + 3] = 255;
                } else {
                    // Transparent
                    data[idx + 3] = 0;
                }
            }
        }

        data
    }

    /// Get classification debug info
    #[wasm_bindgen]
    pub fn get_classification_debug(&mut self) -> Vec<f64> {
        let classification = node_classifier::classify_nodes(&mut self.state);
        
        let mut data = Vec::new();
        
        // Active nodes count
        data.push(classification.active_nodes.len() as f64);
        for node_id in &classification.active_nodes {
            data.push(*node_id as f64);
        }
        
        // Legal pairs count
        data.push(classification.legal_pairs.len() as f64);
        for (from, to) in &classification.legal_pairs {
            data.push(*from as f64);
            data.push(*to as f64);
        }
        
        // Self-loop candidates count
        data.push(classification.self_loop_candidates.len() as f64);
        for node_id in &classification.self_loop_candidates {
            data.push(*node_id as f64);
        }
        
        data
    }

    /// Test a specific pair and return why it fails
    #[wasm_bindgen]
    pub fn test_pair(&mut self, from_node: usize, to_node: usize) -> String {
        // Try to find path
        match pathfinding::find_path_on_skeleton(&self.state, from_node, to_node) {
            Some(path) => {
                if path.len() < 2 {
                    return format!("Path too short: {} points", path.len());
                }
                
                // Find placement
                let new_node_pos = ai::find_optimal_node_placement(&path, &self.state.nodes);
                
                // Check placement distance
                let mut min_node_dist = f64::INFINITY;
                for node in &self.state.nodes {
                    let dx = new_node_pos.x - node.position.x;
                    let dy = new_node_pos.y - node.position.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    min_node_dist = min_node_dist.min(dist);
                }
                
                let mov = types::Move {
                    from_node,
                    to_node,
                    polyline: path.clone(),
                    new_node_pos,
                    player: self.state.current_player,
                };
                
                match validation::validate_ai_move(&self.state, &mov) {
                    Ok(_) => format!("Valid! (path: {} pts, placement dist: {:.1}px)", path.len(), min_node_dist),
                    Err(e) => format!("Failed: {} (path: {} pts, placement dist: {:.1}px)", e, path.len(), min_node_dist),
                }
            }
            None => "No path found on skeleton".to_string(),
        }
    }

    /// Find path between two nodes
    #[wasm_bindgen]
    pub fn find_path(&mut self, from_node: usize, to_node: usize) -> Vec<f64> {
        if let Some(path) = pathfinding::find_path_on_skeleton(&self.state, from_node, to_node) {
            let mut data = Vec::new();
            for point in path {
                data.push(point.x);
                data.push(point.y);
            }
            data
        } else {
            vec![]
        }
    }
}
