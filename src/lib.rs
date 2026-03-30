mod abstract_graph;
mod ai;
mod components;
mod geometry;
mod morphology;
mod node_classifier;
mod opening_book;
mod pathfinding;
mod types;
mod validation;

use wasm_bindgen::prelude::*;
use types::{GameState, Move, Player, Point};

fn parse_polyline(path_data: &[f64]) -> Vec<Point> {
    path_data
        .chunks_exact(2)
        .map(|c| Point::new(c[0], c[1]))
        .collect()
}

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

    #[wasm_bindgen]
    pub fn new_with_board_size(initial_nodes: usize, board_size: usize) -> Self {
        Self {
            state: GameState::with_board_size(initial_nodes, board_size),
        }
    }

    /// Get current game state as Float64Array.
    /// Format: [node_count, ...nodes, line_count, ...lines, current_player]
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

        // Current player (0 = human, 1 = AI)
        data.push(match self.state.current_player {
            Player::Human => 0.0,
            Player::AI => 1.0,
        });

        data
    }

    #[wasm_bindgen]
    pub fn get_active_nodes(&mut self) -> Vec<f64> {
        let classification = node_classifier::classify_nodes(&mut self.state);
        classification
            .active_nodes
            .into_iter()
            .map(|id| id as f64)
            .collect()
    }

    #[wasm_bindgen]
    pub fn get_move_error(
        &self,
        from_node: usize,
        to_node: usize,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> String {
        let mov = Move {
            from_node,
            to_node,
            polyline: parse_polyline(&path_data),
            new_node_pos: Point::new(new_node_x, new_node_y),
            player: self.state.current_player,
        };
        match validation::validate_move(&self.state, &mov) {
            Ok(_) => "Valid".to_string(),
            Err(e) => e,
        }
    }

    #[wasm_bindgen]
    pub fn apply_move(
        &mut self,
        from_node: usize,
        to_node: usize,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> bool {
        let mov = Move {
            from_node,
            to_node,
            polyline: parse_polyline(&path_data),
            new_node_pos: Point::new(new_node_x, new_node_y),
            player: self.state.current_player,
        };
        // Use AI-level validation when it's the AI's turn, human-level otherwise
        let valid = match self.state.current_player {
            Player::AI => validation::validate_ai_move(&self.state, &mov).is_ok(),
            Player::Human => validation::validate_move(&self.state, &mov).is_ok(),
        };
        if valid {
            self.state.apply_move(mov);
            true
        } else {
            false
        }
    }

    #[wasm_bindgen]
    pub fn undo(&mut self) -> bool {
        self.state.undo_move()
    }

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

    #[wasm_bindgen]
    pub fn is_game_over(&mut self) -> bool {
        let classification = node_classifier::classify_nodes(&mut self.state);
        if classification.active_nodes.is_empty() {
            return true;
        }
        if classification.legal_pairs.is_empty()
            && classification.self_loop_candidates.is_empty()
        {
            return true;
        }

        // The classifier says moves exist — but it only checks spatial
        // connectivity, not whether a valid path can actually be drawn.
        // Build actual candidate moves and validate them to be sure.
        for (from_id, to_id) in &classification.legal_pairs {
            let paths: [Option<Vec<types::Point>>; 3] = [
                pathfinding::find_path_on_skeleton(&self.state, *from_id, *to_id),
                pathfinding::find_path_relaxed(&self.state, *from_id, *to_id),
                pathfinding::find_path_tight(&self.state, *from_id, *to_id),
            ];
            for path in paths.into_iter().flatten() {
                if path.len() < 3 { continue; }
                let new_node_pos = ai::find_optimal_node_placement(&path, &self.state);
                let mov = types::Move {
                    from_node: *from_id,
                    to_node: *to_id,
                    polyline: path,
                    new_node_pos,
                    player: self.state.current_player,
                };
                if validation::validate_ai_move(&self.state, &mov).is_ok()
                    || validation::validate_ai_move_tight(&self.state, &mov).is_ok()
                {
                    return false;
                }
            }
        }
        for &node_id in &classification.self_loop_candidates {
            let paths: [Option<Vec<types::Point>>; 2] = [
                pathfinding::find_self_loop_on_skeleton(&self.state, node_id),
                pathfinding::generate_geometric_self_loop(&self.state, node_id),
            ];
            for path in paths.into_iter().flatten() {
                if path.len() < 5 { continue; }
                let new_node_pos = ai::find_optimal_node_placement(&path, &self.state);
                let mov = types::Move {
                    from_node: node_id,
                    to_node: node_id,
                    polyline: path,
                    new_node_pos,
                    player: self.state.current_player,
                };
                if validation::validate_ai_move(&self.state, &mov).is_ok()
                    || validation::validate_ai_move_tight(&self.state, &mov).is_ok()
                {
                    return false;
                }
            }
        }
        // No validated move found — game is over
        true
    }

    #[wasm_bindgen]
    pub fn get_closest_point_on_path(&self, path_data: Vec<f64>, target_x: f64, target_y: f64) -> Vec<f64> {
        let polyline = parse_polyline(&path_data);
        let target = Point::new(target_x, target_y);
        let (closest, _) = geometry::closest_point_on_polyline(&polyline, &target);
        vec![closest.x, closest.y]
    }

    #[wasm_bindgen]
    pub fn validate_placement(
        &self,
        path_data: Vec<f64>,
        new_node_x: f64,
        new_node_y: f64,
    ) -> bool {
        let polyline = parse_polyline(&path_data);
        let new_node_pos = Point::new(new_node_x, new_node_y);
        validation::validate_new_node_placement(&polyline, &new_node_pos, &self.state.nodes, self.state.board_size).is_ok()
    }

}
