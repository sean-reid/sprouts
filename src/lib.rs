mod abstract_graph;
mod ai;
mod components;
pub mod face;
mod game_over;
mod geometry;
mod morphology;
mod node_classifier;
mod opening_book;
mod pathfinding;
mod protocol;
mod types;
mod validation;

use protocol::{Move, Outcome, Placement, PlacementQuery, Snapshot};
use tsify::{Ts, Tsify};
use types::GameState;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct SproutsGame {
    state: GameState,
}

#[wasm_bindgen]
impl SproutsGame {
    #[wasm_bindgen(constructor)]
    pub fn new(initial_nodes: usize, board_size: usize) -> Self {
        Self {
            state: GameState::with_board_size(initial_nodes, board_size),
        }
    }

    pub fn snapshot(&mut self) -> Result<Ts<Snapshot>, JsError> {
        Ok(protocol::snapshot(&mut self.state).into_ts()?)
    }

    #[wasm_bindgen(js_name = applyMove)]
    pub fn apply_move(&mut self, mov: Ts<Move>) -> Result<Ts<Outcome>, JsError> {
        Ok(protocol::apply(&mut self.state, mov.to_rust()?).into_ts()?)
    }

    pub fn undo(&mut self) -> bool {
        self.state.undo_move()
    }

    #[wasm_bindgen(js_name = aiMove)]
    pub fn ai_move(&mut self) -> Result<Option<Ts<Move>>, JsError> {
        Ok(protocol::ai_move(&mut self.state)
            .map(|mov| mov.into_ts())
            .transpose()?)
    }

    pub fn placement(&self, query: Ts<PlacementQuery>) -> Result<Ts<Placement>, JsError> {
        Ok(protocol::placement(&self.state, query.to_rust()?).into_ts()?)
    }
}
