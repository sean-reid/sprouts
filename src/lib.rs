mod abstract_graph;
mod ai;
mod components;
mod curve;
pub mod face;
mod game_over;
mod geometry;
mod morphology;
mod node_classifier;
mod opening_book;
mod pathfinding;
mod protocol;
#[cfg(feature = "bench")]
pub mod replay;
mod types;
mod validation;

use protocol::{AiMove, Curve, FitQuery, Move, Outcome, Placement, PlacementQuery, Snapshot};
use tsify::{Ts, Tsify};
use types::{GameState, Player};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct SproutsGame {
    state: GameState,
}

#[wasm_bindgen]
impl SproutsGame {
    #[wasm_bindgen(constructor)]
    pub fn new(
        initial_nodes: usize,
        board_size: usize,
        first_player: Ts<Player>,
    ) -> Result<SproutsGame, JsError> {
        Ok(Self {
            state: protocol::start(initial_nodes, board_size, first_player.to_rust()?),
        })
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
    pub fn ai_move(&mut self) -> Result<Option<Ts<AiMove>>, JsError> {
        Ok(protocol::ai_move(&mut self.state)
            .map(|mov| mov.into_ts())
            .transpose()?)
    }

    pub fn fit(&self, query: Ts<FitQuery>) -> Result<Ts<Curve>, JsError> {
        Ok(protocol::fit(&self.state, query.to_rust()?).into_ts()?)
    }

    pub fn placement(&self, query: Ts<PlacementQuery>) -> Result<Ts<Placement>, JsError> {
        Ok(protocol::placement(&self.state, query.to_rust()?).into_ts()?)
    }
}
