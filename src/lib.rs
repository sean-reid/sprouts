mod abstract_graph;
mod ai;
mod curve;
mod embedding;
pub mod face;
mod geometry;
mod nimber;
mod protocol;
#[cfg(feature = "bench")]
pub use abstract_graph::search_rate;
#[cfg(any(test, feature = "bench"))]
pub mod replay;
mod route;
mod types;
mod validation;

use protocol::{
    AiReply, AutoPlacement, AutoPlacementQuery, Curve, FaceMove, FitQuery, Layout, Move, MovesFrom,
    Outcome, Placement, PlacementQuery, RouteReply, Snapshot,
};
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
        layout: Option<Ts<Layout>>,
    ) -> Result<SproutsGame, JsError> {
        let nodes = layout.map(|l| l.to_rust()).transpose()?.map(|l| l.nodes);
        let state = protocol::start(initial_nodes, board_size, first_player.to_rust()?, nodes)
            .map_err(|e| JsError::new(&e))?;
        Ok(Self { state })
    }

    pub fn snapshot(&self) -> Result<Ts<Snapshot>, JsError> {
        Ok(protocol::snapshot(&self.state).into_ts()?)
    }

    #[wasm_bindgen(js_name = applyMove)]
    pub fn apply_move(&mut self, mov: Ts<Move>) -> Result<Ts<Outcome>, JsError> {
        Ok(protocol::apply(&mut self.state, mov.to_rust()?).into_ts()?)
    }

    #[wasm_bindgen(js_name = applyFaceMove)]
    pub fn apply_face_move(&mut self, mov: Ts<FaceMove>) -> Result<Ts<Outcome>, JsError> {
        Ok(protocol::apply_face_move(&mut self.state, mov.to_rust()?).into_ts()?)
    }

    pub fn undo(&mut self) -> bool {
        self.state.undo_move()
    }

    #[wasm_bindgen(js_name = aiMove)]
    pub fn ai_move(&mut self) -> Result<Ts<AiReply>, JsError> {
        Ok(protocol::ai_move(&mut self.state).into_ts()?)
    }

    #[wasm_bindgen(js_name = movesFrom)]
    pub fn moves_from(&self, node: usize) -> Result<Ts<MovesFrom>, JsError> {
        Ok(protocol::moves_from(&self.state, node).into_ts()?)
    }

    #[wasm_bindgen(js_name = routeMove)]
    pub fn route_move(&self, mov: Ts<FaceMove>) -> Result<Ts<RouteReply>, JsError> {
        Ok(protocol::route_move(&self.state, mov.to_rust()?).into_ts()?)
    }

    pub fn fit(&self, query: Ts<FitQuery>) -> Result<Ts<Curve>, JsError> {
        Ok(protocol::fit(&self.state, query.to_rust()?).into_ts()?)
    }

    pub fn placement(&self, query: Ts<PlacementQuery>) -> Result<Ts<Placement>, JsError> {
        Ok(protocol::placement(&self.state, query.to_rust()?).into_ts()?)
    }

    #[wasm_bindgen(js_name = autoPlacement)]
    pub fn auto_placement(
        &self,
        query: Ts<AutoPlacementQuery>,
    ) -> Result<Ts<AutoPlacement>, JsError> {
        Ok(protocol::auto_placement(&self.state, query.to_rust()?).into_ts()?)
    }
}
