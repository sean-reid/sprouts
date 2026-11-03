use serde::{Deserialize, Serialize};
use tsify::Tsify;

use crate::ai;
use crate::curve::{self, Cubic};
use crate::embedding;
use crate::face::{self, IllegalMove, Move as ModelMove};
use crate::geometry;
use crate::route::{self, Unroutable};
use crate::settle;
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
    /// `cut_off` counts the nodes the move put on its new side: inside a
    /// loop, or on the face a splitting chord closes off.
    #[serde(rename_all = "camelCase")]
    Applied {
        snapshot: Snapshot,
        cut_off: usize,
    },
    Rejected {
        rejection: Rejection,
    },
}

/// The AI's answer: a move with the spline it commits as, so the page
/// animates the curve the board will show, and the face move the router
/// drew it from; or no move because the game is over, or no move because
/// none of the listed moves could be drawn while the game is open.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum AiReply {
    Move {
        mov: Move,
        segments: Vec<Cubic>,
        topology: FaceMove,
    },
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

/// A move named on the face model, as the page sees it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct FaceMove {
    pub a: usize,
    pub at_a: Option<usize>,
    pub b: usize,
    pub at_b: Option<usize>,
    pub enclosed: Vec<usize>,
}

impl From<ModelMove> for FaceMove {
    fn from(mv: ModelMove) -> Self {
        Self {
            a: mv.a,
            at_a: mv.at_a,
            b: mv.b,
            at_b: mv.at_b,
            enclosed: mv.enclosed,
        }
    }
}

impl From<FaceMove> for ModelMove {
    fn from(mv: FaceMove) -> Self {
        Self {
            a: mv.a,
            at_a: mv.at_a,
            b: mv.b,
            at_b: mv.at_b,
            enclosed: mv.enclosed,
        }
    }
}

#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct MovesFrom {
    pub node: usize,
    pub moves: Vec<FaceMove>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub enum UnroutableKind {
    /// The model does not list the move.
    Illegal,
    /// No path or no node spot at these clearances.
    Room,
    /// A path was found but could not be drawn as one curve.
    Shape,
    Topology,
    Bug,
}

impl From<&Unroutable> for UnroutableKind {
    fn from(why: &Unroutable) -> Self {
        match why {
            Unroutable::Room | Unroutable::NoSpot => Self::Room,
            Unroutable::Shape(_) => Self::Shape,
            Unroutable::Topology => Self::Topology,
            Unroutable::Bug(_) => Self::Bug,
        }
    }
}

/// A listed move drawn for the player to move, with the spline it commits
/// as, or why the router could not draw it.
#[derive(Debug, Clone, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum RouteReply {
    Routed { mov: Move, segments: Vec<Cubic> },
    Unroutable { reason: UnroutableKind },
}

/// FNV-1a over the node positions and the line flattenings, as hex.
#[cfg(any(test, feature = "hash"))]
pub fn hash_board(state: &GameState) -> String {
    let mut fnv = crate::exact::Fnv::default();
    fnv.points(&state.nodes.iter().map(|n| n.position).collect::<Vec<_>>());
    for line in &state.lines {
        fnv.points(&line.flat);
    }
    fnv.hex()
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

#[cfg(any(test, feature = "bench"))]
pub fn check(state: &GameState, mov: &types::Move) -> Result<(), Rejection> {
    topology(state, mov).map(drop)
}

/// The face move of a line that passes every check for the player to move.
fn topology(state: &GameState, mov: &types::Move) -> Result<face::Move, Rejection> {
    match state.current_player {
        Player::AI => validation::validate_ai_move(state, mov),
        Player::Human => validation::validate_move(state, mov),
    }?;
    embedding::embed(state, mov)
}

/// The move goes in and the board settles around it, so the snapshot
/// carries the relaxed geometry.
pub fn apply(state: &mut GameState, mov: Move) -> Outcome {
    let mov = mov.by(state.current_player, state.board_size);
    match topology(state, &mov) {
        Ok(_) => commit(state, mov),
        Err(rejection) => Outcome::Rejected { rejection },
    }
}

/// Applies a line that has passed every check and settles the board
/// around it, so the snapshot carries the relaxed geometry.
fn commit(state: &mut GameState, line: types::Move) -> Outcome {
    match state.apply_move(line) {
        Ok(()) => {
            settle::settle(state);
            Outcome::Applied {
                cut_off: cut_off(&state.face),
                snapshot: snapshot(state),
            }
        }
        Err(rejection) => Outcome::Rejected { rejection },
    }
}

/// Nodes the last move set apart, read off the model after it is applied.
/// The line and the boundary it left from are a wall; a node whose faces
/// all sit with the new face is on one side, one whose faces all sit
/// with the old face on the other, and the smaller count is returned.
/// The ends and the new node are on the wall, a node beyond the boundary
/// the line attaches to is reached from neither face and counts for
/// neither side, and a move that merged two boundaries sets nothing apart.
fn cut_off(model: &face::FaceModel) -> usize {
    let half_edges = 2 * model.edge_count();
    let h_am = half_edges - 4;
    let h_bm = h_am + 3;
    let (new_face, old_face) = (model.face_of(h_am), model.face_of(h_bm));
    if new_face == old_face {
        return 0;
    }
    let mut wall = vec![false; half_edges];
    for h in model.cycle(h_am).into_iter().chain(model.cycle(h_bm)) {
        wall[h] = true;
        wall[model.twin(h)] = true;
    }
    let mut by_face = vec![Vec::new(); model.face_count()];
    for h in 0..half_edges {
        by_face[model.face_of(h)].push(h);
    }
    let side = |start: usize| {
        let mut reached = vec![false; model.face_count()];
        let mut stack = vec![start];
        reached[start] = true;
        while let Some(f) = stack.pop() {
            for &h in &by_face[f] {
                let across = model.face_of(model.twin(h));
                if !wall[h] && !reached[across] {
                    reached[across] = true;
                    stack.push(across);
                }
            }
        }
        reached
    };
    let (with_new, with_old) = (side(new_face), side(old_face));
    let m = model.node_count() - 1;
    let (a, b) = (model.origin(h_am), model.origin(h_bm));
    let (mut on_new, mut on_old) = (0, 0);
    for v in 0..model.node_count() {
        if v == a || v == b || v == m {
            continue;
        }
        let faces: Vec<usize> = if model.degree(v) == 0 {
            model.corner_face(v, None).into_iter().collect()
        } else {
            model
                .rotation(v)
                .iter()
                .map(|&h| model.face_of(h))
                .collect()
        };
        let new_side = faces.iter().any(|&f| with_new[f]);
        let old_side = faces.iter().any(|&f| with_old[f]);
        match (new_side, old_side) {
            (true, false) => on_new += 1,
            (false, true) => on_old += 1,
            _ => {}
        }
    }
    on_new.min(on_old)
}

/// Changes whenever the router draws a different curve for the same face
/// move. A share link carries it, so a page with another router refuses
/// the link instead of drawing a game the sharer never saw.
pub const ROUTER_VERSION: u8 = 2;

/// A game with its real geometry, for working out which moves the router
/// redraws. `known` holds the face move of a line already known to be
/// router output, which is then taken as read.
#[derive(Debug, Clone, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Classification {
    pub initial_nodes: usize,
    pub board_size: usize,
    pub first_player: Player,
    pub nodes: Option<Vec<Point>>,
    pub moves: Vec<Move>,
    pub known: Vec<Option<FaceMove>>,
}

/// The face move of each classified move, or none where the router draws
/// something else.
#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
pub struct Classified {
    pub topology: Vec<Option<FaceMove>>,
}

/// Replays the real history and classifies each move. A move the engine
/// refuses is an error, since the history came from it.
pub fn classify(plan: Classification) -> Result<Classified, String> {
    let mut state = start(
        plan.initial_nodes,
        plan.board_size,
        plan.first_player,
        plan.nodes,
    )?;
    let mut out = Vec::with_capacity(plan.moves.len());
    for (index, (mov, known)) in plan.moves.into_iter().zip(plan.known).enumerate() {
        let mov = mov.by(state.current_player, state.board_size);
        let face = topology(&state, &mov).map_err(|e| format!("move {index}: {}", e.message))?;
        out.push(known.or_else(|| redraws(&state, &face, &mov).then(|| face.into())));
        state
            .apply_move(mov)
            .map_err(|e| format!("move {index}: {}", e.message))?;
    }
    Ok(Classified { topology: out })
}

/// Positions whose routes are hashed together.
#[cfg(feature = "hash")]
#[derive(Debug, Clone, Deserialize, Tsify)]
pub struct RouteHashPlan {
    pub plans: Vec<Classification>,
}

/// The routing hash of the given positions, as hex: each plan replayed
/// with the AI to move, then every search move routed and its flattening
/// and node fed to one FNV-1a. The native test records the value a
/// platform must match.
#[cfg(feature = "hash")]
pub fn route_hash(plan: RouteHashPlan) -> Result<String, String> {
    let mut fnv = crate::exact::Fnv::default();
    for plan in plan.plans {
        let mut state = start(plan.initial_nodes, plan.board_size, Player::AI, plan.nodes)?;
        for (index, mov) in plan.moves.into_iter().enumerate() {
            state.current_player = Player::AI;
            state
                .apply_move(mov.by(Player::AI, state.board_size))
                .map_err(|e| format!("move {index}: {}", e.message))?;
        }
        state.current_player = Player::AI;
        route::hash_routes(&mut fnv, &state);
    }
    Ok(fnv.hex())
}

/// A move as a share link carries it: drawn, or routed from its face move
/// with its node.
#[derive(Debug, Clone, Deserialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum LinkMove {
    Drawn { mov: Move },
    Routed { face: FaceMove, at: Point },
}

/// A game as a share link carries it, for replaying before it is shared.
#[derive(Debug, Clone, Deserialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Rehearsal {
    pub router_version: u8,
    pub initial_nodes: usize,
    pub board_size: usize,
    pub first_player: Player,
    pub nodes: Option<Vec<Point>>,
    pub moves: Vec<LinkMove>,
}

/// What a rehearsal found: the first move refused, if any, and for every
/// routed move before it the node the replay placed.
#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
pub struct Rehearsed {
    pub refused: Option<usize>,
    pub nodes: Vec<Option<Point>>,
}

/// Replays a link on a fresh board the way a visitor's engine will, and
/// names the first move that is refused, if any, with the node each routed
/// move landed on. Drawn moves arrive with the link's rounded geometry, so
/// a routed move after them meets a board that differs a little from the
/// one it was drawn on. A link made for another router is an error.
pub fn rehearse(plan: Rehearsal) -> Result<Rehearsed, String> {
    if plan.router_version != ROUTER_VERSION {
        return Err(format!(
            "the link is for router version {}, this engine is {ROUTER_VERSION}",
            plan.router_version
        ));
    }
    let mut state = start(
        plan.initial_nodes,
        plan.board_size,
        plan.first_player,
        plan.nodes,
    )?;
    let mut nodes = vec![None; plan.moves.len()];
    for (index, mov) in plan.moves.into_iter().enumerate() {
        let routed = matches!(mov, LinkMove::Routed { .. });
        let outcome = match mov {
            LinkMove::Drawn { mov } => apply(&mut state, mov),
            LinkMove::Routed { face, at } => apply_face_move(&mut state, face, at),
        };
        if matches!(outcome, Outcome::Rejected { .. }) {
            return Ok(Rehearsed {
                refused: Some(index),
                nodes,
            });
        }
        if routed {
            nodes[index] = state.lines.last().map(|line| line.new_node_pos);
        }
    }
    Ok(Rehearsed {
        refused: None,
        nodes,
    })
}

/// Whether the router, given the face move, draws the curve of `mov`: the
/// flattenings stay within the fit tolerance of each other. The node is
/// carried with the move, since the best spot on a curve can jump far for
/// a small change in it.
pub(crate) fn redraws(state: &GameState, face: &face::Move, mov: &types::Move) -> bool {
    let tolerance = curve::tolerance(state.board_size);
    let Ok(drawn) = route::route(state, face) else {
        return false;
    };
    let ours = mov.flatten(tolerance);
    let theirs = drawn.flatten(tolerance);
    let within = |points: &[Point], of: &[Point]| {
        points
            .iter()
            .all(|p| geometry::closest_point_on_polyline(of, p).1 <= tolerance)
    };
    within(&ours, &theirs) && within(&theirs, &ours)
}

/// Draws a move named by its topology with the router, puts its node at
/// the point of the curve nearest `at`, and applies it as the player to
/// move.
pub fn apply_face_move(state: &mut GameState, mov: FaceMove, at: Point) -> Outcome {
    let listed: face::Move = mov.clone().into();
    let drawn = state
        .face
        .check(&listed)
        .map_err(|why| illegal(state, &listed, why))
        .and_then(|()| route::route(state, &listed).map_err(|why| unroutable(&listed, why)))
        .and_then(|routed| {
            let line = types::Move::from_segments(
                routed.from_node,
                routed.to_node,
                routed.polyline,
                routed.segments,
                at,
                routed.player,
            );
            topology(state, &line).map(|_| line)
        });
    match drawn {
        Ok(line) => commit(state, line),
        Err(rejection) => Outcome::Rejected { rejection },
    }
}

fn illegal(state: &GameState, mv: &face::Move, why: IllegalMove) -> Rejection {
    let count = state.face.node_count();
    match why {
        IllegalMove::Node if mv.a >= count => Rejection::missing_node(mv.a, "Source"),
        IllegalMove::Node => Rejection::missing_node(mv.b, "Target"),
        IllegalMove::Degree if mv.a == mv.b => Rejection::self_loop_full(mv.a),
        IllegalMove::Degree if state.face.degree(mv.a) == 3 => Rejection::node_full(mv.a, "Source"),
        IllegalMove::Degree => Rejection::node_full(mv.b, "Target"),
        IllegalMove::Corner | IllegalMove::Face | IllegalMove::Enclosed => {
            Rejection::unroutable(mv.a, "the move does not fit the position")
        }
    }
}

fn unroutable(mv: &face::Move, why: Unroutable) -> Rejection {
    match why {
        Unroutable::Room => Rejection::unroutable(mv.a, "no room"),
        Unroutable::NoSpot => Rejection::unroutable(mv.a, "no room for the new node"),
        Unroutable::Shape(what) => Rejection::unroutable(mv.a, what),
        Unroutable::Topology => Rejection::unroutable(mv.a, "the curve reads as another move"),
        Unroutable::Bug(what) => Rejection::unroutable(mv.a, what),
    }
}

/// Takes the last move back and puts the geometry back as it was before
/// that move settled, so the board is the one the next move was drawn on.
pub fn undo(state: &mut GameState) -> bool {
    let undone = state.undo_move();
    if undone {
        // Every applied move settles once, so the record on top belongs to
        // the move just taken back.
        let paired = state
            .settles
            .last()
            .is_some_and(|settled| settled.board == state.lines.len() + 1);
        debug_assert!(paired, "undo of a move that was not settled once");
        if paired {
            if let Some(settled) = state.settles.pop() {
                settled.restore(state);
            }
        }
    }
    undone
}

/// The AI's line is router output, so its face move is read straight back
/// off the curve; a curve that will not read back cannot be applied
/// either, so it is reported as undrawn rather than trapping the worker.
pub fn ai_move(state: &mut GameState) -> AiReply {
    match ai::select_ai_move(state) {
        ai::Reply::Move(mov) => match embedding::embed(state, &mov) {
            Ok(face) => AiReply::Move {
                topology: face.into(),
                segments: mov.segments.clone(),
                mov: Move::from(mov),
            },
            Err(_) => AiReply::Undrawn,
        },
        ai::Reply::GameOver => AiReply::GameOver,
        ai::Reply::Undrawn(_) => AiReply::Undrawn,
    }
}

/// Every legal move with `node` at one end. A chord between two different
/// boundary pieces has its lower node first; `enclosed` is sorted.
pub fn moves_from(state: &GameState, node: usize) -> MovesFrom {
    let face = &state.face;
    let mut moves: Vec<ModelMove> = Vec::new();
    if node < face.node_count() {
        let touches = |mv: &ModelMove| mv.a == node || mv.b == node;
        for listed in face.search_moves() {
            let members = ai::class_members_where(state, &listed, usize::MAX, touches);
            for mut mv in std::iter::once(listed).chain(members) {
                if !touches(&mv) {
                    continue;
                }
                let split = face.component_of(mv.a, mv.at_a) == face.component_of(mv.b, mv.at_b);
                if !split && mv.a > mv.b {
                    std::mem::swap(&mut mv.a, &mut mv.b);
                    std::mem::swap(&mut mv.at_a, &mut mv.at_b);
                }
                mv.enclosed.sort_unstable();
                if face.check(&mv).is_ok() && !moves.contains(&mv) {
                    moves.push(mv);
                }
            }
        }
    }
    MovesFrom {
        node,
        moves: moves.into_iter().map(FaceMove::from).collect(),
    }
}

/// Draws `mv` for the player to move without applying it.
pub fn route_move(state: &GameState, mv: FaceMove) -> RouteReply {
    let listed = ModelMove::from(mv);
    if state.face.check(&listed).is_err() {
        return RouteReply::Unroutable {
            reason: UnroutableKind::Illegal,
        };
    }
    match route::route(state, &listed) {
        Ok(mov) => RouteReply::Routed {
            segments: mov.segments.clone(),
            mov: Move::from(mov),
        },
        Err(why) => RouteReply::Unroutable {
            reason: UnroutableKind::from(&why),
        },
    }
}

/// The spline a stroke commits as, so the page can show it before the move.
pub fn fit(state: &GameState, query: FitQuery) -> Curve {
    Curve {
        segments: curve::fit(&query.path, curve::tolerance(state.board_size)),
    }
}

pub fn placement(state: &GameState, query: PlacementQuery) -> Placement {
    let Some(hit) = curve::nearest(&query.segments, &query.at).map(|hit| curve::Nearest {
        point: crate::exact::snap_to_grid(hit.point),
        ..hit
    }) else {
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
        .map(|hit| crate::exact::snap_to_grid(hit.point))
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
    /// The hash of a fresh four-node board on fixed positions, recorded on
    /// a Mac; tests/unit/board-hash.test.ts asks the wasm build for it.
    const BOARD_HASH: &str = "0x8d980b253c42108e";

    #[test]
    fn a_fixed_board_hashes_to_the_recorded_value() {
        let nodes = vec![
            Point::new(632.2875655532296, 500.0),
            Point::new(331.0475650550718, 654.7742702330713),
            Point::new(525.8608781260419, 205.32863223151458),
            Point::new(712.9536013426019, 777.7602629520936),
        ];
        let state = start(4, 1000, Player::Human, Some(nodes)).unwrap();
        assert_eq!(hash_board(&state), BOARD_HASH);
    }

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
        let Outcome::Applied { snapshot: snap, .. } = apply(
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
        let Outcome::Applied { snapshot: snap, .. } = outcome else {
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
            Outcome::Applied { snapshot, .. } => assert_eq!(snapshot.current_player, Player::Human),
            Outcome::Rejected { rejection } => panic!("opening rejected: {}", rejection.message),
        }
    }

    /// The AI's reply names the face move it drew, and that face move
    /// draws the same line again.
    #[test]
    fn the_ai_reply_carries_a_face_move_that_redraws_its_line() {
        let mut state = start(4, 1000, Player::AI, None).unwrap();
        let AiReply::Move { mov, topology, .. } = ai_move(&mut state) else {
            panic!("the AI has an opening move");
        };
        assert_eq!((topology.a, topology.b), (mov.from_node, mov.to_node));
        assert_eq!(topology.at_a, None);
        let wire = serde_json::to_value(&topology).unwrap();
        assert_eq!(wire["atA"], json!(null));
        assert_eq!(wire["enclosed"], json!([]));
        let mut again = state.clone();
        let Outcome::Applied { snapshot, .. } = apply(&mut state, mov.clone()) else {
            panic!("the AI's own move was rejected");
        };
        let Outcome::Applied {
            snapshot: redrawn, ..
        } = apply_face_move(&mut again, topology, mov.new_node_pos)
        else {
            panic!("the topology did not route");
        };
        // The wire move refits its polyline and snaps the node again, which
        // moves the last bit, so the two lines agree to a micron.
        assert_eq!(
            redrawn.lines[0].segments.len(),
            snapshot.lines[0].segments.len()
        );
        for (x, y) in redrawn.lines[0]
            .segments
            .iter()
            .zip(&snapshot.lines[0].segments)
        {
            for (px, py) in [(x.p0, y.p0), (x.p1, y.p1), (x.p2, y.p2), (x.p3, y.p3)] {
                assert!(px.distance_to(&py) < 1e-6);
            }
        }
        assert!(
            redrawn.nodes[4]
                .position
                .distance_to(&snapshot.nodes[4].position)
                < 1e-6
        );
        assert_eq!(redrawn.current_player, Player::Human);
    }

    /// A wavy human chord is nothing the router draws, so it classifies
    /// as geometry; the AI's reply classifies as its face move, and a face
    /// move given as known is taken as read without routing.
    #[test]
    fn classify_tells_routed_lines_from_hand_drawn_ones() {
        let mut state = GameState::new(4);
        let path = wavy(state.nodes[0].position, state.nodes[1].position);
        let chord = Move {
            from_node: 0,
            to_node: 1,
            new_node_pos: path[20],
            polyline: path,
        };
        assert!(matches!(
            apply(&mut state, chord.clone()),
            Outcome::Applied { .. }
        ));
        let AiReply::Move {
            mov: reply,
            topology,
            ..
        } = ai_move(&mut state)
        else {
            panic!("the AI has a reply");
        };
        let plan = |known: Vec<Option<FaceMove>>| Classification {
            initial_nodes: 4,
            board_size: 1000,
            first_player: Player::Human,
            nodes: None,
            moves: vec![chord.clone(), reply.clone()],
            known,
        };
        assert_eq!(
            classify(plan(vec![None, None])),
            Ok(Classified {
                topology: vec![None, Some(topology.clone())]
            })
        );
        let claimed = FaceMove {
            enclosed: vec![3],
            ..topology.clone()
        };
        assert_eq!(
            classify(plan(vec![Some(claimed.clone()), None])),
            Ok(Classified {
                topology: vec![Some(claimed), Some(topology)]
            })
        );
        let short = Move {
            polyline: vec![chord.polyline[0], chord.polyline[0]],
            ..chord.clone()
        };
        assert!(classify(Classification {
            moves: vec![short],
            known: vec![None],
            ..plan(vec![])
        })
        .unwrap_err()
        .starts_with("move 0:"));
    }

    /// The node a link carries lands on the routed curve where the sharer
    /// put it; one on top of an end node fails the spacing check, with the
    /// kind the visitor's page shows.
    #[test]
    fn a_face_move_whose_node_sits_on_a_node_is_rejected_as_too_close() {
        let mut state = start(4, 1000, Player::AI, None).unwrap();
        let AiReply::Move { mov, topology, .. } = ai_move(&mut state) else {
            panic!("the AI has an opening move");
        };
        let on_a = state.nodes[mov.from_node].position;
        let Outcome::Rejected { rejection } = apply_face_move(&mut state, topology, on_a) else {
            panic!("a node on top of another is refused");
        };
        assert_eq!(rejection.kind, validation::RejectionKind::NodeTooClose);
        assert!(state.lines.is_empty());
    }

    /// A link of a human chord and the AI's routed reply replays whole; the
    /// same link with the reply's face move pointed at a spoke that does
    /// not exist stops at the reply, and so does one whose second drawn
    /// move is too short.
    #[test]
    fn a_rehearsal_names_the_first_move_a_fresh_engine_refuses() {
        let mut state = start(4, 1000, Player::Human, None).unwrap();
        let a = state.nodes[0].position;
        let b = state.nodes[1].position;
        let polyline = straight(a, b, 10);
        let chord = Move {
            from_node: 0,
            to_node: 1,
            new_node_pos: polyline[5],
            polyline,
        };
        assert!(matches!(
            apply(&mut state, chord.clone()),
            Outcome::Applied { .. }
        ));
        let AiReply::Move {
            mov: reply,
            topology: face,
            ..
        } = ai_move(&mut state)
        else {
            panic!("the AI has a reply");
        };
        let plan = |face: FaceMove| Rehearsal {
            router_version: ROUTER_VERSION,
            initial_nodes: 4,
            board_size: 1000,
            first_player: Player::Human,
            nodes: None,
            moves: vec![
                LinkMove::Drawn { mov: chord.clone() },
                LinkMove::Routed {
                    face,
                    at: reply.new_node_pos,
                },
            ],
        };
        let rehearsed = rehearse(plan(face.clone())).unwrap();
        assert_eq!(rehearsed.refused, None);
        // The node reported is the one the same replay places on its board.
        let mut replayed = GameState::new(4);
        assert!(matches!(
            apply(&mut replayed, chord.clone()),
            Outcome::Applied { .. }
        ));
        assert!(matches!(
            apply_face_move(&mut replayed, face.clone(), reply.new_node_pos),
            Outcome::Applied { .. }
        ));
        let placed = replayed.lines.last().unwrap().new_node_pos;
        assert_eq!(rehearsed.nodes, vec![None, Some(placed)]);
        let wrong = FaceMove {
            at_a: Some(1),
            ..face.clone()
        };
        let refused = rehearse(plan(wrong)).unwrap();
        assert_eq!(refused.refused, Some(1));
        assert_eq!(refused.nodes, vec![None, None]);
        let other_router = Rehearsal {
            router_version: ROUTER_VERSION + 1,
            ..plan(face.clone())
        };
        assert!(rehearse(other_router)
            .unwrap_err()
            .contains("router version"));
        let wire = serde_json::json!({
            "routerVersion": ROUTER_VERSION, "initialNodes": 4, "boardSize": 1000,
            "firstPlayer": "human", "nodes": null,
            "moves": [{ "kind": "drawn", "mov": chord }],
        });
        let parsed: Rehearsal = serde_json::from_value(wire).unwrap();
        assert_eq!(rehearse(parsed).unwrap().refused, None);
        // A drawn move the visitor's engine refuses is named the same way.
        let short = Move {
            polyline: vec![a, a],
            ..chord.clone()
        };
        assert_eq!(
            rehearse(Rehearsal {
                moves: vec![
                    LinkMove::Drawn { mov: chord.clone() },
                    LinkMove::Drawn { mov: short }
                ],
                ..plan(face.clone())
            })
            .unwrap()
            .refused,
            Some(1)
        );
        assert!(rehearse(Rehearsal {
            nodes: Some(vec![a]),
            ..plan(face)
        })
        .is_err());
    }

    #[test]
    fn a_face_move_that_does_not_fit_is_rejected_without_a_panic() {
        let mut state = GameState::new(4);
        let face = |a: usize, at_a: Option<usize>, b: usize| FaceMove {
            a,
            at_a,
            b,
            at_b: None,
            enclosed: Vec::new(),
        };
        let at = state.nodes[0].position;
        let missing = apply_face_move(&mut state, face(9, None, 0), at);
        let Outcome::Rejected { rejection } = missing else {
            panic!("node 9 does not exist");
        };
        assert_eq!(rejection.kind, validation::RejectionKind::MissingNode);
        let Outcome::Rejected { rejection } = apply_face_move(&mut state, face(0, Some(3), 1), at)
        else {
            panic!("node 0 has no spoke 3");
        };
        assert_eq!(rejection.kind, validation::RejectionKind::Unroutable);
        let wire = serde_json::to_value(&rejection).unwrap();
        assert_eq!(wire["kind"], json!("unroutable"));
        assert_eq!(wire["at"], json!({ "kind": "node", "id": 0 }));
        assert!(state.lines.is_empty());
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
            Outcome::Applied { snapshot, .. } => {
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
            Outcome::Applied { snapshot, .. } => {
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
        let Outcome::Applied { snapshot, .. } = apply(&mut state, mov) else {
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
        let AiReply::Move { mov, segments, .. } = ai_move(&mut state) else {
            panic!("the AI has a move on an open board");
        };
        let Outcome::Applied { snapshot, .. } = apply(&mut state, mov) else {
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
        let (Outcome::Applied { snapshot: a, .. }, Outcome::Applied { snapshot: b, .. }) =
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

    /// A bare node sealed inside a loop of radius 18 with every other node
    /// dead: its only moves are loops, and the cell is smaller than the
    /// smallest loop the router draws, so the AI reports that it cannot
    /// draw and the game stays open. Each recorded move passes the check
    /// its player gets.
    #[test]
    fn an_undrawn_position_does_not_end_the_game() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("fixtures/undrawn/sealed-2n-1000.json");
        let raw: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let nodes: Vec<Point> = raw["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| Point::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
            .collect();
        assert_eq!(raw["firstPlayer"], "ai");
        let mut state = start(2, 1000, Player::AI, Some(nodes)).unwrap();
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
                state.current_player,
                1000,
            );
            assert_eq!(check(&state, &mov), Ok(()));
            state.apply_move(mov).unwrap();
        }
        assert_eq!(state.current_player, Player::AI);
        assert!(!state.face.search_moves().is_empty());
        assert!(matches!(ai_move(&mut state), AiReply::Undrawn));
        assert!(!snapshot(&state).game_over);
    }

    fn face_move(a: usize, b: usize, enclosed: Vec<usize>) -> FaceMove {
        FaceMove {
            a,
            at_a: None,
            b,
            at_b: None,
            enclosed,
        }
    }

    /// A bare loop's two sides are one position, so node 2 holds the
    /// lowest other node outside and lists loops round 1 and 3 only.
    #[test]
    fn moves_from_lists_every_member_of_the_search_classes_at_the_node() {
        let state = GameState::new(4);
        let listed: Vec<FaceMove> = state
            .face
            .search_moves()
            .into_iter()
            .map(FaceMove::from)
            .collect();
        assert_eq!(
            listed,
            vec![
                face_move(0, 0, vec![]),
                face_move(0, 0, vec![1]),
                face_move(0, 1, vec![]),
            ]
        );
        assert_eq!(
            moves_from(&state, 0).moves,
            vec![
                face_move(0, 0, vec![]),
                face_move(0, 0, vec![1]),
                face_move(0, 0, vec![2]),
                face_move(0, 0, vec![3]),
                face_move(0, 1, vec![]),
                face_move(0, 2, vec![]),
                face_move(0, 3, vec![]),
            ]
        );
        assert_eq!(
            moves_from(&state, 2).moves,
            vec![
                face_move(2, 2, vec![]),
                face_move(2, 2, vec![1]),
                face_move(2, 2, vec![3]),
                face_move(0, 2, vec![]),
                face_move(1, 2, vec![]),
                face_move(2, 3, vec![]),
            ]
        );
        for node in 0..4 {
            for mv in moves_from(&state, node).moves {
                assert!(state.face.check(&ModelMove::from(mv)).is_ok());
            }
        }
        assert!(moves_from(&state, 4).moves.is_empty());
        let wire = serde_json::to_value(moves_from(&state, 1)).unwrap();
        assert_eq!(wire["node"], json!(1));
        assert_eq!(wire["moves"][0]["atA"], json!(null));
        assert_eq!(wire["moves"][0]["enclosed"], json!([]));
    }

    /// The pieces of the face other than the loop's node that `mv` leaves
    /// outside; a bare loop may read back as either side.
    fn other_side(state: &GameState, mv: &FaceMove) -> Vec<usize> {
        let face = state.face.corner_face(mv.a, None).unwrap();
        let mut out: Vec<usize> = state
            .face
            .components(face)
            .iter()
            .map(|&c| state.face.lowest_node(c))
            .filter(|&v| v != mv.a && !mv.enclosed.contains(&v))
            .collect();
        out.sort_unstable();
        out
    }

    #[test]
    fn route_move_draws_a_listed_move_that_embeds_as_itself_and_applies() {
        let state = GameState::new(4);
        for listed in moves_from(&state, 0).moves {
            let reply = route_move(&state, listed.clone());
            let RouteReply::Routed { mov, segments } = reply else {
                panic!("{listed:?} did not route: {reply:?}");
            };
            let read = embedding::embed(&state, &mov.clone().by(Player::Human, 1000)).unwrap();
            assert_eq!(
                (read.a, read.at_a, read.b, read.at_b),
                (listed.a, None, listed.b, None)
            );
            let mut got = read.enclosed.clone();
            got.sort_unstable();
            let bare_loop = listed.a == listed.b;
            assert!(
                got == listed.enclosed || (bare_loop && got == other_side(&state, &listed)),
                "{listed:?} read back as {read:?}"
            );
            let mut played = state.clone();
            let Outcome::Applied { snapshot, .. } = apply(&mut played, mov) else {
                panic!("the routed move for {listed:?} was rejected");
            };
            assert_eq!(snapshot.lines[0].segments, segments);
        }
        let wire = serde_json::to_value(route_move(&state, face_move(0, 1, vec![]))).unwrap();
        assert_eq!(wire["kind"], json!("routed"));
        assert!(!wire["segments"].as_array().unwrap().is_empty());
    }

    #[test]
    fn route_move_names_why_a_move_cannot_be_drawn() {
        let state = GameState::new(4);
        let off_corner = FaceMove {
            at_a: Some(7),
            ..face_move(0, 1, vec![])
        };
        let wire = serde_json::to_value(route_move(&state, off_corner)).unwrap();
        assert_eq!(wire["kind"], json!("unroutable"));
        assert_eq!(wire["reason"], json!("illegal"));
    }

    fn listed(state: &GameState, want: impl Fn(&FaceMove) -> bool) -> FaceMove {
        moves_from(state, 0)
            .moves
            .into_iter()
            .find(|m| want(m))
            .expect("the move is listed")
    }

    fn routed(state: &mut GameState, mv: FaceMove) -> usize {
        let shown = format!("{mv:?}");
        match drawn(state, mv) {
            Outcome::Applied { cut_off, .. } => cut_off,
            Outcome::Rejected { rejection } => {
                panic!("{shown} did not route: {}", rejection.message)
            }
        }
    }

    fn drawn(state: &mut GameState, mv: FaceMove) -> Outcome {
        let path = route::route(state, &mv.clone().into())
            .expect("the move routes")
            .polyline;
        apply_face_move(state, mv, third_along(&path))
    }

    // The node goes a third of the way along the curve, clear of the centre
    // where an earlier node may sit.
    fn third_along(path: &[Point]) -> Point {
        let total: f64 = path.windows(2).map(|w| w[0].distance_to(&w[1])).sum();
        let mut left = total / 3.0;
        for w in path.windows(2) {
            let d = w[0].distance_to(&w[1]);
            if left <= d {
                let f = left / d;
                return Point::new(
                    w[0].x + (w[1].x - w[0].x) * f,
                    w[0].y + (w[1].y - w[0].y) * f,
                );
            }
            left -= d;
        }
        path[path.len() - 1]
    }

    fn bare(a: usize, b: usize, enclosed: Vec<usize>) -> FaceMove {
        FaceMove {
            a,
            at_a: None,
            b,
            at_b: None,
            enclosed,
        }
    }

    /// Six nodes: a loop on 1 round bare node 2 sets 2 apart from 0, 3, 4
    /// and 5. A loop on 0 round that piece then sets 1, 6 and the nested 2
    /// apart from 3, 4 and 5. Five nodes: a chord 0 to 1 and a loop on 3
    /// round nothing set nothing apart; a second chord 0 to 1 with the
    /// two-node piece {3, 6} on its new side sets 3 and 6 apart from 2
    /// and 4, with node 5 on both arcs and so on the wall.
    #[test]
    fn cut_off_counts_the_smaller_side_by_hand() {
        let mut state = GameState::new(6);
        assert_eq!(routed(&mut state, bare(1, 1, vec![2])), 1);
        assert_eq!(routed(&mut state, bare(0, 0, vec![1])), 3);

        let mut state = GameState::new(5);
        assert_eq!(routed(&mut state, bare(0, 1, vec![])), 0);
        assert_eq!(routed(&mut state, bare(3, 3, vec![])), 0);
        let split = listed(&state, |m| m.a == 0 && m.b == 1 && m.enclosed == vec![3]);
        let wire = serde_json::to_value(drawn(&mut state, split)).unwrap();
        assert_eq!(wire["kind"], json!("applied"));
        assert_eq!(wire["cutOff"], json!(2));
    }

    /// Six nodes: a loop on 0 round 1, 2, 3 and 4 leaves 5 alone outside,
    /// so it reads 1 from the smaller side although four nodes sit inside.
    #[test]
    fn a_loop_round_the_larger_side_reads_the_smaller() {
        let mut state = GameState::new(6);
        assert_eq!(routed(&mut state, bare(0, 0, vec![1, 2, 3, 4])), 1);
    }

    /// Seven nodes: a loop on 2 round 3 puts 3 behind the piece {2, 7}. A
    /// chord from 2 to 7 outside the loop with 0 and 1 on its new side
    /// sets them apart from 4, 5 and 6, and 3 counts for neither side;
    /// counted with the new side it would read 3.
    #[test]
    fn a_node_behind_the_split_boundary_counts_for_neither_side() {
        let mut state = GameState::new(7);
        assert_eq!(routed(&mut state, bare(2, 2, vec![3])), 1);
        let chord = moves_from(&state, 2)
            .moves
            .into_iter()
            .find(|m| m.a == 2 && m.b == 7 && m.enclosed == vec![0, 1])
            .expect("the chord is listed");
        assert_eq!(routed(&mut state, chord), 2);
    }

    /// The same curve as a drawn move, one way and the other.
    fn both_ways(state: &GameState, mv: FaceMove) -> (Move, Move) {
        let path = route::route(state, &mv.clone().into())
            .expect("the move routes")
            .polyline;
        let at = third_along(&path);
        let forward = Move {
            from_node: mv.a,
            to_node: mv.b,
            polyline: path.clone(),
            new_node_pos: at,
        };
        let back = Move {
            from_node: mv.b,
            to_node: mv.a,
            polyline: path.into_iter().rev().collect(),
            new_node_pos: at,
        };
        (forward, back)
    }

    fn count(state: &mut GameState, mov: Move) -> (usize, FaceMove) {
        let line = mov.clone().by(state.current_player, state.board_size);
        let face = embedding::embed(state, &line).expect("the curve embeds");
        match apply(state, mov) {
            Outcome::Applied { cut_off, .. } => (cut_off, face.into()),
            Outcome::Rejected { rejection } => panic!("rejected: {}", rejection.message),
        }
    }

    /// A chord drawn from b to a lists the other side of the face as
    /// enclosed; a loop drawn the other way round is the same face move.
    /// Both read the same count.
    #[test]
    fn cut_off_does_not_depend_on_the_drawing_direction() {
        let mut state = GameState::new(5);
        assert_eq!(routed(&mut state, bare(0, 1, vec![])), 0);
        assert_eq!(routed(&mut state, bare(3, 3, vec![])), 0);
        let split = listed(&state, |m| m.a == 0 && m.b == 1 && m.enclosed == vec![3]);
        let (forward, back) = both_ways(&state, split);
        let (there, face_there) = count(&mut state.clone(), forward);
        let (and_back, face_back) = count(&mut state.clone(), back);
        assert_eq!(face_there.enclosed, vec![3]);
        assert_eq!(face_back.enclosed, vec![2, 4]);
        assert_eq!((there, and_back), (2, 2));

        let state = GameState::new(6);
        let (forward, back) = both_ways(&state, bare(1, 1, vec![2]));
        let (there, face_there) = count(&mut state.clone(), forward);
        let (and_back, face_back) = count(&mut state.clone(), back);
        assert_eq!(face_there, face_back);
        assert_eq!((there, and_back), (1, 1));
    }

    /// Every recorded move applies, reads the same count drawn the other
    /// way, and the fixtures between them hold chords that split a face.
    #[test]
    fn fixtures_count_the_same_either_way() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures/games");
        let mut chords_cut = 0;
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            let fixture = crate::replay::load(&path).unwrap();
            let mut state = fixture.start();
            for recorded in &fixture.moves {
                let mov = Move {
                    from_node: recorded.from,
                    to_node: recorded.to,
                    polyline: recorded
                        .polyline
                        .iter()
                        .map(|p| Point::new(p[0], p[1]))
                        .collect(),
                    new_node_pos: Point::new(recorded.new_node[0], recorded.new_node[1]),
                };
                let back = Move {
                    from_node: mov.to_node,
                    to_node: mov.from_node,
                    polyline: mov.polyline.iter().rev().copied().collect(),
                    new_node_pos: mov.new_node_pos,
                };
                let (reversed, _) = count(&mut state.clone(), back);
                let Outcome::Applied {
                    cut_off: reported, ..
                } = apply(&mut state, mov)
                else {
                    panic!("{} rejected a recorded move", path.display());
                };
                assert_eq!(reported, reversed, "{}", path.display());
                if recorded.from != recorded.to && reported > 0 {
                    chords_cut += 1;
                }
            }
        }
        assert!(chords_cut > 0);
    }
}
