use serde::Serialize;
use tsify::Tsify;

use crate::curve;
use crate::geometry::{
    closest_point_on_polyline, closest_point_on_segment, distance, polyline_length,
    segments_intersect,
};
use crate::types::{
    scale, GameState, Move, Point, AI_INTERSECTION_TOLERANCE, HUMAN_INTERSECTION_TOLERANCE,
    MIN_NODE_SPACING, MIN_NODE_SPACING_AI, MIN_PATH_CLEARANCE, MIN_PATH_CLEARANCE_AI,
    MIN_PATH_LENGTH,
};

/// Why a move was refused and where on the board the problem is.
#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub struct Rejection {
    pub kind: RejectionKind,
    pub message: String,
    pub at: BoardLocation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Tsify)]
#[serde(rename_all = "camelCase")]
pub enum RejectionKind {
    MissingNode,
    NodeFull,
    PathTooShort,
    NodeTooClose,
    PathTooCloseToNode,
    PathCrossesLine,
    NodeOffPath,
    NodeOffBoard,
    NodeNotInMiddle,
    NoRoomForNode,
}

/// Segment i of the submitted path joins its points i and i+1; the range is
/// inclusive at both ends.
#[derive(Debug, Clone, PartialEq, Serialize, Tsify)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum BoardLocation {
    Segments { first: usize, last: usize },
    Node { id: usize },
    Point { at: Point },
}

impl Rejection {
    fn node(kind: RejectionKind, id: usize, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            at: BoardLocation::Node { id },
        }
    }

    fn point(kind: RejectionKind, at: Point, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            at: BoardLocation::Point { at },
        }
    }

    fn whole_path(kind: RejectionKind, path: &[Point], message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
            at: BoardLocation::Segments {
                first: 0,
                last: path.len().saturating_sub(2),
            },
        }
    }

    /// A crossing the geometry checks missed, known only from the path
    /// ending in a different region than it started.
    pub(crate) fn crossing_somewhere(path: &[Point]) -> Self {
        Self::whole_path(
            RejectionKind::PathCrossesLine,
            path,
            "Path crosses an existing line",
        )
    }

    fn crossing(path: &[Point], crossing: &Point) -> Self {
        let segment = nearest_segment(path, crossing);
        Self {
            kind: RejectionKind::PathCrossesLine,
            message: "Path crosses an existing line".to_string(),
            at: BoardLocation::Segments {
                first: segment,
                last: segment,
            },
        }
    }

    pub(crate) fn missing_node(id: usize, role: &str) -> Self {
        Self::node(
            RejectionKind::MissingNode,
            id,
            format!("{role} node not found"),
        )
    }

    pub(crate) fn node_full(id: usize, role: &str) -> Self {
        Self::node(
            RejectionKind::NodeFull,
            id,
            format!("{role} node already has 3 connections"),
        )
    }

    pub(crate) fn self_loop_full(id: usize) -> Self {
        Self::node(
            RejectionKind::NodeFull,
            id,
            "Node needs 2 free connections for a self-loop",
        )
    }

    fn too_short(path: &[Point], length: f64, minimum: f64) -> Self {
        Self::whole_path(
            RejectionKind::PathTooShort,
            path,
            format!("Path too short: {length:.1}px (minimum: {minimum:.1}px)"),
        )
    }

    fn node_too_close(at: Point, dist: f64, minimum: f64) -> Self {
        Self::point(
            RejectionKind::NodeTooClose,
            at,
            format!(
                "New node too close to an existing node: {dist:.1}px (minimum: {minimum:.1}px)"
            ),
        )
    }

    fn path_too_close(id: usize) -> Self {
        Self::node(
            RejectionKind::PathTooCloseToNode,
            id,
            "Path passes too close to an existing node",
        )
    }

    pub fn no_room(at: Point) -> Self {
        Self::point(
            RejectionKind::NoRoomForNode,
            at,
            "No room for a new node on this line",
        )
    }
}

fn nearest_segment(path: &[Point], target: &Point) -> usize {
    let mut best = (0, f64::INFINITY);
    for i in 1..path.len() {
        let (_, dist) = closest_point_on_segment(&path[i - 1], &path[i], target);
        if dist < best.1 {
            best = (i - 1, dist);
        }
    }
    best.0
}

macro_rules! debug_log {
    ($($arg:tt)*) => {
        #[cfg(feature = "debug_validation")]
        {
            web_sys::console::log_1(&format!($($arg)*).into());
        }
    };
}

pub fn validate_move(state: &GameState, mov: &Move) -> Result<(), Rejection> {
    debug_log!("[VALIDATION] Starting HUMAN move validation");
    debug_log!(
        "  From node: {} (connections: {})",
        mov.from_node,
        state
            .find_node(mov.from_node)
            .map(|n| n.connection_count)
            .unwrap_or(99)
    );
    debug_log!(
        "  To node: {} (connections: {})",
        mov.to_node,
        state
            .find_node(mov.to_node)
            .map(|n| n.connection_count)
            .unwrap_or(99)
    );
    debug_log!("  Path points: {}", mov.polyline.len());
    validate_move_internal(state, mov, false)
}

pub fn validate_ai_move(state: &GameState, mov: &Move) -> Result<(), Rejection> {
    debug_log!("[VALIDATION] Starting AI move validation");
    validate_move_internal(state, mov, true)
}

/// Last-resort AI validation with further reduced clearances for tight corridors.
/// Only used when normal AI validation fails for all candidate moves.
pub fn validate_ai_move_tight(state: &GameState, mov: &Move) -> Result<(), Rejection> {
    debug_log!("[VALIDATION] Starting TIGHT AI move validation");
    validate_move_internal_tight(state, mov)
}

fn validate_move_internal(
    state: &GameState,
    mov: &Move,
    is_ai_move: bool,
) -> Result<(), Rejection> {
    #[cfg(feature = "debug_validation")]
    let mode = if is_ai_move { "AI" } else { "HUMAN" };
    let s = scale(state.board_size);
    let from_node = state
        .find_node(mov.from_node)
        .ok_or_else(|| Rejection::missing_node(mov.from_node, "Source"))?;
    let to_node = state
        .find_node(mov.to_node)
        .ok_or_else(|| Rejection::missing_node(mov.to_node, "Target"))?;

    // Self-loop: need 2 free connections on the same node
    if mov.from_node == mov.to_node {
        if from_node.connection_count > 1 {
            debug_log!(
                "[{}] FAIL: Self-loop node has {} connections (need <= 1)",
                mode,
                from_node.connection_count
            );
            return Err(Rejection::self_loop_full(mov.from_node));
        }
    } else {
        if from_node.connection_count >= 3 {
            debug_log!("[{}] FAIL: Source node has 3 connections", mode);
            return Err(Rejection::node_full(mov.from_node, "Source"));
        }
        if to_node.connection_count >= 3 {
            debug_log!("[{}] FAIL: Target node has 3 connections", mode);
            return Err(Rejection::node_full(mov.to_node, "Target"));
        }
    }
    debug_log!("[{}] Check 1 passed: Connection limits OK", mode);

    let path = mov.flatten(curve::tolerance(state.board_size));
    let scaled_min_path = MIN_PATH_LENGTH * s;
    let path_length = polyline_length(&path);
    if path_length < scaled_min_path {
        debug_log!("[{}] FAIL: Path too short: {:.1}px", mode, path_length);
        return Err(Rejection::too_short(
            &mov.polyline,
            path_length,
            scaled_min_path,
        ));
    }
    debug_log!(
        "[{}] Check 2 passed: Path length {:.1}px OK",
        mode,
        path_length
    );

    let min_spacing = if is_ai_move {
        MIN_NODE_SPACING_AI
    } else {
        MIN_NODE_SPACING
    } * s;
    for node in &state.nodes {
        let dist = mov.new_node_pos.distance_to(&node.position);
        if dist < min_spacing {
            debug_log!(
                "[{}] FAIL: New node too close to node {}: {:.1}px",
                mode,
                node.id,
                dist
            );
            return Err(Rejection::node_too_close(
                mov.new_node_pos,
                dist,
                min_spacing,
            ));
        }
    }
    debug_log!("[{}] Check 3 passed: Node spacing OK", mode);

    let min_clearance = if is_ai_move {
        MIN_PATH_CLEARANCE_AI
    } else {
        MIN_PATH_CLEARANCE
    } * s;
    for node in &state.nodes {
        if node.id == mov.from_node || node.id == mov.to_node {
            continue;
        }
        let dist =
            curve::nearest(&mov.segments, &node.position).map_or(f64::INFINITY, |hit| hit.distance);
        if dist < min_clearance {
            debug_log!(
                "[{}] FAIL: Path too close to node {}: {:.1}px",
                mode,
                node.id,
                dist
            );
            return Err(Rejection::path_too_close(node.id));
        }
    }
    debug_log!("[{}] Check 4 passed: Path clearance OK", mode);
    debug_log!(
        "[{}] Check 5: Intersection testing ({} existing lines)",
        mode,
        state.lines.len()
    );

    let geom_share_dist = MIN_PATH_CLEARANCE * s;
    for line in &state.lines {
        let shares_node = line.from_node == mov.from_node
            || line.from_node == mov.to_node
            || line.to_node == mov.from_node
            || line.to_node == mov.to_node;
        let mut geometrically_shares_node = shares_node;
        if !shares_node {
            for node in &state.nodes {
                let on_move_path =
                    closest_point_on_polyline(&path, &node.position).1 < geom_share_dist;
                let on_line_path =
                    closest_point_on_polyline(&line.flat, &node.position).1 < geom_share_dist;
                if on_move_path && on_line_path {
                    geometrically_shares_node = true;
                    debug_log!(
                        "[{}]   Line {}: geometrically shares node {} at ({:.1},{:.1})",
                        mode,
                        line.id,
                        node.id,
                        node.position.x,
                        node.position.y
                    );
                    break;
                }
            }
        }
        for i in 1..path.len() {
            let seg_start = &path[i - 1];
            let seg_end = &path[i];
            for j in 1..line.flat.len() {
                let line_seg_start = &line.flat[j - 1];
                let line_seg_end = &line.flat[j];
                if !segments_intersect(seg_start, seg_end, line_seg_start, line_seg_end) {
                    continue;
                }
                debug_log!("[{}]   Line {}: INTERSECTION DETECTED", mode, line.id);
                let crossing =
                    get_intersection_point(seg_start, seg_end, line_seg_start, line_seg_end);
                if geometrically_shares_node {
                    if let Some(intersection) = crossing {
                        let from_pos = from_node.position;
                        let to_pos = to_node.position;
                        let dist_from = intersection.distance_to(&from_pos);
                        let dist_to = intersection.distance_to(&to_pos);
                        let mut min_dist_to_any_node = dist_from.min(dist_to);
                        for node in &state.nodes {
                            let dist = intersection.distance_to(&node.position);
                            min_dist_to_any_node = min_dist_to_any_node.min(dist);
                        }

                        let tolerance = if is_ai_move {
                            AI_INTERSECTION_TOLERANCE
                        } else {
                            HUMAN_INTERSECTION_TOLERANCE
                        } * s;
                        debug_log!(
                            "    Min dist to node: {:.1}px, tolerance: {:.1}px",
                            min_dist_to_any_node,
                            tolerance
                        );

                        if min_dist_to_any_node >= tolerance {
                            debug_log!("    FAIL: Intersection not at node");
                            return Err(Rejection::crossing(&mov.polyline, &intersection));
                        } else {
                            debug_log!(
                                "    PASS: Intersection at node (within {:.1}px)",
                                tolerance
                            );
                        }
                    }
                } else {
                    debug_log!("    FAIL: Lines don't share nodes, any intersection invalid");
                    return Err(Rejection::crossing(
                        &mov.polyline,
                        &crossing.unwrap_or(*seg_start),
                    ));
                }
            }
        }
    }
    debug_log!("[{}] Check 5 passed: No invalid intersections", mode);
    Ok(())
}

/// Tight validation for last-resort AI moves in narrow corridors.
/// Reduces clearance and spacing proportionally, with further reductions.
fn validate_move_internal_tight(state: &GameState, mov: &Move) -> Result<(), Rejection> {
    let s = scale(state.board_size);
    let from_node = state
        .find_node(mov.from_node)
        .ok_or_else(|| Rejection::missing_node(mov.from_node, "Source"))?;
    let to_node = state
        .find_node(mov.to_node)
        .ok_or_else(|| Rejection::missing_node(mov.to_node, "Target"))?;

    if mov.from_node == mov.to_node {
        if from_node.connection_count > 1 {
            return Err(Rejection::self_loop_full(mov.from_node));
        }
    } else {
        if from_node.connection_count >= 3 {
            return Err(Rejection::node_full(mov.from_node, "Source"));
        }
        if to_node.connection_count >= 3 {
            return Err(Rejection::node_full(mov.to_node, "Target"));
        }
    }

    let path = mov.flatten(curve::tolerance(state.board_size));
    let path_length = polyline_length(&path);
    if path_length < MIN_PATH_LENGTH * s {
        return Err(Rejection::too_short(
            &mov.polyline,
            path_length,
            MIN_PATH_LENGTH * s,
        ));
    }

    // Tight node spacing: 12px scaled
    let tight_spacing = 12.0 * s;
    for node in &state.nodes {
        let dist = mov.new_node_pos.distance_to(&node.position);
        if dist < tight_spacing {
            return Err(Rejection::node_too_close(
                mov.new_node_pos,
                dist,
                tight_spacing,
            ));
        }
    }

    // Tight path clearance: 3px scaled
    let tight_clearance = 3.0 * s;
    for node in &state.nodes {
        if node.id == mov.from_node || node.id == mov.to_node {
            continue;
        }
        let dist =
            curve::nearest(&mov.segments, &node.position).map_or(f64::INFINITY, |hit| hit.distance);
        if dist < tight_clearance {
            return Err(Rejection::path_too_close(node.id));
        }
    }

    // Intersection check with 12px scaled tolerance
    let tight_tolerance = 12.0 * s;
    let geom_dist = tight_clearance;
    for line in &state.lines {
        let shares_node = line.from_node == mov.from_node
            || line.from_node == mov.to_node
            || line.to_node == mov.from_node
            || line.to_node == mov.to_node;
        let mut geometrically_shares_node = shares_node;
        if !shares_node {
            for node in &state.nodes {
                let on_move_path = closest_point_on_polyline(&path, &node.position).1 < geom_dist;
                let on_line_path =
                    closest_point_on_polyline(&line.flat, &node.position).1 < geom_dist;
                if on_move_path && on_line_path {
                    geometrically_shares_node = true;
                    break;
                }
            }
        }
        for i in 1..path.len() {
            let seg_start = &path[i - 1];
            let seg_end = &path[i];
            for j in 1..line.flat.len() {
                let line_seg_start = &line.flat[j - 1];
                let line_seg_end = &line.flat[j];
                if !segments_intersect(seg_start, seg_end, line_seg_start, line_seg_end) {
                    continue;
                }
                let crossing =
                    get_intersection_point(seg_start, seg_end, line_seg_start, line_seg_end);
                if geometrically_shares_node {
                    if let Some(intersection) = crossing {
                        let mut min_dist_to_any_node = f64::INFINITY;
                        for node in &state.nodes {
                            let dist = intersection.distance_to(&node.position);
                            min_dist_to_any_node = min_dist_to_any_node.min(dist);
                        }
                        if min_dist_to_any_node >= tight_tolerance {
                            return Err(Rejection::crossing(&mov.polyline, &intersection));
                        }
                    }
                } else {
                    return Err(Rejection::crossing(
                        &mov.polyline,
                        &crossing.unwrap_or(*seg_start),
                    ));
                }
            }
        }
    }
    Ok(())
}

fn get_intersection_point(a1: &Point, a2: &Point, b1: &Point, b2: &Point) -> Option<Point> {
    let s1_x = a2.x - a1.x;
    let s1_y = a2.y - a1.y;
    let s2_x = b2.x - b1.x;
    let s2_y = b2.y - b1.y;
    let denom = -s2_x * s1_y + s1_x * s2_y;
    if denom.abs() < 1e-10 {
        return None;
    }
    let s = (-s1_y * (a1.x - b1.x) + s1_x * (a1.y - b1.y)) / denom;
    let t = (s2_x * (a1.y - b1.y) - s2_y * (a1.x - b1.x)) / denom;
    if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
        Some(Point::new(a1.x + t * s1_x, a1.y + t * s1_y))
    } else {
        None
    }
}

pub fn validate_new_node_placement(
    polyline: &[Point],
    new_node_pos: &Point,
    existing_nodes: &[crate::types::Node],
    board_size: usize,
) -> Result<(), Rejection> {
    let s = scale(board_size);
    let edge = board_size as f64;
    if !(0.0..=edge).contains(&new_node_pos.x) || !(0.0..=edge).contains(&new_node_pos.y) {
        return Err(Rejection::point(
            RejectionKind::NodeOffBoard,
            *new_node_pos,
            "New node must be on the board",
        ));
    }
    let (closest, dist_to_path) = closest_point_on_polyline(polyline, new_node_pos);
    if dist_to_path > 5.0 * s {
        debug_log!(
            "[PLACEMENT] FAIL: New node not on path (distance: {:.1}px)",
            dist_to_path
        );
        return Err(Rejection::point(
            RejectionKind::NodeOffPath,
            *new_node_pos,
            "New node must be placed on the drawn path",
        ));
    }
    let mut accumulated_length = 0.0;
    let total_length = polyline_length(polyline);
    let mut position_ratio = 0.0;
    for i in 1..polyline.len() {
        let seg_len = distance(&polyline[i - 1], &polyline[i]);
        let dist_to_start = distance(&polyline[i - 1], &closest);
        let dist_to_end = distance(&polyline[i], &closest);
        if dist_to_start + dist_to_end <= seg_len + 1.0 {
            position_ratio = (accumulated_length + dist_to_start) / total_length;
            break;
        }
        accumulated_length += seg_len;
    }
    if !(0.1..=0.9).contains(&position_ratio) {
        debug_log!(
            "[PLACEMENT] FAIL: Node at {:.0}% (must be 10-90%)",
            position_ratio * 100.0
        );
        return Err(Rejection::point(
            RejectionKind::NodeNotInMiddle,
            *new_node_pos,
            format!(
                "Node must be placed in the middle 80% of the path (currently at {:.0}%)",
                position_ratio * 100.0
            ),
        ));
    }
    let scaled_spacing = MIN_NODE_SPACING * s;
    for node in existing_nodes {
        let dist = new_node_pos.distance_to(&node.position);
        if dist < scaled_spacing {
            debug_log!(
                "[PLACEMENT] FAIL: Too close to node {}: {:.1}px",
                node.id,
                dist
            );
            return Err(Rejection::node_too_close(
                *new_node_pos,
                dist,
                scaled_spacing,
            ));
        }
    }
    debug_log!(
        "[PLACEMENT] Placement valid at {:.0}%",
        position_ratio * 100.0
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::*;

    fn p(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    /// Build a simple 2-node game and a valid-looking move between them.
    fn setup_valid_move() -> (GameState, Move) {
        let mut gs = GameState::with_board_size(2, 1000);
        // Place nodes far apart so we have room
        gs.nodes[0].position = p(200.0, 500.0);
        gs.nodes[1].position = p(800.0, 500.0);

        let polyline: Vec<Point> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                p(
                    200.0 + t * 600.0,
                    500.0 + 80.0 * (t * std::f64::consts::PI).sin(),
                )
            })
            .collect();
        let new_node_pos = p(500.0, 500.0 + 80.0 * (0.5 * std::f64::consts::PI).sin());

        let mov = Move::new(0, 1, polyline, new_node_pos, Player::Human, 1000);
        (gs, mov)
    }

    #[test]
    fn valid_move_passes() {
        let (gs, mov) = setup_valid_move();
        assert!(validate_move(&gs, &mov).is_ok());
    }

    #[test]
    fn connection_limit_rejected() {
        let (mut gs, mut mov) = setup_valid_move();
        // Saturate node 0 to 3 connections
        gs.nodes[0].connection_count = 3;
        mov.from_node = 0;
        mov.to_node = 1;
        let rejection = validate_move(&gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::NodeFull);
        assert_eq!(rejection.at, BoardLocation::Node { id: 0 });
    }

    #[test]
    fn self_loop_needs_low_connections() {
        let (mut gs, _) = setup_valid_move();
        gs.nodes[0].connection_count = 2; // needs <= 1 for self-loop

        let polyline: Vec<Point> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                let angle = t * 2.0 * std::f64::consts::PI;
                p(200.0 + 50.0 * angle.cos(), 500.0 + 50.0 * angle.sin())
            })
            .collect();

        let mov = Move::new(0, 0, polyline, p(250.0, 500.0), Player::Human, 1000);
        let rejection = validate_move(&gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::NodeFull);
        assert_eq!(rejection.at, BoardLocation::Node { id: 0 });
    }

    #[test]
    fn path_too_short_rejected() {
        let (gs, _) = setup_valid_move();
        // Very short path (< MIN_PATH_LENGTH = 20px)
        let mov = Move::new(
            0,
            1,
            vec![p(200.0, 500.0), p(205.0, 500.0)],
            p(202.5, 500.0),
            Player::Human,
            1000,
        );
        let rejection = validate_move(&gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::PathTooShort);
        assert_eq!(rejection.at, BoardLocation::Segments { first: 0, last: 0 });
    }

    #[test]
    fn new_node_too_close_rejected() {
        let (gs, _) = setup_valid_move();
        // Place new node right on top of node 0
        let polyline: Vec<Point> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                p(
                    200.0 + t * 600.0,
                    500.0 + 80.0 * (t * std::f64::consts::PI).sin(),
                )
            })
            .collect();
        let mov = Move::new(0, 1, polyline, p(201.0, 500.0), Player::Human, 1000);
        let rejection = validate_move(&gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::NodeTooClose);
        assert_eq!(
            rejection.at,
            BoardLocation::Point {
                at: mov.new_node_pos
            }
        );
    }

    #[test]
    fn a_new_node_past_the_board_edge_is_rejected() {
        let mut gs = GameState::with_board_size(2, 1000);
        gs.nodes[0].position = p(200.0, 100.0);
        gs.nodes[1].position = p(800.0, 100.0);
        // An arc that leaves the board above the top edge between the nodes.
        let polyline: Vec<Point> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                p(
                    200.0 + t * 600.0,
                    100.0 - 140.0 * (t * std::f64::consts::PI).sin(),
                )
            })
            .collect();
        let outside = polyline[20];
        let inside = polyline[3];
        assert!(outside.y < 0.0);
        let flat = Move::new(0, 1, polyline.clone(), outside, Player::Human, 1000).flatten(1.5);
        let rejection = validate_new_node_placement(&flat, &outside, &gs.nodes, 1000).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::NodeOffBoard);
        assert_eq!(rejection.at, BoardLocation::Point { at: outside });
        assert_eq!(
            validate_new_node_placement(&flat, &inside, &gs.nodes, 1000).map_err(|r| r.kind),
            Err(RejectionKind::NodeNotInMiddle)
        );
    }

    #[test]
    fn path_clearance_too_close_to_node() {
        let (mut gs, _) = setup_valid_move();
        // Add a third node right in the path
        gs.nodes.push(Node::new(2, p(500.0, 502.0)));
        gs.next_node_id = 3;

        // Path goes very close to node 2
        let polyline: Vec<Point> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                p(200.0 + t * 600.0, 500.0) // straight line at y=500, node 2 at y=502
            })
            .collect();
        let mov = Move::new(0, 1, polyline, p(400.0, 500.0), Player::Human, 1000);
        let rejection = validate_move(&gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::PathTooCloseToNode);
        assert_eq!(rejection.at, BoardLocation::Node { id: 2 });
    }

    /// A diagonal line from node 0 to node 1 with its new node at the centre.
    fn board_with_a_diagonal() -> GameState {
        let mut gs = GameState::with_board_size(4, 1000);
        gs.nodes[0].position = p(200.0, 200.0);
        gs.nodes[1].position = p(800.0, 800.0);
        gs.nodes[2].position = p(200.0, 800.0);
        gs.nodes[3].position = p(800.0, 200.0);
        gs.apply_move(Move::new(
            0,
            1,
            vec![p(200.0, 200.0), p(500.0, 500.0), p(800.0, 800.0)],
            p(500.0, 500.0),
            Player::Human,
            1000,
        ))
        .unwrap();
        gs
    }

    fn validate_at(tier: Tier, gs: &GameState, mov: &Move) -> Result<(), Rejection> {
        match tier {
            Tier::Human => validate_move(gs, mov),
            Tier::Ai => validate_ai_move(gs, mov),
            Tier::Tight => validate_ai_move_tight(gs, mov),
        }
    }

    /// The crossing is found on the flattening of the fit, so the reported
    /// segment has to be mapped back onto the path the player drew. Four
    /// segments; only the third crosses the diagonal, at (550, 550).
    fn crossing_without_a_shared_node(tier: Tier) {
        let gs = board_with_a_diagonal();
        let polyline = vec![
            p(200.0, 800.0),
            p(350.0, 700.0),
            p(450.0, 650.0),
            p(750.0, 350.0),
            p(800.0, 200.0),
        ];
        let player = match tier {
            Tier::Human => Player::Human,
            Tier::Ai | Tier::Tight => Player::AI,
        };
        let mov = Move::new(2, 3, polyline, p(350.0, 700.0), player, 1000);
        let rejection = validate_at(tier, &gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::PathCrossesLine);
        assert_eq!(rejection.at, BoardLocation::Segments { first: 2, last: 2 });
    }

    /// The move leaves node 0, which the diagonal also touches, and crosses
    /// it on its second segment near (695, 695), far from any node.
    fn crossing_with_a_shared_node(tier: Tier) {
        let gs = board_with_a_diagonal();
        let polyline = vec![
            p(200.0, 200.0),
            p(300.0, 620.0),
            p(720.0, 700.0),
            p(800.0, 200.0),
        ];
        let player = match tier {
            Tier::Human => Player::Human,
            Tier::Ai | Tier::Tight => Player::AI,
        };
        let mov = Move::new(0, 3, polyline, p(300.0, 620.0), player, 1000);
        let rejection = validate_at(tier, &gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::PathCrossesLine);
        assert_eq!(rejection.at, BoardLocation::Segments { first: 1, last: 1 });
    }

    #[test]
    fn crossing_names_the_segment_of_the_submitted_path() {
        crossing_without_a_shared_node(Tier::Human);
        crossing_without_a_shared_node(Tier::Ai);
    }

    #[test]
    fn crossing_past_a_shared_node_names_the_segment_of_the_submitted_path() {
        crossing_with_a_shared_node(Tier::Human);
        crossing_with_a_shared_node(Tier::Ai);
    }

    #[test]
    fn the_tight_tier_names_the_segment_of_the_submitted_path() {
        crossing_without_a_shared_node(Tier::Tight);
        crossing_with_a_shared_node(Tier::Tight);
    }

    #[test]
    fn rejection_wire_format_is_stable() {
        let rejection =
            Rejection::crossing(&[p(0.0, 0.0), p(10.0, 0.0), p(20.0, 0.0)], &p(15.0, 0.0));
        let wire = serde_json::to_value(&rejection).unwrap();
        assert_eq!(
            wire,
            serde_json::json!({
                "kind": "pathCrossesLine",
                "message": "Path crosses an existing line",
                "at": { "kind": "segments", "first": 1, "last": 1 },
            })
        );
        let wire = serde_json::to_value(Rejection::path_too_close(3)).unwrap();
        assert_eq!(wire["at"], serde_json::json!({ "kind": "node", "id": 3 }));
        let wire = serde_json::to_value(Rejection::node_too_close(p(1.0, 2.0), 3.0, 20.0)).unwrap();
        assert_eq!(
            wire["at"],
            serde_json::json!({ "kind": "point", "at": { "x": 1.0, "y": 2.0 } })
        );
    }

    // The flattening's chords sit inside a bend, so a node just under the
    // clearance from the curve can be just over it from the chord.
    #[test]
    fn clearance_is_measured_on_the_curve_not_on_its_flattening() {
        let (mut gs, _) = setup_valid_move();
        let polyline: Vec<Point> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                p(
                    200.0 + t * 600.0,
                    500.0 - 150.0 * (t * std::f64::consts::PI).sin(),
                )
            })
            .collect();
        let probe = Move::new(0, 1, polyline.clone(), polyline[20], Player::Human, 1000);
        let flat = probe.flatten(curve::tolerance(1000));
        let (chord_mid, on_curve, sag) = flat
            .windows(2)
            .map(|w| {
                let m = p((w[0].x + w[1].x) / 2.0, (w[0].y + w[1].y) / 2.0);
                let hit = curve::nearest(&probe.segments, &m).unwrap();
                (m, hit.point, hit.distance)
            })
            .max_by(|a, b| a.2.total_cmp(&b.2))
            .unwrap();
        assert!(
            sag > 0.5,
            "flattening sag of {sag} is too small to tell the two apart"
        );
        let (nx, ny) = (
            (on_curve.x - chord_mid.x) / sag,
            (on_curve.y - chord_mid.y) / sag,
        );
        let reach = MIN_PATH_CLEARANCE - sag / 2.0;
        let intruder = p(on_curve.x + nx * reach, on_curve.y + ny * reach);
        assert!(closest_point_on_polyline(&flat, &intruder).1 >= MIN_PATH_CLEARANCE);
        gs.nodes.push(Node::new(2, intruder));

        let far = *polyline
            .iter()
            .skip(4)
            .find(|v| v.distance_to(&intruder) > 60.0)
            .unwrap();
        let mov = Move::new(0, 1, polyline, far, Player::Human, 1000);
        let rejection = validate_move(&gs, &mov).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::PathTooCloseToNode);
        assert_eq!(rejection.at, BoardLocation::Node { id: 2 });
    }

    // A node placed on a line is only on that line's flattening to within
    // the curve tolerance, and nowhere near its vertices.
    #[derive(Clone, Copy)]
    enum Tier {
        Human,
        Ai,
        Tight,
    }

    fn departing_from_a_mid_line_node(curved: bool, tier: Tier) -> Result<(), Rejection> {
        let mut gs = GameState::with_board_size(3, 1000);
        gs.nodes[0].position = p(200.0, 500.0);
        gs.nodes[1].position = p(800.0, 500.0);
        gs.nodes[2].position = p(500.0, 200.0);
        let first: Vec<Point> = (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                let bulge = if curved {
                    100.0 * (t * std::f64::consts::PI).sin()
                } else {
                    0.0
                };
                p(200.0 + 600.0 * t, 500.0 + bulge)
            })
            .collect();
        let apex = first[20];
        let opening = Move::new(0, 1, first, apex, Player::Human, 1000);
        validate_move(&gs, &opening)?;
        gs.apply_move(opening).unwrap();

        let from = gs.nodes[3].position;
        let to = gs.nodes[2].position;
        let path: Vec<Point> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                p(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)
            })
            .collect();
        let player = match tier {
            Tier::Human => Player::Human,
            Tier::Ai | Tier::Tight => Player::AI,
        };
        let departure = Move::new(3, 2, path.clone(), path[10], player, 1000);
        match tier {
            Tier::Human => validate_move(&gs, &departure),
            Tier::Ai => validate_ai_move(&gs, &departure),
            Tier::Tight => validate_ai_move_tight(&gs, &departure),
        }
    }

    #[test]
    fn a_human_may_leave_a_node_placed_on_a_straight_line() {
        assert_eq!(departing_from_a_mid_line_node(false, Tier::Human), Ok(()));
    }

    #[test]
    fn a_human_may_leave_a_node_placed_on_a_curved_line() {
        assert_eq!(departing_from_a_mid_line_node(true, Tier::Human), Ok(()));
    }

    #[test]
    fn the_ai_may_leave_a_node_placed_on_a_straight_line() {
        assert_eq!(departing_from_a_mid_line_node(false, Tier::Ai), Ok(()));
    }

    #[test]
    fn the_ai_may_leave_a_node_placed_on_a_curved_line() {
        assert_eq!(departing_from_a_mid_line_node(true, Tier::Ai), Ok(()));
    }

    #[test]
    fn the_tight_tier_lets_the_ai_leave_a_node_placed_on_a_straight_line() {
        assert_eq!(departing_from_a_mid_line_node(false, Tier::Tight), Ok(()));
    }

    #[test]
    fn the_tight_tier_lets_the_ai_leave_a_node_placed_on_a_curved_line() {
        assert_eq!(departing_from_a_mid_line_node(true, Tier::Tight), Ok(()));
    }
}
