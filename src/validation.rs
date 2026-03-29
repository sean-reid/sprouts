use crate::geometry::{closest_point_on_polyline, distance, polyline_length, segments_intersect, smooth_path_chaikin};
use crate::types::{
    GameState, Move, Point,
    MIN_PATH_LENGTH, MIN_NODE_SPACING, MIN_NODE_SPACING_AI,
    MIN_PATH_CLEARANCE, MIN_PATH_CLEARANCE_AI,
    HUMAN_INTERSECTION_TOLERANCE, AI_INTERSECTION_TOLERANCE,
};

macro_rules! debug_log {
    ($($arg:tt)*) => {
        #[cfg(feature = "debug_validation")]
        {
            web_sys::console::log_1(&format!($($arg)*).into());
        }
    };
}

pub fn validate_move(state: &GameState, mov: &Move) -> Result<(), String> {
    debug_log!("[VALIDATION] Starting HUMAN move validation");
    debug_log!("  From node: {} (connections: {})", mov.from_node,
        state.find_node(mov.from_node).map(|n| n.connection_count).unwrap_or(99));
    debug_log!("  To node: {} (connections: {})", mov.to_node,
        state.find_node(mov.to_node).map(|n| n.connection_count).unwrap_or(99));
    debug_log!("  Path points: {}", mov.polyline.len());
    validate_move_internal(state, mov, false)
}

pub fn validate_ai_move(state: &GameState, mov: &Move) -> Result<(), String> {
    debug_log!("[VALIDATION] Starting AI move validation");
    validate_move_internal(state, mov, true)
}

fn validate_move_internal(state: &GameState, mov: &Move, is_ai_move: bool) -> Result<(), String> {
    #[cfg(feature = "debug_validation")]
    let mode = if is_ai_move { "AI" } else { "HUMAN" };
    let from_node = state.find_node(mov.from_node).ok_or("Source node not found")?;
    let to_node = state.find_node(mov.to_node).ok_or("Target node not found")?;

    // Self-loop: need 2 free connections on the same node
    if mov.from_node == mov.to_node {
        if from_node.connection_count > 1 {
            debug_log!("[{}] FAIL: Self-loop node has {} connections (need <= 1)", mode, from_node.connection_count);
            return Err("Node needs at least 2 free connections for a self-loop".to_string());
        }
    } else {
        if from_node.connection_count >= 3 {
            debug_log!("[{}] FAIL: Source node has 3 connections", mode);
            return Err("Source node already has 3 connections".to_string());
        }
        if to_node.connection_count >= 3 {
            debug_log!("[{}] FAIL: Target node has 3 connections", mode);
            return Err("Target node already has 3 connections".to_string());
        }
    }
    debug_log!("[{}] Check 1 passed: Connection limits OK", mode);

    let path_length = polyline_length(&mov.polyline);
    if path_length < MIN_PATH_LENGTH {
        debug_log!("[{}] FAIL: Path too short: {:.1}px", mode, path_length);
        return Err(format!("Path too short: {:.1}px (minimum: {:.1}px)", path_length, MIN_PATH_LENGTH));
    }
    debug_log!("[{}] Check 2 passed: Path length {:.1}px OK", mode, path_length);

    let min_spacing = if is_ai_move { MIN_NODE_SPACING_AI } else { MIN_NODE_SPACING };
    for node in &state.nodes {
        let dist = mov.new_node_pos.distance_to(&node.position);
        if dist < min_spacing {
            debug_log!("[{}] FAIL: New node too close to node {}: {:.1}px", mode, node.id, dist);
            return Err(format!("New node too close to existing node: {:.1}px (minimum: {:.1}px)", dist, min_spacing));
        }
    }
    debug_log!("[{}] Check 3 passed: Node spacing OK", mode);

    // For human moves, smooth the path to reduce jitter-induced false failures
    let path_for_clearance = if is_ai_move {
        mov.polyline.clone()
    } else {
        smooth_path_chaikin(&mov.polyline, 2)
    };

    let min_clearance = if is_ai_move { MIN_PATH_CLEARANCE_AI } else { MIN_PATH_CLEARANCE };
    for node in &state.nodes {
        if node.id == mov.from_node || node.id == mov.to_node { continue; }
        let (_, dist) = closest_point_on_polyline(&path_for_clearance, &node.position);
        if dist < min_clearance {
            debug_log!("[{}] FAIL: Path too close to node {}: {:.1}px", mode, node.id, dist);
            return Err("Path passes too close to an existing node".to_string());
        }
    }
    debug_log!("[{}] Check 4 passed: Path clearance OK", mode);
    debug_log!("[{}] Check 5: Intersection testing ({} existing lines)", mode, state.lines.len());

    let path_for_intersect = if is_ai_move {
        &mov.polyline
    } else {
        &path_for_clearance
    };

    for line in &state.lines {
        let shares_node =
            line.from_node == mov.from_node || line.from_node == mov.to_node ||
            line.to_node == mov.from_node || line.to_node == mov.to_node;
        let mut geometrically_shares_node = shares_node;
        if !shares_node {
            for node in &state.nodes {
                let on_move_path = path_for_intersect.iter().any(|p| p.distance_to(&node.position) < MIN_PATH_CLEARANCE);
                let on_line_path = line.polyline.iter().any(|p| p.distance_to(&node.position) < MIN_PATH_CLEARANCE);
                if on_move_path && on_line_path {
                    geometrically_shares_node = true;
                    debug_log!("[{}]   Line {}: geometrically shares node {} at ({:.1},{:.1})",
                        mode, line.id, node.id, node.position.x, node.position.y);
                    break;
                }
            }
        }
        for i in 1..path_for_intersect.len() {
            let seg_start = &path_for_intersect[i - 1];
            let seg_end = &path_for_intersect[i];
            for j in 1..line.polyline.len() {
                let line_seg_start = &line.polyline[j - 1];
                let line_seg_end = &line.polyline[j];
                if !segments_intersect(seg_start, seg_end, line_seg_start, line_seg_end) {
                    continue;
                }
                debug_log!("[{}]   Line {}: INTERSECTION DETECTED", mode, line.id);
                if geometrically_shares_node {
                    if let Some(intersection) = get_intersection_point(seg_start, seg_end, line_seg_start, line_seg_end) {
                        let from_pos = from_node.position;
                        let to_pos = to_node.position;
                        let dist_from = intersection.distance_to(&from_pos);
                        let dist_to = intersection.distance_to(&to_pos);
                        let mut min_dist_to_any_node = dist_from.min(dist_to);
                        for node in &state.nodes {
                            let dist = intersection.distance_to(&node.position);
                            min_dist_to_any_node = min_dist_to_any_node.min(dist);
                        }

                        let tolerance = if is_ai_move { AI_INTERSECTION_TOLERANCE } else { HUMAN_INTERSECTION_TOLERANCE };
                        debug_log!("    Min dist to node: {:.1}px, tolerance: {:.1}px", min_dist_to_any_node, tolerance);

                        if min_dist_to_any_node >= tolerance {
                            debug_log!("    FAIL: Intersection not at node");
                            return Err("Path intersects with an existing line".to_string());
                        } else {
                            debug_log!("    PASS: Intersection at node (within {:.1}px)", tolerance);
                        }
                    }
                } else {
                    debug_log!("    FAIL: Lines don't share nodes, any intersection invalid");
                    return Err("Path intersects with an existing line".to_string());
                }
            }
        }
    }
    debug_log!("[{}] Check 5 passed: No invalid intersections", mode);
    Ok(())
}

fn get_intersection_point(a1: &Point, a2: &Point, b1: &Point, b2: &Point) -> Option<Point> {
    let s1_x = a2.x - a1.x;
    let s1_y = a2.y - a1.y;
    let s2_x = b2.x - b1.x;
    let s2_y = b2.y - b1.y;
    let denom = -s2_x * s1_y + s1_x * s2_y;
    if denom.abs() < 1e-10 { return None; }
    let s = (-s1_y * (a1.x - b1.x) + s1_x * (a1.y - b1.y)) / denom;
    let t = (s2_x * (a1.y - b1.y) - s2_y * (a1.x - b1.x)) / denom;
    if s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0 {
        Some(Point::new(a1.x + t * s1_x, a1.y + t * s1_y))
    } else {
        None
    }
}

pub fn validate_new_node_placement(polyline: &[Point], new_node_pos: &Point, existing_nodes: &[crate::types::Node]) -> Result<(), String> {
    let (closest, dist_to_path) = closest_point_on_polyline(polyline, new_node_pos);
    if dist_to_path > 5.0 {
        debug_log!("[PLACEMENT] FAIL: New node not on path (distance: {:.1}px)", dist_to_path);
        return Err("New node must be placed on the drawn path".to_string());
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
    if position_ratio < 0.1 || position_ratio > 0.9 {
        debug_log!("[PLACEMENT] FAIL: Node at {:.0}% (must be 10-90%)", position_ratio * 100.0);
        return Err(format!("Node must be placed in the middle 80% of the path (currently at {:.0}%)", position_ratio * 100.0));
    }
    for node in existing_nodes {
        let dist = new_node_pos.distance_to(&node.position);
        if dist < MIN_NODE_SPACING {
            debug_log!("[PLACEMENT] FAIL: Too close to node {}: {:.1}px", node.id, dist);
            return Err(format!("Too close to existing node: {:.1}px (minimum: {:.1}px)", dist, MIN_NODE_SPACING));
        }
    }
    debug_log!("[PLACEMENT] Placement valid at {:.0}%", position_ratio * 100.0);
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
                p(200.0 + t * 600.0, 500.0 + 80.0 * (t * std::f64::consts::PI).sin())
            })
            .collect();
        let new_node_pos = p(500.0, 500.0 + 80.0 * (0.5 * std::f64::consts::PI).sin());

        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline,
            new_node_pos,
            player: Player::Human,
        };
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
        let result = validate_move(&gs, &mov);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("3 connections"));
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

        let mov = Move {
            from_node: 0,
            to_node: 0,
            polyline,
            new_node_pos: p(250.0, 500.0),
            player: Player::Human,
        };
        let result = validate_move(&gs, &mov);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("self-loop"));
    }

    #[test]
    fn path_too_short_rejected() {
        let (gs, _) = setup_valid_move();
        // Very short path (< MIN_PATH_LENGTH = 20px)
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline: vec![p(200.0, 500.0), p(205.0, 500.0)],
            new_node_pos: p(202.5, 500.0),
            player: Player::Human,
        };
        let result = validate_move(&gs, &mov);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("too short"));
    }

    #[test]
    fn new_node_too_close_rejected() {
        let (gs, _) = setup_valid_move();
        // Place new node right on top of node 0
        let polyline: Vec<Point> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                p(200.0 + t * 600.0, 500.0 + 80.0 * (t * std::f64::consts::PI).sin())
            })
            .collect();
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline,
            new_node_pos: p(201.0, 500.0), // very close to node 0 at (200, 500)
            player: Player::Human,
        };
        let result = validate_move(&gs, &mov);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("too close"));
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
        let mov = Move {
            from_node: 0,
            to_node: 1,
            polyline,
            new_node_pos: p(400.0, 500.0),
            player: Player::Human,
        };
        let result = validate_move(&gs, &mov);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("too close"));
    }
}
