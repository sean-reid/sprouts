use crate::geometry::{decimate_polyline, polyline_length, resample_polyline, shortcut_path, smooth_path_chaikin};
use crate::types::{GameState, Grid, Point, PixelCoord};
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

const MAX_ASTAR_ITERATIONS: usize = 100_000;

#[derive(Debug, Clone, Copy, PartialEq)]
struct AStarNode {
    coord: PixelCoord,
    g_score: f64,
    f_score: f64,
}

impl Eq for AStarNode {}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

pub fn find_path_on_skeleton(
    state: &GameState,
    from_node_id: usize,
    to_node_id: usize,
) -> Option<Vec<Point>> {
    find_path_on_skeleton_inner(state, from_node_id, to_node_id, false)
}

/// Relaxed variant: no forbidden node exclusion zones.  Used as a fallback
/// when the standard pathfinding can't find any move.
pub fn find_path_relaxed(
    state: &GameState,
    from_node_id: usize,
    to_node_id: usize,
) -> Option<Vec<Point>> {
    find_path_on_skeleton_inner(state, from_node_id, to_node_id, true)
}

fn find_path_on_skeleton_inner(
    state: &GameState,
    from_node_id: usize,
    to_node_id: usize,
    relaxed: bool,
) -> Option<Vec<Point>> {
    let from_node = state.find_node(from_node_id)?;
    let to_node = state.find_node(to_node_id)?;

    let skeleton = &state.skeleton_cache.skeleton;
    let distance_transform = &state.skeleton_cache.distance_transform;

    let start_pixel = find_nearest_skeleton_pixel(skeleton, &from_node.position)?;
    let goal_pixel = find_nearest_skeleton_pixel(skeleton, &to_node.position)?;

    let forbidden_nodes: Vec<Point> = if relaxed {
        Vec::new()
    } else {
        state
            .nodes
            .iter()
            .filter(|n| n.id != from_node_id && n.id != to_node_id)
            .map(|n| n.position)
            .collect()
    };

    let path_pixels = astar_search(
        skeleton,
        distance_transform,
        start_pixel,
        goal_pixel,
        &forbidden_nodes,
        true, // allow off-skeleton for regular paths
    )?;

    let mut path_points: Vec<Point> = path_pixels
        .iter()
        .map(|p| Point::new(p.x as f64 + 0.5, p.y as f64 + 0.5))
        .collect();

    path_points.insert(0, from_node.position);
    path_points.push(to_node.position);

    // 1. Remove doublebacks: shortcut detours, then strip any points that
    //    move away from the goal (monotonicity filter).
    let shortcutted = shortcut_path(&path_points, distance_transform, 2);
    let shortcutted = remove_backtracking(&shortcutted);

    // 2. Resample → Smooth (single clean pass)
    let resampled = resample_polyline(&shortcutted, 2.0);
    let smoothed = smooth_path_chaikin(&resampled, 4);

    // 3. Clearance check — fall back to resampled (unsmoothed) if needed
    let mut failed_points = 0;
    for point in &smoothed {
        let px = point.x.round() as i32;
        let py = point.y.round() as i32;
        let dist = distance_transform.get(px, py).copied().unwrap_or(0);
        if dist < 1 {
            failed_points += 1;
        }
    }

    let final_path = if failed_points as f64 / smoothed.len().max(1) as f64 <= 0.20 {
        smoothed
    } else {
        resample_polyline(&shortcutted, 2.0)
    };

    let decimated = decimate_polyline(&final_path, 1.0);

    Some(decimated)
}

/// Find a self-loop path: a curve from a node back to itself.
///
/// Strategy: find two skeleton pixels near the node in different angular
/// directions, then A* between them on a skeleton where the node's center
/// is blocked. This forces the path to arc around the node, creating a
/// natural loop when combined with the node-to-pixel segments.
pub fn find_self_loop_on_skeleton(state: &GameState, node_id: usize) -> Option<Vec<Point>> {
    let node = state.find_node(node_id)?;
    let skeleton = &state.skeleton_cache.skeleton;
    let distance_transform = &state.skeleton_cache.distance_transform;

    let cx = node.position.x.round() as i32;
    let cy = node.position.y.round() as i32;

    // Collect skeleton pixels near the node (5–25px away) with their angles
    let mut by_angle: Vec<(PixelCoord, f64)> = Vec::new();
    for r in 5i32..30 {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r {
                    continue;
                }
                let x = cx + dx;
                let y = cy + dy;
                if skeleton.in_bounds(x, y) && *skeleton.get(x, y).unwrap_or(&false) {
                    let angle = (dy as f64).atan2(dx as f64);
                    by_angle.push((PixelCoord::new(x, y), angle));
                }
            }
        }
        if by_angle.len() >= 20 {
            break;
        }
    }

    if by_angle.len() < 2 {
        return None;
    }

    // Find the pair with maximum angular separation (ideally ~180 degrees)
    let mut best_i = 0;
    let mut best_j = 1;
    let mut best_diff = 0.0f64;
    for i in 0..by_angle.len() {
        for j in (i + 1)..by_angle.len() {
            let diff = (by_angle[i].1 - by_angle[j].1).abs();
            let diff = if diff > std::f64::consts::PI {
                2.0 * std::f64::consts::PI - diff
            } else {
                diff
            };
            if diff > best_diff {
                best_diff = diff;
                best_i = i;
                best_j = j;
            }
        }
    }

    // Need at least 60 degrees of separation for a meaningful loop
    if best_diff < std::f64::consts::PI / 3.0 {
        return None;
    }

    let pixel_a = by_angle[best_i].0;
    let pixel_b = by_angle[best_j].0;

    // Block a small area around the node center on a skeleton copy.
    // This prevents the A* from taking the direct path through the node,
    // forcing it to go around and form a loop.
    // Try progressively smaller block radii if A* fails (the skeleton
    // near a node can be thin, so a large block may disconnect it).
    // No forbidden_nodes: the self-loop path often must pass near other
    // nodes in tight positions.  validate_ai_move enforces the real 8px
    // clearance; duplicating a wider exclusion here causes false negatives
    // and makes the AI unable to play its only available move.
    let no_forbidden: Vec<Point> = Vec::new();
    let mut path_pixels = None;
    for block_radius in [4i32, 3, 2] {
        let mut loop_skeleton = skeleton.clone();
        for dy in -block_radius..=block_radius {
            for dx in -block_radius..=block_radius {
                if dx * dx + dy * dy <= block_radius * block_radius {
                    loop_skeleton.set(cx + dx, cy + dy, false);
                }
            }
        }

        if let Some(pixels) = astar_search(
            &loop_skeleton,
            distance_transform,
            pixel_a,
            pixel_b,
            &no_forbidden,
            false, // no off-skeleton for self-loops (center is blocked)
        ) {
            if pixels.len() >= 8 {
                path_pixels = Some(pixels);
                break;
            }
        }
    }

    let path_pixels = path_pixels?;

    // Build loop: node → pixel_a → path → pixel_b → node
    let mut path_points = Vec::new();
    path_points.push(node.position);
    for p in &path_pixels {
        path_points.push(Point::new(p.x as f64 + 0.5, p.y as f64 + 0.5));
    }
    path_points.push(node.position);

    let resampled = resample_polyline(&path_points, 3.0);
    let smoothed = smooth_path_chaikin(&resampled, 3);
    let decimated = decimate_polyline(&smoothed, 1.0);

    if polyline_length(&decimated) < 22.0 {
        return None;
    }

    Some(decimated)
}

/// Find the nearest skeleton pixel to a position.
/// Searches expanding rings and returns the closest by Euclidean distance.
fn find_nearest_skeleton_pixel(skeleton: &Grid<bool>, pos: &Point) -> Option<PixelCoord> {
    let cx = pos.x.round() as i32;
    let cy = pos.y.round() as i32;

    // Check center
    if skeleton.in_bounds(cx, cy) && *skeleton.get(cx, cy).unwrap_or(&false) {
        return Some(PixelCoord::new(cx, cy));
    }

    let max_radius = 350;

    for radius in 1i32..max_radius {
        let mut best: Option<(PixelCoord, i32)> = None;

        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx.abs() != radius && dy.abs() != radius {
                    continue;
                }
                let x = cx + dx;
                let y = cy + dy;
                if skeleton.in_bounds(x, y) && *skeleton.get(x, y).unwrap_or(&false) {
                    let dist_sq = dx * dx + dy * dy;
                    if best.is_none() || dist_sq < best.unwrap().1 {
                        best = Some((PixelCoord::new(x, y), dist_sq));
                    }
                }
            }
        }

        if let Some((coord, _)) = best {
            return Some(coord);
        }
    }

    None
}

/// Remove points where the path backtracks — i.e., moves further from the
/// goal than a previous point.  Keeps the first and last points intact.
fn remove_backtracking(points: &[Point]) -> Vec<Point> {
    if points.len() <= 2 {
        return points.to_vec();
    }
    let goal = points[points.len() - 1];
    let mut result = vec![points[0]];
    let mut best_dist_sq = f64::INFINITY;

    for i in 1..points.len() - 1 {
        let dx = points[i].x - goal.x;
        let dy = points[i].y - goal.y;
        let dist_sq = dx * dx + dy * dy;
        // Only keep points that are making progress toward the goal
        // (or not moving significantly further away)
        if dist_sq <= best_dist_sq * 1.15 {
            result.push(points[i]);
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
            }
        }
    }
    result.push(goal);
    result
}

fn astar_search(
    skeleton: &Grid<bool>,
    distance_transform: &Grid<u8>,
    start: PixelCoord,
    goal: PixelCoord,
    forbidden_nodes: &[Point],
    allow_off_skeleton: bool,
) -> Option<Vec<PixelCoord>> {
    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<PixelCoord, PixelCoord> = HashMap::new();
    let mut g_score: HashMap<PixelCoord, f64> = HashMap::new();
    let mut closed_set: HashSet<PixelCoord> = HashSet::new();

    // Measure available space for dynamic parameters
    let mut space_samples = 0;
    let mut total_distance = 0u32;
    for sample in 0..100 {
        let x = (start.x + goal.x) / 2 + (sample as i32 % 10 - 5) * 10;
        let y = (start.y + goal.y) / 2 + (sample as i32 / 10 - 5) * 10;
        if let Some(&dist) = distance_transform.get(x, y) {
            if dist > 0 {
                space_samples += 1;
                total_distance += dist as u32;
            }
        }
    }

    let avg_width = if space_samples > 0 {
        total_distance as f64 / space_samples as f64
    } else {
        10.0
    };

    // Keep paths aesthetically clear of nodes.  Off-skeleton routing gives
    // the A* room to swing wide, so we can use generous radii in open areas.
    // In tight corridors, shrink so valid paths aren't blocked entirely.
    let forbidden_radius = if avg_width < 3.0 {
        6.0
    } else if avg_width < 6.0 {
        10.0
    } else {
        14.0
    };

    // Distance penalty: strongly prefer the center of corridors.
    // Higher values push paths away from obstacles even when the skeleton allows proximity.
    let dist_penalty_strength = if avg_width < 3.0 {
        0.3
    } else if avg_width < 5.0 {
        1.5
    } else {
        4.0
    };

    let heuristic_weight = if avg_width < 3.0 {
        1.0
    } else if avg_width < 5.0 {
        1.1
    } else {
        1.2
    };

    g_score.insert(start, 0.0);
    let h_start = heuristic(&start, &goal);
    open_set.push(AStarNode {
        coord: start,
        g_score: 0.0,
        f_score: h_start,
    });

    let mut iterations = 0;

    while let Some(current_node) = open_set.pop() {
        iterations += 1;
        if iterations > MAX_ASTAR_ITERATIONS {
            return None;
        }

        let current = current_node.coord;

        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        if closed_set.contains(&current) {
            continue;
        }
        closed_set.insert(current);

        for dy in -1..=1 {
            for dx in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }

                let neighbor = PixelCoord::new(current.x + dx, current.y + dy);

                if !skeleton.in_bounds(neighbor.x, neighbor.y) {
                    continue;
                }

                let on_skeleton = *skeleton.get(neighbor.x, neighbor.y).unwrap_or(&false);
                let neighbor_dist = distance_transform
                    .get(neighbor.x, neighbor.y)
                    .copied()
                    .unwrap_or(0) as f64;

                // Allow off-skeleton movement through open space (dist >= 3),
                // but block movement into or very near obstacles.
                // Disabled for self-loops where the skeleton center is blocked
                // to force the path to arc around the node.
                if !on_skeleton {
                    if !allow_off_skeleton || neighbor_dist < 3.0 {
                        continue;
                    }
                }

                if closed_set.contains(&neighbor) {
                    continue;
                }

                let neighbor_point =
                    Point::new(neighbor.x as f64 + 0.5, neighbor.y as f64 + 0.5);
                let mut too_close_to_forbidden = false;
                for forbidden in forbidden_nodes {
                    let fdx = neighbor_point.x - forbidden.x;
                    let fdy = neighbor_point.y - forbidden.y;
                    let dist = (fdx * fdx + fdy * fdy).sqrt();
                    if dist < forbidden_radius {
                        too_close_to_forbidden = true;
                        break;
                    }
                }

                if too_close_to_forbidden {
                    continue;
                }

                let move_cost = if dx == 0 || dy == 0 { 1.0 } else { 1.414 };

                // Off-skeleton pixels get a base cost premium so the skeleton
                // is still preferred when it runs through open space.
                let off_skeleton_cost = if on_skeleton { 1.0 } else { 1.5 };

                let dist_penalty = 1.0 + dist_penalty_strength / (1.0 + neighbor_dist * neighbor_dist);

                let tentative_g = g_score.get(&current).unwrap_or(&f64::INFINITY)
                    + move_cost * dist_penalty * off_skeleton_cost;

                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g);

                    let h = heuristic(&neighbor, &goal) * heuristic_weight;
                    open_set.push(AStarNode {
                        coord: neighbor,
                        g_score: tentative_g,
                        f_score: tentative_g + h,
                    });
                }
            }
        }
    }

    None
}

fn heuristic(a: &PixelCoord, b: &PixelCoord) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

fn reconstruct_path(
    came_from: &HashMap<PixelCoord, PixelCoord>,
    mut current: PixelCoord,
) -> Vec<PixelCoord> {
    let mut path = vec![current];

    while let Some(&prev) = came_from.get(&current) {
        path.push(prev);
        current = prev;
    }

    path.reverse();
    path
}
