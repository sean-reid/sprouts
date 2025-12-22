use crate::geometry::{decimate_polyline, resample_polyline, smooth_path_chaikin};
use crate::types::{GameState, Point, PixelCoord};
use std::collections::{BinaryHeap, HashMap, HashSet};

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
    let from_node = state.nodes.iter().find(|n| n.id == from_node_id)?;
    let to_node = state.nodes.iter().find(|n| n.id == to_node_id)?;

    let skeleton = &state.skeleton_cache.skeleton;
    let distance_transform = &state.skeleton_cache.distance_transform;

    // Find nearest skeleton pixels to start and end
    let start_pixel = find_nearest_skeleton_pixel(skeleton, &from_node.position)?;
    let goal_pixel = find_nearest_skeleton_pixel(skeleton, &to_node.position)?;

    // Identify forbidden regions: areas around intermediate nodes on existing lines
    // These are nodes that are neither the start nor end of our path
    let forbidden_nodes: Vec<Point> = state.nodes
        .iter()
        .filter(|n| n.id != from_node_id && n.id != to_node_id)
        .map(|n| n.position)
        .collect();

    // Run A*
    let path_pixels = astar_search(skeleton, distance_transform, start_pixel, goal_pixel, &forbidden_nodes)?;

    // Convert pixel path to continuous path
    let mut path_points: Vec<Point> = path_pixels
        .iter()
        .map(|p| Point::new(p.x as f64 + 0.5, p.y as f64 + 0.5))
        .collect();

    // Add actual start and end positions
    path_points.insert(0, from_node.position);
    path_points.push(to_node.position);

    // Resample for smoother curve  
    let resampled = resample_polyline(&path_points, 3.0);
    
    // Apply Chaikin's corner cutting for smooth organic curves (3 iterations for more smoothness)
    let smoothed = smooth_path_chaikin(&resampled, 3);
    
    // Validate smoothed path stays within safe distance from obstacles
    let mut smoothed_is_safe = true;
    let mut failed_points = 0;
    
    for point in &smoothed {
        let px = point.x.round() as i32;
        let py = point.y.round() as i32;
        
        if let Some(&dist) = distance_transform.get(px, py) {
            // Very permissive - only 1px clearance required for tight spaces
            if dist < 1 {
                smoothed_is_safe = false;
                failed_points += 1;
            }
        } else {
            smoothed_is_safe = false;
            failed_points += 1;
        }
    }
    
    // Use smoothed path if at least 85% of points are safe (more lenient for tight spaces)
    let final_path = if smoothed_is_safe || (failed_points as f64 / smoothed.len() as f64) < 0.15 {
        smoothed
    } else {
        resampled
    };

    // Light decimation to preserve smooth curves
    let decimated = decimate_polyline(&final_path, 1.0);

    Some(decimated)
}

fn find_nearest_skeleton_pixel(skeleton: &crate::types::Grid<bool>, pos: &Point) -> Option<PixelCoord> {
    let cx = pos.x.round() as i32;
    let cy = pos.y.round() as i32;

    // Search in expanding radius - increased to handle larger margins and corridors
    for radius in 0..150 {
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if (dx as i32).abs() != radius && (dy as i32).abs() != radius {
                    continue; // Only check perimeter
                }

                let x = cx + dx;
                let y = cy + dy;

                if skeleton.in_bounds(x, y) && *skeleton.get(x, y).unwrap_or(&false) {
                    return Some(PixelCoord::new(x, y));
                }
            }
        }
    }

    None
}

fn astar_search(
    skeleton: &crate::types::Grid<bool>,
    distance_transform: &crate::types::Grid<u8>,
    start: PixelCoord,
    goal: PixelCoord,
    forbidden_nodes: &[Point],
) -> Option<Vec<PixelCoord>> {
    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<PixelCoord, PixelCoord> = HashMap::new();
    let mut g_score: HashMap<PixelCoord, f64> = HashMap::new();
    let mut closed_set: HashSet<PixelCoord> = HashSet::new();

    g_score.insert(start, 0.0);
    let h_start = heuristic(&start, &goal);
    open_set.push(AStarNode {
        coord: start,
        g_score: 0.0,
        f_score: h_start,
    });

    while let Some(current_node) = open_set.pop() {
        let current = current_node.coord;

        if current == goal {
            return Some(reconstruct_path(&came_from, current));
        }

        if closed_set.contains(&current) {
            continue;
        }
        closed_set.insert(current);

        // Explore neighbors (8-connectivity)
        for dy in -1..=1 {
            for dx in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }

                let neighbor = PixelCoord::new(current.x + dx, current.y + dy);

                if !skeleton.in_bounds(neighbor.x, neighbor.y) {
                    continue;
                }

                if !skeleton.get(neighbor.x, neighbor.y).unwrap_or(&false) {
                    continue;
                }

                if closed_set.contains(&neighbor) {
                    continue;
                }

                // Check if this pixel is too close to a forbidden node (intermediate node)
                // Use 8px radius - tight enough to prevent cutting through, loose enough for narrow passages
                let neighbor_point = Point::new(neighbor.x as f64 + 0.5, neighbor.y as f64 + 0.5);
                let mut too_close_to_forbidden = false;
                for forbidden in forbidden_nodes {
                    let dx = neighbor_point.x - forbidden.x;
                    let dy = neighbor_point.y - forbidden.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist < 8.0 {
                        too_close_to_forbidden = true;
                        break;
                    }
                }
                
                if too_close_to_forbidden {
                    continue;
                }

                let move_cost = if dx == 0 || dy == 0 { 1.0 } else { 1.414 };
                
                // Prefer paths through wider passages (but don't make this too strong)
                let dist_penalty = if let Some(&dist) = distance_transform.get(neighbor.x, neighbor.y) {
                    1.0 + 0.05 / (1.0 + dist as f64)
                } else {
                    1.5
                };

                let tentative_g = g_score.get(&current).unwrap_or(&f64::INFINITY) + move_cost * dist_penalty;

                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g);

                    // Moderate heuristic weight (1.2x) to find paths in tight spaces while preferring shorter routes
                    let h = heuristic(&neighbor, &goal) * 1.2;
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

fn reconstruct_path(came_from: &HashMap<PixelCoord, PixelCoord>, mut current: PixelCoord) -> Vec<PixelCoord> {
    let mut path = vec![current];

    while let Some(&prev) = came_from.get(&current) {
        path.push(prev);
        current = prev;
    }

    path.reverse();
    path
}
