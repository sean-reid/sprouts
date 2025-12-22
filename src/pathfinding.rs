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
    
    // Measure path tightness by checking distance transform along the path
    let mut tightness_sum = 0.0;
    let mut tightness_samples = 0;
    for point in &resampled {
        let px = point.x.round() as i32;
        let py = point.y.round() as i32;
        if let Some(&dist) = distance_transform.get(px, py) {
            tightness_sum += dist as f64;
            tightness_samples += 1;
        }
    }
    let avg_clearance = if tightness_samples > 0 {
        tightness_sum / tightness_samples as f64
    } else {
        5.0  // Assume moderate if we can't measure
    };
    
    // Dynamic clearance requirement based on path tightness
    // Very tight paths (avg < 2px) → require 0px clearance (trust the skeleton)
    // Tight paths (avg < 4px) → require 1px clearance
    // Normal paths (avg >= 4px) → require 2px clearance
    let required_clearance = if avg_clearance < 2.0 {
        0  // Very tight - trust skeleton completely
    } else if avg_clearance < 4.0 {
        1  // Tight - minimal clearance
    } else {
        1  // Normal - standard clearance (reduced from 2 for better tight space handling)
    };
    
    // Dynamic failure tolerance based on path tightness
    // Tighter paths get more tolerance for smoothing failures
    let failure_tolerance = if avg_clearance < 2.0 {
        0.30  // Very tight - allow 30% failures
    } else if avg_clearance < 4.0 {
        0.20  // Tight - allow 20% failures
    } else {
        0.15  // Normal - allow 15% failures
    };
    
    // Validate smoothed path stays within safe distance from obstacles
    let mut smoothed_is_safe = true;
    let mut failed_points = 0;
    
    for point in &smoothed {
        let px = point.x.round() as i32;
        let py = point.y.round() as i32;
        
        if let Some(&dist) = distance_transform.get(px, py) {
            if dist < required_clearance {
                smoothed_is_safe = false;
                failed_points += 1;
            }
        } else {
            smoothed_is_safe = false;
            failed_points += 1;
        }
    }
    
    // Use smoothed path if safe enough given the context
    let final_path = if smoothed_is_safe || (failed_points as f64 / smoothed.len() as f64) < failure_tolerance {
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

    // Calculate distance to nearest border
    let dist_to_left = pos.x;
    let dist_to_right = 800.0 - pos.x;
    let dist_to_top = pos.y;
    let dist_to_bottom = 800.0 - pos.y;
    let min_border_dist = dist_to_left.min(dist_to_right).min(dist_to_top).min(dist_to_bottom);
    
    // Dynamic max search radius based on border proximity
    // Nodes near borders may need to search further due to thin margins
    let max_radius = if min_border_dist < 15.0 {
        350  // Very close to border - search extensively
    } else if min_border_dist < 30.0 {
        300  // Near border - increased search
    } else {
        250  // Interior - standard search
    };

    // Search in expanding radius
    for radius in 0..max_radius {
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

    // Measure available space by sampling the skeleton
    // This helps us understand if we're in a tight endgame scenario
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
    
    // Calculate average passage width in the search area
    let avg_width = if space_samples > 0 {
        (total_distance as f64 / space_samples as f64)
    } else {
        10.0  // Assume moderate space if we can't measure
    };
    
    // Dynamic parameters based on available space
    // Tight spaces (avg_width < 3) → minimal penalties, tight constraints
    // Open spaces (avg_width > 8) → normal penalties and constraints
    let forbidden_radius = if avg_width < 3.0 {
        5.0  // Very tight - allow closer to nodes
    } else if avg_width < 5.0 {
        6.5  // Tight - slightly relaxed
    } else {
        8.0  // Normal - standard constraint
    };
    
    let dist_penalty_strength = if avg_width < 3.0 {
        0.01  // Very tight - barely prefer wider paths
    } else if avg_width < 5.0 {
        0.03  // Tight - mild preference
    } else {
        0.05  // Normal - moderate preference
    };
    
    let heuristic_weight = if avg_width < 3.0 {
        1.0  // Very tight - pure A* (no bias)
    } else if avg_width < 5.0 {
        1.1  // Tight - slight preference for directness
    } else {
        1.2  // Normal - prefer shorter paths
    };

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
                // Use dynamic radius based on available space
                let neighbor_point = Point::new(neighbor.x as f64 + 0.5, neighbor.y as f64 + 0.5);
                let mut too_close_to_forbidden = false;
                for forbidden in forbidden_nodes {
                    let dx = neighbor_point.x - forbidden.x;
                    let dy = neighbor_point.y - forbidden.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist < forbidden_radius {
                        too_close_to_forbidden = true;
                        break;
                    }
                }
                
                if too_close_to_forbidden {
                    continue;
                }

                let move_cost = if dx == 0 || dy == 0 { 1.0 } else { 1.414 };
                
                // Prefer paths through wider passages (strength varies with context)
                let dist_penalty = if let Some(&dist) = distance_transform.get(neighbor.x, neighbor.y) {
                    1.0 + dist_penalty_strength / (1.0 + dist as f64)
                } else {
                    1.5
                };

                let tentative_g = g_score.get(&current).unwrap_or(&f64::INFINITY) + move_cost * dist_penalty;

                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g);

                    // Dynamic heuristic weight based on available space
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

fn reconstruct_path(came_from: &HashMap<PixelCoord, PixelCoord>, mut current: PixelCoord) -> Vec<PixelCoord> {
    let mut path = vec![current];

    while let Some(&prev) = came_from.get(&current) {
        path.push(prev);
        current = prev;
    }

    path.reverse();
    path
}
