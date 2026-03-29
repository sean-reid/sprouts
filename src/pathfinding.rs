use crate::geometry::{decimate_polyline, polyline_length, resample_polyline, shortcut_path, smooth_path_chaikin};
use crate::morphology::build_line_buffer_grid;
use crate::types::{GameState, Grid, Point, PixelCoord, MAX_ASTAR_ITERATIONS};
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
    find_path_on_skeleton_inner(state, from_node_id, to_node_id, false)
}

/// Last-resort fallback: free-space A* with a thin (4px) line buffer instead
/// of the 10px-dilated skeleton obstacle map. This can squeeze through gaps
/// at node junctions that the dilated map seals shut.
pub fn find_path_relaxed(
    state: &GameState,
    from_node_id: usize,
    to_node_id: usize,
) -> Option<Vec<Point>> {
    let from_node = state.find_node(from_node_id)?;
    let to_node = state.find_node(to_node_id)?;
    let distance_transform = &state.skeleton_cache.distance_transform;
    let bs_i = state.board_size as i32;

    let blocked = build_line_buffer_grid(&state.lines, 4.0, state.board_size);

    let start = PixelCoord::new(from_node.position.x.round() as i32, from_node.position.y.round() as i32);
    let goal = PixelCoord::new(to_node.position.x.round() as i32, to_node.position.y.round() as i32);

    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<PixelCoord, PixelCoord> = HashMap::new();
    let mut g_score: HashMap<PixelCoord, f64> = HashMap::new();
    let mut closed_set: HashSet<PixelCoord> = HashSet::new();

    g_score.insert(start, 0.0);
    open_set.push(AStarNode { coord: start, g_score: 0.0, f_score: heuristic(&start, &goal) });

    let mut iterations = 0;
    while let Some(current_node) = open_set.pop() {
        iterations += 1;
        if iterations > MAX_ASTAR_ITERATIONS { return None; }

        let current = current_node.coord;
        if current == goal { break; }
        if closed_set.contains(&current) { continue; }
        closed_set.insert(current);

        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                if dx == 0 && dy == 0 { continue; }
                let neighbor = PixelCoord::new(current.x + dx, current.y + dy);
                if neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= bs_i || neighbor.y >= bs_i { continue; }
                if *blocked.get(neighbor.x, neighbor.y).unwrap_or(&true) && neighbor != goal { continue; }
                if closed_set.contains(&neighbor) { continue; }

                let move_cost = if dx == 0 || dy == 0 { 1.0 } else { 1.414 };
                // Prefer pixels far from obstacles (use the full distance_transform for aesthetics)
                let dt_dist = distance_transform.get(neighbor.x, neighbor.y).copied().unwrap_or(0) as f64;
                let clearance_bonus = 1.0 + 2.0 / (1.0 + dt_dist * dt_dist);
                let tentative_g = g_score.get(&current).unwrap_or(&f64::INFINITY) + move_cost * clearance_bonus;

                if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                    came_from.insert(neighbor, current);
                    g_score.insert(neighbor, tentative_g);
                    let h = heuristic(&neighbor, &goal);
                    open_set.push(AStarNode { coord: neighbor, g_score: tentative_g, f_score: tentative_g + h });
                }
            }
        }
    }

    if !came_from.contains_key(&goal) && start != goal { return None; }
    let path_pixels = reconstruct_path(&came_from, goal);

    let mut path_points: Vec<Point> = path_pixels
        .iter()
        .map(|p| Point::new(p.x as f64 + 0.5, p.y as f64 + 0.5))
        .collect();
    path_points.insert(0, from_node.position);
    path_points.push(to_node.position);

    let shortcutted = shortcut_path(&path_points, distance_transform, 1);
    let sparse = decimate_polyline(&shortcutted, 12.0);
    let resampled = resample_polyline(&sparse, 15.0);
    let smoothed = smooth_path_chaikin(&resampled, 4);
    let decimated = decimate_polyline(&smoothed, 6.0);

    Some(decimated)
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

    // Post-processing: convert pixel-grid A* path into smooth curves.
    // The JS renderer uses Catmull-Rom splines, so we want FEW widely-spaced
    // control points — the spline fills in smooth curves between them.

    // 1. Shortcut doublebacks
    let shortcutted = shortcut_path(&path_points, distance_transform, 3);

    // 2. Aggressive decimation to extract just the meaningful shape.
    //    12px epsilon: only keep points where the path genuinely changes direction.
    let sparse = decimate_polyline(&shortcutted, 12.0);

    // 3. Coarse resampling for uniform input to the smoother
    let resampled = resample_polyline(&sparse, 15.0);

    // 4. Chaikin smoothing rounds off any remaining angular turns
    let smoothed = smooth_path_chaikin(&resampled, 4);

    // 5. Clearance check — fall back if smoothing pushed into obstacles
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
        // Fallback: less aggressive but still smooth
        let sparse2 = decimate_polyline(&shortcutted, 6.0);
        let resampled2 = resample_polyline(&sparse2, 10.0);
        smooth_path_chaikin(&resampled2, 3)
    };

    // Loose final decimation — let Catmull-Rom do the work
    let decimated = decimate_polyline(&final_path, 6.0);

    Some(decimated)
}

/// Find a self-loop path: a curve from a node back to itself.
///
/// Strategy: find skeleton pixel pairs near the node at different angles,
/// then A* between them on a skeleton where the node center is blocked.
/// Tries multiple pairs, block radii, and allows off-skeleton movement
/// as a last resort.
pub fn find_self_loop_on_skeleton(state: &GameState, node_id: usize) -> Option<Vec<Point>> {
    let node = state.find_node(node_id)?;
    let skeleton = &state.skeleton_cache.skeleton;
    let distance_transform = &state.skeleton_cache.distance_transform;

    let cx = node.position.x.round() as i32;
    let cy = node.position.y.round() as i32;

    // Collect skeleton pixels near the node — search wider (up to 60px)
    let mut by_angle: Vec<(PixelCoord, f64, i32)> = Vec::new(); // (coord, angle, distance)
    for r in 3i32..60 {
        for dy in -r..=r {
            for dx in -r..=r {
                if dx.abs() != r && dy.abs() != r { continue; }
                let x = cx + dx;
                let y = cy + dy;
                if skeleton.in_bounds(x, y) && *skeleton.get(x, y).unwrap_or(&false) {
                    let angle = (dy as f64).atan2(dx as f64);
                    by_angle.push((PixelCoord::new(x, y), angle, r));
                }
            }
        }
        if by_angle.len() >= 40 { break; }
    }

    if by_angle.len() < 2 {
        return None;
    }

    // Build candidate pairs sorted by angular separation (best first).
    // Try multiple pairs, not just the single best — the best angular pair
    // might fail in tight topologies where the skeleton is fragmented.
    let mut pairs: Vec<(usize, usize, f64)> = Vec::new();
    for i in 0..by_angle.len() {
        for j in (i + 1)..by_angle.len() {
            let diff = (by_angle[i].1 - by_angle[j].1).abs();
            let diff = if diff > std::f64::consts::PI {
                2.0 * std::f64::consts::PI - diff
            } else {
                diff
            };
            // Accept any pair with >= 30° separation (was 60°)
            if diff >= std::f64::consts::PI / 6.0 {
                pairs.push((i, j, diff));
            }
        }
    }
    // Sort: prefer larger angular separation, then closer pixels
    pairs.sort_by(|a, b| {
        b.2.partial_cmp(&a.2)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                let dist_a = by_angle[a.0].2 + by_angle[a.1].2;
                let dist_b = by_angle[b.0].2 + by_angle[b.1].2;
                dist_a.cmp(&dist_b)
            })
    });

    // Try top candidate pairs with various block radii and off-skeleton options
    let no_forbidden: Vec<Point> = Vec::new();
    let max_pairs_to_try = pairs.len().min(6);

    for &(pi, pj, _) in pairs.iter().take(max_pairs_to_try) {
        let pixel_a = by_angle[pi].0;
        let pixel_b = by_angle[pj].0;

        // Try progressively smaller block radii, then allow off-skeleton
        for (block_radius, allow_off) in [(5i32, false), (3, false), (2, false), (2, true)] {
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
                allow_off,
            ) {
                if pixels.len() >= 5 {
                    // Build loop: node → path → node
                    let mut path_points = Vec::new();
                    path_points.push(node.position);
                    for p in &pixels {
                        path_points.push(Point::new(p.x as f64 + 0.5, p.y as f64 + 0.5));
                    }
                    path_points.push(node.position);

                    let sparse = decimate_polyline(&path_points, 10.0);
                    let resampled = resample_polyline(&sparse, 12.0);
                    let smoothed = smooth_path_chaikin(&resampled, 4);
                    let decimated = decimate_polyline(&smoothed, 5.0);

                    if polyline_length(&decimated) >= 22.0 {
                        return Some(decimated);
                    }
                }
            }
        }
    }

    None
}

/// Geometric self-loop fallback: creates circular/elliptical arcs from the
/// node without using the skeleton.  Validates against existing lines and
/// the distance transform for clearance.
pub fn generate_geometric_self_loop(state: &GameState, node_id: usize) -> Option<Vec<Point>> {
    use crate::geometry::point_to_segment_distance;

    let node = state.find_node(node_id)?;
    let cx = node.position.x;
    let cy = node.position.y;
    let bs = state.board_size as f64;
    let dt = &state.skeleton_cache.distance_transform;

    let edge_dist = cx.min(cy).min(bs - cx).min(bs - cy);
    let max_radius = (edge_dist - 5.0).max(12.0);

    // Try many radii from small to large
    let radii: [f64; 6] = [15.0, 20.0, 25.0, 35.0, 50.0, 70.0];

    // 24 directions (every 15°) at each radius, plus elliptical stretches
    let mut best_path: Option<Vec<Point>> = None;
    let mut best_clearance = 0.0f64;

    for &radius in &radii {
        let r = radius.min(max_radius);
        if r < 10.0 { continue; }

        for angle_idx in 0..24 {
            let base_angle = (angle_idx as f64) * std::f64::consts::PI / 12.0;

            // Try circular and two elliptical stretches (wider/narrower)
            for &stretch in &[1.0, 0.6, 1.5] {
                let mut points = Vec::new();
                points.push(node.position);
                let steps = 28;
                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let angle = base_angle + std::f64::consts::PI * (t - 0.5);
                    // Elliptical: stretch perpendicular to base direction
                    let local_r_x = r;
                    let local_r_y = r * stretch;
                    let lx = local_r_x * angle.cos();
                    let ly = local_r_y * angle.sin();
                    // Rotate by base_angle
                    let rx = lx * base_angle.cos() - ly * base_angle.sin();
                    let ry = lx * base_angle.sin() + ly * base_angle.cos();
                    let x = (cx + rx).clamp(5.0, bs - 5.0);
                    let y = (cy + ry).clamp(5.0, bs - 5.0);
                    points.push(Point::new(x, y));
                }
                points.push(node.position);

                // Validate: check clearance against dt and existing lines
                let mut min_clearance = f64::INFINITY;
                let mut valid = true;

                for pt in &points[1..points.len() - 1] {
                    let ipx = pt.x.round() as i32;
                    let ipy = pt.y.round() as i32;
                    let d = dt.get(ipx, ipy).copied().unwrap_or(0) as f64;
                    min_clearance = min_clearance.min(d);
                    if d < 3.0 { valid = false; break; }

                    // Check distance to existing lines
                    for line in &state.lines {
                        for si in 1..line.polyline.len() {
                            let p1 = &line.polyline[si - 1];
                            let p2 = &line.polyline[si];
                            let ld = point_to_segment_distance(
                                pt.x, pt.y, p1.x, p1.y, p2.x, p2.y,
                            );
                            min_clearance = min_clearance.min(ld);
                            if ld < 6.0 { valid = false; break; }
                        }
                        if !valid { break; }
                    }
                    if !valid { break; }
                }

                if valid && polyline_length(&points) >= 20.0 && min_clearance > best_clearance {
                    best_clearance = min_clearance;
                    best_path = Some(points);
                }
            }
        }
    }

    let path = best_path?;
    let sparse = decimate_polyline(&path, 8.0);
    let resampled = resample_polyline(&sparse, 10.0);
    let smoothed = smooth_path_chaikin(&resampled, 3);
    let decimated = decimate_polyline(&smoothed, 5.0);

    if polyline_length(&decimated) >= 20.0 {
        Some(decimated)
    } else {
        None
    }
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
        10.0
    } else if avg_width < 6.0 {
        14.0
    } else {
        18.0
    };

    // Distance penalty: strongly prefer the center of corridors.
    // Higher values push paths away from obstacles even when the skeleton allows proximity.
    let dist_penalty_strength = if avg_width < 3.0 {
        2.0
    } else if avg_width < 5.0 {
        3.0
    } else {
        6.0
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
