use crate::geometry::extract_line_segment;
use crate::types::{GameState, Grid, Point};
use std::collections::{BinaryHeap, HashMap, HashSet};

const LINE_WIDTH: f64 = 3.0;
const DILATION_RADIUS: f64 = 7.0;  // Reduced to allow tighter gaps while preventing line crossings
const CORRIDOR_LENGTH: f64 = 6.0;
const BORDER_MARGIN: f64 = 20.0;

pub fn generate_skeleton(state: &GameState) -> (Grid<bool>, Grid<u8>) {
    let mut obstacle_map = Grid::new(800, 800, false);
    for line in &state.lines {
        rasterize_polyline(&mut obstacle_map, &line.polyline, LINE_WIDTH);
    }
    add_border_obstacles(&mut obstacle_map, &state.lines);
    let mut protected_mask = Grid::new(800, 800, false);
    for node in &state.nodes {
        if node.connection_count >= 3 { continue; }
        let node_x = node.position.x as i32;
        let node_y = node.position.y as i32;
        let node_radius = 6;
        for dy in -node_radius..=node_radius {
            for dx in -node_radius..=node_radius {
                if dx * dx + dy * dy <= node_radius * node_radius {
                    protected_mask.set(node_x + dx, node_y + dy, true);
                }
            }
        }
        let connected_lines = state.lines_connected_to(node.id);
        if connected_lines.is_empty() {
            for angle_idx in 0..8 {
                let angle = (angle_idx as f64) * std::f64::consts::PI / 4.0;
                let dx = angle.cos();
                let dy = angle.sin();
                let mut segment = vec![node.position];
                for step in 1..=15 {
                    let x = node.position.x + dx * step as f64 * 2.0;
                    let y = node.position.y + dy * step as f64 * 2.0;
                    if x < 0.0 || x >= 800.0 || y < 0.0 || y >= 800.0 { break; }
                    segment.push(Point::new(x, y));
                }
                rasterize_corridor(&mut protected_mask, &segment, 6.0);
            }
        } else {
            for line in connected_lines {
                let node_on_line_idx = line.polyline.iter()
                    .position(|p| p.distance_to(&node.position) < 5.0);
                if let Some(idx) = node_on_line_idx {
                    let (dx_line, dy_line) = if idx + 1 < line.polyline.len() {
                        let dx = line.polyline[idx + 1].x - line.polyline[idx].x;
                        let dy = line.polyline[idx + 1].y - line.polyline[idx].y;
                        let len = (dx * dx + dy * dy).sqrt();
                        if len > 0.1 { (dx / len, dy / len) } else { (1.0, 0.0) }
                    } else if idx > 0 {
                        let dx = line.polyline[idx].x - line.polyline[idx - 1].x;
                        let dy = line.polyline[idx].y - line.polyline[idx - 1].y;
                        let len = (dx * dx + dy * dy).sqrt();
                        if len > 0.1 { (dx / len, dy / len) } else { (1.0, 0.0) }
                    } else {
                        (1.0, 0.0)
                    };
                    let perp1_x = -dy_line;
                    let perp1_y = dx_line;
                    let perp2_x = dy_line;
                    let perp2_y = -dx_line;
                    for (px, py) in [(perp1_x, perp1_y), (perp2_x, perp2_y)] {
                        let mut segment = vec![node.position];
                        for step in 1..=15 {
                            let x = node.position.x + px * step as f64 * 2.0;
                            let y = node.position.y + py * step as f64 * 2.0;
                            if x < 0.0 || x >= 800.0 || y < 0.0 || y >= 800.0 { break; }
                            segment.push(Point::new(x, y));
                        }
                        rasterize_corridor(&mut protected_mask, &segment, 6.0);
                    }
                }
            }
        }
    }
    let mut dilated = Grid::new(800, 800, false);
    for y in 0..800 {
        for x in 0..800 {
            if *protected_mask.get(x, y).unwrap_or(&false) {
                dilated.set(x, y, false);
            } else if *obstacle_map.get(x, y).unwrap_or(&false) {
                dilate_pixel(&mut dilated, x, y, DILATION_RADIUS);
            }
        }
    }
    let free_space = dilated.invert();
    let distance_transform = chamfer_distance_transform(&dilated);
    let skeleton = zhang_suen_thinning(&free_space);
    let repaired_skeleton = repair_skeleton_gaps(&skeleton);
    let mut connected_skeleton = repaired_skeleton.clone();
    for node in &state.nodes {
        if node.connection_count >= 3 { continue; }
        // Pass DILATED map so A* searches in the same space where skeleton exists
        ensure_node_connected(&mut connected_skeleton, node.position.x as i32, node.position.y as i32, &dilated, &distance_transform);
    }
    (connected_skeleton, distance_transform)
}

fn repair_skeleton_gaps(skeleton: &Grid<bool>) -> Grid<bool> {
    let mut repaired = skeleton.clone();
    let mut endpoints = Vec::new();
    for y in 1..799 {
        for x in 1..799 {
            if !skeleton.data[y * 800 + x] { continue; }
            let neighbors = get_neighbors(skeleton, x, y);
            let count = neighbors.iter().filter(|&&n| n).count();
            if count == 1 {
                endpoints.push((x as i32, y as i32));
            }
        }
    }
    for i in 0..endpoints.len() {
        for j in (i + 1)..endpoints.len() {
            let (x1, y1) = endpoints[i];
            let (x2, y2) = endpoints[j];
            let dx = x2 - x1;
            let dy = y2 - y1;
            let dist = ((dx * dx + dy * dy) as f64).sqrt();
            if dist <= 15.0 {
                bresenham_line(&mut repaired, x1, y1, x2, y2);
            }
        }
    }
    repaired
}

fn ensure_node_connected(skeleton: &mut Grid<bool>, node_x: i32, node_y: i32, obstacle_map: &Grid<bool>, distance_transform: &Grid<u8>) {
    for dy in -5..=5 {
        for dx in -5..=5 {
            if *skeleton.get(node_x + dx, node_y + dy).unwrap_or(&false) { return; }
        }
    }
    let mut skeleton_targets = Vec::new();
    for search_radius in 1..500 {  // Increased from 400 to 500
        for dy in -search_radius..=search_radius {
            for dx in -search_radius..=search_radius {
                if (dx as i32).abs() != search_radius && (dy as i32).abs() != search_radius { continue; }
                let check_x = node_x + dx;
                let check_y = node_y + dy;
                if *skeleton.get(check_x, check_y).unwrap_or(&false) {
                    let dist = ((dx * dx + dy * dy) as f64).sqrt();
                    skeleton_targets.push((check_x, check_y, dist));
                }
            }
        }
        if skeleton_targets.len() >= 2 { break; }
    }
    if skeleton_targets.is_empty() {
        // No skeleton found anywhere near this node
        // Node is genuinely isolated - don't create artificial skeleton
        // Component labeling will correctly mark it as unreachable
        return;
    }
    skeleton_targets.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
    let mut connections_made = 0;
    for (target_x, target_y, _dist) in skeleton_targets.iter().take(2) {
        if let Some(path) = astar_connect(node_x, node_y, *target_x, *target_y, obstacle_map, distance_transform) {
            for (x, y) in path {
                skeleton.set(x, y, true);
            }
            connections_made += 1;
        }
    }
    if connections_made == 0 && !skeleton_targets.is_empty() {
        // A* failed to connect - node is genuinely isolated
        // Don't create stubs, let it be properly marked as isolated
        // Component labeling will handle this correctly
    }
}

fn astar_connect(start_x: i32, start_y: i32, goal_x: i32, goal_y: i32, obstacle_map: &Grid<bool>, distance_transform: &Grid<u8>) -> Option<Vec<(i32, i32)>> {
    #[derive(Clone, Copy, PartialEq)]
    struct Node {
        x: i32,
        y: i32,
        f_score: i32,
    }
    impl Eq for Node {}
    impl Ord for Node {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            other.f_score.cmp(&self.f_score)
        }
    }
    impl PartialOrd for Node {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
    let mut g_score: HashMap<(i32, i32), i32> = HashMap::new();
    let mut closed_set: HashSet<(i32, i32)> = HashSet::new();
    g_score.insert((start_x, start_y), 0);
    let h = ((goal_x - start_x).abs() + (goal_y - start_y).abs()) * 5;  // Reduced from 10 - less greedy
    open_set.push(Node { x: start_x, y: start_y, f_score: h });
    
    let mut iterations = 0;
    const MAX_ITERATIONS: usize = 200000;  // Very high to ensure we always find paths
    
    while let Some(current) = open_set.pop() {
        iterations += 1;
        if iterations > MAX_ITERATIONS { return None; }  // Give up after 50k iterations
        
        let pos = (current.x, current.y);
        if current.x == goal_x && current.y == goal_y {
            let mut path = vec![(goal_x, goal_y)];
            let mut curr = (goal_x, goal_y);
            while let Some(&prev) = came_from.get(&curr) {
                path.push(prev);
                curr = prev;
            }
            path.reverse();
            return Some(path);
        }
        if closed_set.contains(&pos) { continue; }
        closed_set.insert(pos);
        for dy in -1..=1 {
            for dx in -1..=1 {
                if dx == 0 && dy == 0 { continue; }
                let nx = current.x + dx;
                let ny = current.y + dy;
                if nx < 0 || ny < 0 || nx >= 800 || ny >= 800 { continue; }
                if *obstacle_map.get(nx, ny).unwrap_or(&true) { continue; }
                let neighbor = (nx, ny);
                if closed_set.contains(&neighbor) { continue; }
                let move_cost = if dx == 0 || dy == 0 { 10 } else { 14 };
                let dist_to_obstacle = distance_transform.get(nx, ny).copied().unwrap_or(0);
                // Minimal penalty - A* should ALWAYS find path if one exists
                let penalty = if dist_to_obstacle < 2 {
                    100   // 1px away - small penalty
                } else if dist_to_obstacle < 4 {
                    40   // 2-3px away - tiny penalty
                } else {
                    0    // 4+ px away - no penalty
                };
                let total_cost = move_cost + penalty;
                let tentative_g = g_score.get(&pos).unwrap_or(&i32::MAX).saturating_add(total_cost);
                if tentative_g < *g_score.get(&neighbor).unwrap_or(&i32::MAX) {
                    came_from.insert(neighbor, pos);
                    g_score.insert(neighbor, tentative_g);
                    let h = ((goal_x - nx).abs() + (goal_y - ny).abs()) * 5;  // Reduced from 10
                    open_set.push(Node { x: nx, y: ny, f_score: tentative_g + h });
                }
            }
        }
    }
    None
}

pub fn zhang_suen_thinning(input: &Grid<bool>) -> Grid<bool> {
    let mut current = input.clone();
    let mut changed = true;
    while changed {
        changed = false;
        let mut to_delete = Vec::new();
        for y in 1..799 {
            for x in 1..799 {
                if !current.data[y * 800 + x] { continue; }
                let neighbors = get_neighbors(&current, x, y);
                if zhang_suen_condition(&neighbors, true) { to_delete.push((x, y)); }
            }
        }
        if !to_delete.is_empty() {
            for (x, y) in to_delete { current.data[y * 800 + x] = false; }
            changed = true;
        }
        let mut to_delete = Vec::new();
        for y in 1..799 {
            for x in 1..799 {
                if !current.data[y * 800 + x] { continue; }
                let neighbors = get_neighbors(&current, x, y);
                if zhang_suen_condition(&neighbors, false) { to_delete.push((x, y)); }
            }
        }
        if !to_delete.is_empty() {
            for (x, y) in to_delete { current.data[y * 800 + x] = false; }
            changed = true;
        }
    }
    current
}

fn get_neighbors(grid: &Grid<bool>, x: usize, y: usize) -> [bool; 8] {
    let x = x as i32;
    let y = y as i32;
    [
        *grid.get(x, y - 1).unwrap_or(&false), *grid.get(x + 1, y - 1).unwrap_or(&false),
        *grid.get(x + 1, y).unwrap_or(&false), *grid.get(x + 1, y + 1).unwrap_or(&false),
        *grid.get(x, y + 1).unwrap_or(&false), *grid.get(x - 1, y + 1).unwrap_or(&false),
        *grid.get(x - 1, y).unwrap_or(&false), *grid.get(x - 1, y - 1).unwrap_or(&false),
    ]
}

fn zhang_suen_condition(neighbors: &[bool; 8], first_subiteration: bool) -> bool {
    let b = neighbors.iter().filter(|&&n| n).count();
    if b < 2 || b > 6 { return false; }
    let mut a = 0;
    for i in 0..8 {
        if !neighbors[i] && neighbors[(i + 1) % 8] { a += 1; }
    }
    if a != 1 { return false; }
    if first_subiteration {
        if neighbors[0] && neighbors[2] && neighbors[4] { return false; }
        if neighbors[2] && neighbors[4] && neighbors[6] { return false; }
    } else {
        if neighbors[0] && neighbors[2] && neighbors[6] { return false; }
        if neighbors[0] && neighbors[4] && neighbors[6] { return false; }
    }
    true
}

fn bresenham_line(grid: &mut Grid<bool>, x0: i32, y0: i32, x1: i32, y1: i32) {
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;
    let mut x = x0;
    let mut y = y0;
    loop {
        grid.set(x, y, true);
        if x == x1 && y == y1 { break; }
        let e2 = 2 * err;
        if e2 > -dy { err -= dy; x += sx; }
        if e2 < dx { err += dx; y += sy; }
    }
}

pub fn chamfer_distance_transform(obstacle_map: &Grid<bool>) -> Grid<u8> {
    let mut dist = Grid::new(800, 800, 255u8);
    for y in 0..800 {
        for x in 0..800 {
            if obstacle_map.data[y * 800 + x] { dist.data[y * 800 + x] = 0; }
        }
    }
    for y in 1..800 {
        for x in 1..800 {
            let current = dist.data[y * 800 + x];
            let up = dist.data[(y - 1) * 800 + x].saturating_add(1);
            let left = dist.data[y * 800 + (x - 1)].saturating_add(1);
            dist.data[y * 800 + x] = current.min(up).min(left);
        }
    }
    for y in (0..799).rev() {
        for x in (0..799).rev() {
            let current = dist.data[y * 800 + x];
            let down = dist.data[(y + 1) * 800 + x].saturating_add(1);
            let right = dist.data[y * 800 + (x + 1)].saturating_add(1);
            dist.data[y * 800 + x] = current.min(down).min(right);
        }
    }
    dist
}

fn add_border_obstacles(grid: &mut Grid<bool>, lines: &[crate::types::Line]) {
    let mut has_content_left = false;
    let mut has_content_right = false;
    let mut has_content_top = false;
    let mut has_content_bottom = false;
    for line in lines {
        for point in &line.polyline {
            if point.x < 30.0 { has_content_left = true; }
            if point.x > 770.0 { has_content_right = true; }
            if point.y < 30.0 { has_content_top = true; }
            if point.y > 770.0 { has_content_bottom = true; }
        }
    }
    if !has_content_top {
        for x in 0..800 { for y in 0..20 { grid.set(x, y, true); } }
    }
    if !has_content_bottom {
        for x in 0..800 { for y in 0..20 { grid.set(x, 799 - y, true); } }
    }
    if !has_content_left {
        for y in 0..800 { for x in 0..20 { grid.set(x, y, true); } }
    }
    if !has_content_right {
        for y in 0..800 { for x in 0..20 { grid.set(799 - x, y, true); } }
    }
}

fn rasterize_polyline(grid: &mut Grid<bool>, polyline: &[Point], width: f64) {
    let half_width = width / 2.0;
    for i in 1..polyline.len() {
        rasterize_segment(grid, &polyline[i - 1], &polyline[i], half_width);
    }
}

fn rasterize_segment(grid: &mut Grid<bool>, p1: &Point, p2: &Point, half_width: f64) {
    let min_x = (p1.x.min(p2.x) - half_width).floor() as i32;
    let max_x = (p1.x.max(p2.x) + half_width).ceil() as i32;
    let min_y = (p1.y.min(p2.y) - half_width).floor() as i32;
    let max_y = (p1.y.max(p2.y) + half_width).ceil() as i32;
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            if !grid.in_bounds(x, y) { continue; }
            let px = x as f64 + 0.5;
            let py = y as f64 + 0.5;
            if point_to_segment_distance(px, py, p1.x, p1.y, p2.x, p2.y) <= half_width {
                grid.set(x, y, true);
            }
        }
    }
}

fn point_to_segment_distance(px: f64, py: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-10 {
        let dpx = px - x1;
        let dpy = py - y1;
        return (dpx * dpx + dpy * dpy).sqrt();
    }
    let t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let closest_x = x1 + t * dx;
    let closest_y = y1 + t * dy;
    let dpx = px - closest_x;
    let dpy = py - closest_y;
    (dpx * dpx + dpy * dpy).sqrt()
}

fn rasterize_corridor(grid: &mut Grid<bool>, segment: &[Point], width: f64) {
    for i in 1..segment.len() {
        rasterize_segment(grid, &segment[i - 1], &segment[i], width / 2.0);
    }
    for point in segment {
        let x = point.x.round() as i32;
        let y = point.y.round() as i32;
        let radius = (width / 2.0) as i32;
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy <= radius * radius {
                    grid.set(x + dx, y + dy, true);
                }
            }
        }
    }
}

fn dilate_pixel(grid: &mut Grid<bool>, cx: i32, cy: i32, radius: f64) {
    let r = radius.ceil() as i32;
    for dy in -r..=r {
        for dx in -r..=r {
            let dist = ((dx * dx + dy * dy) as f64).sqrt();
            if dist <= radius { grid.set(cx + dx, cy + dy, true); }
        }
    }
}
