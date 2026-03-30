use crate::geometry::point_to_segment_distance;
use crate::types::{GameState, Grid, Line, Point, LINE_RASTER_WIDTH, NODE_PROTECTION_RADIUS, scale};
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

// Adaptive dilation: fraction of local corridor width to dilate from each side.
// 0.35 means 35% from each wall, leaving 30% for the skeleton in the center.
const DILATION_FRACTION: f64 = 0.35;
// Base values for 1000px board — scaled in generate_skeleton.
const MIN_DILATION_BASE: f64 = 3.0;
const MAX_DILATION_BASE: f64 = 50.0;
const BLOCK_SIZE_BASE: usize = 25;

pub fn generate_skeleton(state: &GameState) -> (Grid<bool>, Grid<u8>) {
    let bs = state.board_size;
    let bs_f = bs as f64;
    let s = scale(bs);
    let min_dilation = MIN_DILATION_BASE * s;
    let max_dilation = MAX_DILATION_BASE * s;
    let block_size = (BLOCK_SIZE_BASE as f64 * s).round().max(8.0) as usize;
    let mut obstacle_map = Grid::new(bs, bs, false);
    for line in &state.lines {
        rasterize_polyline(&mut obstacle_map, &line.polyline, LINE_RASTER_WIDTH);
    }
    add_border_obstacles(&mut obstacle_map, &state.lines, &state.nodes, bs);
    let mut protected_mask = Grid::new(bs, bs, false);
    for node in &state.nodes {
        if node.connection_count >= 3 { continue; }
        let node_x = node.position.x as i32;
        let node_y = node.position.y as i32;
        let node_radius = (NODE_PROTECTION_RADIUS as f64 * s).round() as i32;
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
                    if x < 0.0 || x >= bs_f || y < 0.0 || y >= bs_f { break; }
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
                            if x < 0.0 || x >= bs_f || y < 0.0 || y >= bs_f { break; }
                            segment.push(Point::new(x, y));
                        }
                        rasterize_corridor(&mut protected_mask, &segment, 6.0);
                    }
                }
            }
        }
    }
    // --- Adaptive dilation ---
    // Instead of a fixed radius, dilate proportionally to local corridor width.
    // Wide open areas get thick dilation (skeleton forced to center),
    // narrow corridors get thin dilation (corridor stays passable).

    // Step 1: Distance transform of raw obstacles (before dilation)
    let raw_dt = chamfer_distance_transform(&obstacle_map);

    // Step 2: Block-level max pooling to estimate local corridor width.
    // Each block's max dt value ≈ half the width of the widest corridor in that region.
    let blocks_x = bs.div_ceil(block_size);
    let blocks_y = bs.div_ceil(block_size);
    let mut block_max = vec![0u8; blocks_x * blocks_y];
    for y in 0..bs {
        for x in 0..bs {
            let val = raw_dt.data[y * bs + x];
            let bi = (y / block_size) * blocks_x + (x / block_size);
            block_max[bi] = block_max[bi].max(val);
        }
    }

    // Smooth block_max with a 1-ring max to avoid sharp threshold discontinuities
    let mut smooth_block_max = block_max.clone();
    for by in 0..blocks_y {
        for bx in 0..blocks_x {
            let mut m = block_max[by * blocks_x + bx];
            if bx > 0 { m = m.max(block_max[by * blocks_x + (bx - 1)]); }
            if bx + 1 < blocks_x { m = m.max(block_max[by * blocks_x + (bx + 1)]); }
            if by > 0 { m = m.max(block_max[(by - 1) * blocks_x + bx]); }
            if by + 1 < blocks_y { m = m.max(block_max[(by + 1) * blocks_x + bx]); }
            smooth_block_max[by * blocks_x + bx] = m;
        }
    }

    // Step 3: Adaptive dilation — block pixel if it's within a fraction of
    // the local corridor width from the nearest obstacle.
    let mut dilated = Grid::new(bs, bs, false);
    for y in 0..bs {
        for x in 0..bs {
            let raw_dist = raw_dt.data[y * bs + x] as f64;
            if raw_dist == 0.0 {
                // Obstacle pixel itself — always blocked
                dilated.data[y * bs + x] = true;
                continue;
            }
            let bi = (y / block_size) * blocks_x + (x / block_size);
            let local_space = smooth_block_max[bi] as f64;
            let threshold = (local_space * DILATION_FRACTION).clamp(min_dilation, max_dilation);
            if raw_dist < threshold {
                dilated.data[y * bs + x] = true;
            }
        }
    }

    // Step 4: Clear protected pixels (active node zones must stay free)
    for y in 0..bs {
        for x in 0..bs {
            if protected_mask.data[y * bs + x] {
                dilated.data[y * bs + x] = false;
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
        ensure_node_connected(&mut connected_skeleton, node.position.x as i32, node.position.y as i32, &dilated, &distance_transform);
    }
    bridge_active_components(&mut connected_skeleton, state, &dilated, &distance_transform);
    (connected_skeleton, distance_transform)
}

fn bridge_active_components(skeleton: &mut Grid<bool>, state: &GameState, obstacle_map: &Grid<bool>, distance_transform: &Grid<u8>) {
    let bs = state.board_size;
    let bs_i = bs as i32;
    let mut seeds: Vec<(i32, i32)> = Vec::new();
    for node in &state.nodes {
        if node.connection_count >= 3 { continue; }
        let nx = node.position.x.round() as i32;
        let ny = node.position.y.round() as i32;
        let mut found = None;
        'search: for r in 0i32..50 {
            for dy in -r..=r {
                for dx in -r..=r {
                    if r > 0 && dx.abs() != r && dy.abs() != r { continue; }
                    let sx = nx + dx;
                    let sy = ny + dy;
                    if *skeleton.get(sx, sy).unwrap_or(&false) {
                        found = Some((sx, sy));
                        break 'search;
                    }
                }
            }
        }
        if let Some(s) = found { seeds.push(s); }
    }
    if seeds.len() < 2 { return; }
    let mut reachable = Grid::new(bs, bs, false);
    let mut queue = VecDeque::new();
    queue.push_back(seeds[0]);
    reachable.set(seeds[0].0, seeds[0].1, true);
    while let Some((cx, cy)) = queue.pop_front() {
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                if dx == 0 && dy == 0 { continue; }
                let nx = cx + dx;
                let ny = cy + dy;
                if skeleton.in_bounds(nx, ny)
                    && *skeleton.get(nx, ny).unwrap_or(&false)
                    && !*reachable.get(nx, ny).unwrap_or(&true)
                {
                    reachable.set(nx, ny, true);
                    queue.push_back((nx, ny));
                }
            }
        }
    }
    for &(sx, sy) in &seeds[1..] {
        if *reachable.get(sx, sy).unwrap_or(&false) { continue; }
        let mut target = None;
        'bridge: for r in 1i32..bs_i {
            for dy in -r..=r {
                for dx in -r..=r {
                    if dx.abs() != r && dy.abs() != r { continue; }
                    let tx = sx + dx;
                    let ty = sy + dy;
                    if *reachable.get(tx, ty).unwrap_or(&false) {
                        target = Some((tx, ty));
                        break 'bridge;
                    }
                }
            }
        }
        if let Some((tx, ty)) = target {
            if let Some(path) = astar_connect(sx, sy, tx, ty, obstacle_map, distance_transform) {
                for &(px, py) in &path {
                    skeleton.set(px, py, true);
                    reachable.set(px, py, true);
                }
                let mut bq = VecDeque::new();
                bq.push_back((sx, sy));
                while let Some((cx, cy)) = bq.pop_front() {
                    for dy in -1..=1i32 {
                        for dx in -1..=1i32 {
                            if dx == 0 && dy == 0 { continue; }
                            let nx = cx + dx;
                            let ny = cy + dy;
                            if skeleton.in_bounds(nx, ny)
                                && *skeleton.get(nx, ny).unwrap_or(&false)
                                && !*reachable.get(nx, ny).unwrap_or(&true)
                            {
                                reachable.set(nx, ny, true);
                                bq.push_back((nx, ny));
                            }
                        }
                    }
                }
            }
        }
    }
}

fn repair_skeleton_gaps(skeleton: &Grid<bool>) -> Grid<bool> {
    let bs = skeleton.width;
    let bm = bs - 1;
    let mut repaired = skeleton.clone();
    let mut endpoints = Vec::new();
    for y in 1..bm {
        for x in 1..bm {
            if !skeleton.data[y * bs + x] { continue; }
            let neighbors = get_neighbors(skeleton, x, y);
            let count = neighbors.iter().filter(|&&n| n).count();
            if count == 1 { endpoints.push((x as i32, y as i32)); }
        }
    }
    for i in 0..endpoints.len() {
        for j in (i + 1)..endpoints.len() {
            let (x1, y1) = endpoints[i];
            let (x2, y2) = endpoints[j];
            let dx = x2 - x1;
            let dy = y2 - y1;
            let dist = ((dx * dx + dy * dy) as f64).sqrt();
            if dist <= 25.0 { bresenham_line(&mut repaired, x1, y1, x2, y2); }
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
    let mut skeleton_targets: Vec<(i32, i32, f64)> = Vec::new();
    let mut found_ring = false;
    for search_radius in 1i32..600 {
        for dy in -search_radius..=search_radius {
            for dx in -search_radius..=search_radius {
                if dx.abs() != search_radius && dy.abs() != search_radius { continue; }
                let check_x = node_x + dx;
                let check_y = node_y + dy;
                if *skeleton.get(check_x, check_y).unwrap_or(&false) {
                    let dist = ((dx * dx + dy * dy) as f64).sqrt();
                    skeleton_targets.push((check_x, check_y, dist));
                }
            }
        }
        if !skeleton_targets.is_empty() {
            if found_ring { break; }
            found_ring = true;
        }
    }
    if skeleton_targets.is_empty() { return; }
    skeleton_targets.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
    for (target_x, target_y, _dist) in skeleton_targets.iter().take(2) {
        if let Some(path) = astar_connect(node_x, node_y, *target_x, *target_y, obstacle_map, distance_transform) {
            for (x, y) in path { skeleton.set(x, y, true); }
        }
    }
}

fn astar_connect(start_x: i32, start_y: i32, goal_x: i32, goal_y: i32, obstacle_map: &Grid<bool>, distance_transform: &Grid<u8>) -> Option<Vec<(i32, i32)>> {
    #[derive(Clone, Copy, PartialEq)]
    struct Node { x: i32, y: i32, f_score: i32 }
    impl Eq for Node {}
    impl Ord for Node { fn cmp(&self, other: &Self) -> std::cmp::Ordering { other.f_score.cmp(&self.f_score) } }
    impl PartialOrd for Node { fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(other)) } }
    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<(i32, i32), (i32, i32)> = HashMap::new();
    let mut g_score: HashMap<(i32, i32), i32> = HashMap::new();
    let mut closed_set: HashSet<(i32, i32)> = HashSet::new();
    g_score.insert((start_x, start_y), 0);
    let h = ((goal_x - start_x).abs() + (goal_y - start_y).abs()) * 5;
    open_set.push(Node { x: start_x, y: start_y, f_score: h });
    let mut iterations = 0;
    const MAX_ITERATIONS: usize = 200000;
    while let Some(current) = open_set.pop() {
        iterations += 1;
        if iterations > MAX_ITERATIONS { return None; }
        let pos = (current.x, current.y);
        if current.x == goal_x && current.y == goal_y {
            let mut path = vec![(goal_x, goal_y)];
            let mut curr = (goal_x, goal_y);
            while let Some(&prev) = came_from.get(&curr) { path.push(prev); curr = prev; }
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
                if !obstacle_map.in_bounds(nx, ny) { continue; }
                if *obstacle_map.get(nx, ny).unwrap_or(&true) { continue; }
                let neighbor = (nx, ny);
                if closed_set.contains(&neighbor) { continue; }
                let move_cost = if dx == 0 || dy == 0 { 10 } else { 14 };
                let dist_to_obstacle = distance_transform.get(nx, ny).copied().unwrap_or(0);
                let penalty = if dist_to_obstacle < 2 { 100 } else if dist_to_obstacle < 4 { 40 } else { 0 };
                let total_cost = move_cost + penalty;
                let tentative_g = g_score.get(&pos).unwrap_or(&i32::MAX).saturating_add(total_cost);
                if tentative_g < *g_score.get(&neighbor).unwrap_or(&i32::MAX) {
                    came_from.insert(neighbor, pos);
                    g_score.insert(neighbor, tentative_g);
                    let h = ((goal_x - nx).abs() + (goal_y - ny).abs()) * 5;
                    open_set.push(Node { x: nx, y: ny, f_score: tentative_g + h });
                }
            }
        }
    }
    None
}

pub fn zhang_suen_thinning(input: &Grid<bool>) -> Grid<bool> {
    let bs = input.width;
    let bm = bs - 1;
    let mut current = input.clone();
    let mut changed = true;
    while changed {
        changed = false;
        let mut to_delete = Vec::new();
        for y in 1..bm {
            for x in 1..bm {
                if !current.data[y * bs + x] { continue; }
                let neighbors = get_neighbors(&current, x, y);
                if zhang_suen_condition(&neighbors, true) { to_delete.push((x, y)); }
            }
        }
        if !to_delete.is_empty() {
            for (x, y) in to_delete { current.data[y * bs + x] = false; }
            changed = true;
        }
        let mut to_delete = Vec::new();
        for y in 1..bm {
            for x in 1..bm {
                if !current.data[y * bs + x] { continue; }
                let neighbors = get_neighbors(&current, x, y);
                if zhang_suen_condition(&neighbors, false) { to_delete.push((x, y)); }
            }
        }
        if !to_delete.is_empty() {
            for (x, y) in to_delete { current.data[y * bs + x] = false; }
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
    if !(2..=6).contains(&b) { return false; }
    let mut a = 0;
    for i in 0..8 { if !neighbors[i] && neighbors[(i + 1) % 8] { a += 1; } }
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
    let bs = obstacle_map.width;
    let bm = bs - 1;
    let mut dist = Grid::new(bs, bs, 255u8);
    for y in 0..bs {
        for x in 0..bs {
            if obstacle_map.data[y * bs + x] { dist.data[y * bs + x] = 0; }
        }
    }
    for y in 1..bs {
        for x in 1..bm {
            let current = dist.data[y * bs + x];
            let up = dist.data[(y - 1) * bs + x].saturating_add(1);
            let left = dist.data[y * bs + (x - 1)].saturating_add(1);
            let up_left = dist.data[(y - 1) * bs + (x - 1)].saturating_add(1);
            let up_right = dist.data[(y - 1) * bs + (x + 1)].saturating_add(1);
            dist.data[y * bs + x] = current.min(up).min(left).min(up_left).min(up_right);
        }
    }
    for y in (0..bm).rev() {
        for x in (1..bm).rev() {
            let current = dist.data[y * bs + x];
            let down = dist.data[(y + 1) * bs + x].saturating_add(1);
            let right = dist.data[y * bs + (x + 1)].saturating_add(1);
            let down_right = dist.data[(y + 1) * bs + (x + 1)].saturating_add(1);
            let down_left = dist.data[(y + 1) * bs + (x - 1)].saturating_add(1);
            dist.data[y * bs + x] = current.min(down).min(right).min(down_right).min(down_left);
        }
    }
    dist
}

enum Edge { Top, Bottom, Left, Right }

fn add_border_obstacles(grid: &mut Grid<bool>, lines: &[crate::types::Line], nodes: &[crate::types::Node], board_size: usize) {
    add_edge_obstacles(grid, Edge::Top, lines, nodes, board_size);
    add_edge_obstacles(grid, Edge::Bottom, lines, nodes, board_size);
    add_edge_obstacles(grid, Edge::Left, lines, nodes, board_size);
    add_edge_obstacles(grid, Edge::Right, lines, nodes, board_size);
}

fn add_edge_obstacles(grid: &mut Grid<bool>, edge: Edge, lines: &[crate::types::Line], nodes: &[crate::types::Node], board_size: usize) {
    const SEGMENT_SIZE: usize = 50;
    const DETECTION_RANGE: f64 = 80.0;

    let bs = board_size;
    let bs_f = bs as f64;
    let bm = bs - 1;

    // Scale border width proportionally to board size.
    // Thick borders push the skeleton inward so paths use open space
    // rather than hugging existing structures near the center.
    let max_width = (bs as f64 * 0.08).round() as usize;  // 8% of board (80px @ 1000)
    let min_width = (bs as f64 * 0.01).round().max(3.0) as usize;  // 1% of board

    let calc_width = |min_dist: f64| -> usize {
        if min_dist >= DETECTION_RANGE { max_width }
        else if min_dist <= 10.0 { min_width }
        else {
            let t = (min_dist - 10.0) / (DETECTION_RANGE - 10.0);
            (min_width as f64 + t * (max_width - min_width) as f64).round() as usize
        }
    };

    for seg_start in (0..bs).step_by(SEGMENT_SIZE) {
        let seg_end = (seg_start + SEGMENT_SIZE).min(bs);
        let mut min_distance = DETECTION_RANGE + 1.0;

        for node in nodes {
            let (primary, dist_to_edge) = match edge {
                Edge::Top    => (node.position.x, node.position.y),
                Edge::Bottom => (node.position.x, bs_f - node.position.y),
                Edge::Left   => (node.position.y, node.position.x),
                Edge::Right  => (node.position.y, bs_f - node.position.x),
            };
            if primary >= seg_start as f64 && primary < seg_end as f64 {
                min_distance = min_distance.min(dist_to_edge);
            }
        }
        for line in lines {
            for point in &line.polyline {
                let (primary, dist_to_edge) = match edge {
                    Edge::Top    => (point.x, point.y),
                    Edge::Bottom => (point.x, bs_f - point.y),
                    Edge::Left   => (point.y, point.x),
                    Edge::Right  => (point.y, bs_f - point.x),
                };
                if primary >= seg_start as f64 && primary < seg_end as f64 {
                    min_distance = min_distance.min(dist_to_edge);
                }
            }
        }

        let width = calc_width(min_distance);
        match edge {
            Edge::Top => {
                for x in seg_start..seg_end { for y in 0..width { grid.set(x as i32, y as i32, true); } }
            }
            Edge::Bottom => {
                for x in seg_start..seg_end { for y in 0..width { grid.set(x as i32, (bm - y) as i32, true); } }
            }
            Edge::Left => {
                for y in seg_start..seg_end { for x in 0..width { grid.set(x as i32, y as i32, true); } }
            }
            Edge::Right => {
                for y in seg_start..seg_end { for x in 0..width { grid.set((bm - x) as i32, y as i32, true); } }
            }
        }
    }
}

fn rasterize_polyline(grid: &mut Grid<bool>, polyline: &[Point], width: f64) {
    let half_width = width / 2.0;
    for i in 1..polyline.len() { rasterize_segment(grid, &polyline[i - 1], &polyline[i], half_width); }
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
            if point_to_segment_distance(px, py, p1.x, p1.y, p2.x, p2.y) <= half_width { grid.set(x, y, true); }
        }
    }
}

/// Build a grid marking pixels within `buffer_radius` of any existing line.
/// Shared by node_classifier (freespace connectivity) and pathfinding (relaxed A*).
pub fn build_line_buffer_grid(lines: &[Line], buffer_radius: f64, board_size: usize) -> Grid<bool> {
    let mut grid = Grid::new(board_size, board_size, false);
    for line in lines {
        rasterize_polyline(&mut grid, &line.polyline, buffer_radius * 2.0);
    }
    grid
}

fn rasterize_corridor(grid: &mut Grid<bool>, segment: &[Point], width: f64) {
    for i in 1..segment.len() { rasterize_segment(grid, &segment[i - 1], &segment[i], width / 2.0); }
    for point in segment {
        let x = point.x.round() as i32;
        let y = point.y.round() as i32;
        let radius = (width / 2.0) as i32;
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy <= radius * radius { grid.set(x + dx, y + dy, true); }
            }
        }
    }
}

