use crate::components::label_components;
use crate::morphology::generate_skeleton;
use crate::types::{GameState, Grid, PixelCoord, Point, BOARD_SIZE};
use std::collections::{HashMap, HashSet, VecDeque};

#[derive(Debug, Clone)]
pub struct NodeClassification {
    pub active_nodes: HashSet<usize>,
    pub dead_nodes: HashSet<usize>,
    pub legal_pairs: Vec<(usize, usize)>,
    pub self_loop_candidates: Vec<usize>,
}

pub fn classify_nodes(state: &mut GameState) -> NodeClassification {
    // Generate skeleton if not valid
    if !state.skeleton_cache.is_valid {
        let (skeleton, distance_transform) = generate_skeleton(state);
        state.skeleton_cache.skeleton = skeleton;
        state.skeleton_cache.distance_transform = distance_transform;

        let (labels, components) =
            label_components(&state.skeleton_cache.skeleton, state);
        state.skeleton_cache.component_labels = labels;
        state.skeleton_cache.components = components;

        state.skeleton_cache.is_valid = true;
    }

    let mut active_nodes = HashSet::new();
    let mut dead_nodes = HashSet::new();
    let mut legal_pairs = Vec::new();

    // Phase 1: Identify dead nodes (connection limit)
    for node in &state.nodes {
        if node.connection_count >= 3 {
            dead_nodes.insert(node.id);
        }
    }

    // Phase 2: Component-based reachability analysis
    let components = &state.skeleton_cache.components;

    // Build node -> components mapping
    let mut node_to_components: HashMap<usize, Vec<usize>> = HashMap::new();
    for (comp_id, meta) in components {
        for &node_id in &meta.accessible_nodes {
            node_to_components
                .entry(node_id)
                .or_insert_with(Vec::new)
                .push(*comp_id);
        }
    }

    // All nodes with < 3 connections are considered active.
    // The old approach tried to check skeleton-based reachability here, but the
    // skeleton can fragment and falsely declare nodes dead even when clear open
    // space exists between them. Legal pairs (below) still require actual
    // skeleton paths, so only truly reachable pairs will appear as playable.
    for node in &state.nodes {
        if !dead_nodes.contains(&node.id) {
            active_nodes.insert(node.id);
        }
    }

    // Phase 3: Generate legal pairs
    for node_a in &state.nodes {
        if !active_nodes.contains(&node_a.id) {
            continue;
        }
        for node_b in &state.nodes {
            if node_a.id >= node_b.id {
                continue;
            }
            if !active_nodes.contains(&node_b.id) {
                continue;
            }

            let shares_component = if let Some(comps_a) = node_to_components.get(&node_a.id) {
                if let Some(comps_b) = node_to_components.get(&node_b.id) {
                    comps_a.iter().any(|comp_a| comps_b.contains(comp_a))
                } else {
                    false
                }
            } else {
                false
            };

            if shares_component
                || skeleton_connected(
                    &state.skeleton_cache.skeleton,
                    &node_a.position,
                    &node_b.position,
                )
                || freespace_connected(state, &node_a.position, &node_b.position)
            {
                legal_pairs.push((node_a.id, node_b.id));
            }
        }
    }

    // Phase 4: Self-loop candidates — active nodes with <= 1 connections
    // A self-loop uses 2 connections, so the node needs at least 2 free.
    let self_loop_candidates: Vec<usize> = state
        .nodes
        .iter()
        .filter(|n| n.connection_count <= 1 && active_nodes.contains(&n.id))
        .map(|n| n.id)
        .collect();

    NodeClassification {
        active_nodes,
        dead_nodes,
        legal_pairs,
        self_loop_candidates,
    }
}

/// BFS on skeleton pixels to check if two positions are connected.
/// Unlike find_path_on_skeleton, this ignores forbidden node exclusion zones —
/// we only care whether a connection *exists*, not whether a clean playable
/// path can be generated right now.
fn skeleton_connected(skeleton: &Grid<bool>, pos_a: &crate::types::Point, pos_b: &crate::types::Point) -> bool {
    let find_nearest = |pos: &crate::types::Point| -> Option<PixelCoord> {
        let cx = pos.x.round() as i32;
        let cy = pos.y.round() as i32;
        for r in 0i32..50 {
            for dy in -r..=r {
                for dx in -r..=r {
                    if r > 0 && dx.abs() != r && dy.abs() != r { continue; }
                    let x = cx + dx;
                    let y = cy + dy;
                    if skeleton.in_bounds(x, y) && *skeleton.get(x, y).unwrap_or(&false) {
                        return Some(PixelCoord::new(x, y));
                    }
                }
            }
        }
        None
    };

    let start = match find_nearest(pos_a) { Some(p) => p, None => return false };
    let goal = match find_nearest(pos_b) { Some(p) => p, None => return false };

    if start == goal { return true; }

    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    visited.insert(start);
    queue.push_back(start);

    while let Some(cur) = queue.pop_front() {
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                if dx == 0 && dy == 0 { continue; }
                let nx = cur.x + dx;
                let ny = cur.y + dy;
                let neighbor = PixelCoord::new(nx, ny);
                if neighbor == goal { return true; }
                if skeleton.in_bounds(nx, ny)
                    && *skeleton.get(nx, ny).unwrap_or(&false)
                    && !visited.contains(&neighbor)
                {
                    visited.insert(neighbor);
                    queue.push_back(neighbor);
                }
            }
        }
    }
    false
}

/// Free-space BFS fallback: rasterize lines with a thin buffer (5px) and
/// check if two positions are connected through the remaining free space.
/// This catches cases where the heavily-dilated skeleton (10px) closes gaps
/// that a real move could pass through.
fn freespace_connected(state: &GameState, pos_a: &Point, pos_b: &Point) -> bool {
    let bs = BOARD_SIZE;
    let bs_i = bs as i32;
    let mut blocked = Grid::new(bs, bs, false);

    // Rasterize lines with a 5px buffer (much less than the 10px skeleton dilation)
    let half_width = 5.0;
    for line in &state.lines {
        for i in 1..line.polyline.len() {
            let p1 = &line.polyline[i - 1];
            let p2 = &line.polyline[i];
            let min_x = (p1.x.min(p2.x) - half_width).floor().max(0.0) as i32;
            let max_x = (p1.x.max(p2.x) + half_width).ceil().min(bs as f64 - 1.0) as i32;
            let min_y = (p1.y.min(p2.y) - half_width).floor().max(0.0) as i32;
            let max_y = (p1.y.max(p2.y) + half_width).ceil().min(bs as f64 - 1.0) as i32;
            for y in min_y..=max_y {
                for x in min_x..=max_x {
                    let px = x as f64 + 0.5;
                    let py = y as f64 + 0.5;
                    let dx = p2.x - p1.x;
                    let dy = p2.y - p1.y;
                    let len_sq = dx * dx + dy * dy;
                    let dist = if len_sq < 1e-10 {
                        ((px - p1.x).powi(2) + (py - p1.y).powi(2)).sqrt()
                    } else {
                        let t = ((px - p1.x) * dx + (py - p1.y) * dy) / len_sq;
                        let t = t.clamp(0.0, 1.0);
                        let cx = p1.x + t * dx;
                        let cy = p1.y + t * dy;
                        ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
                    };
                    if dist <= half_width {
                        blocked.set(x, y, true);
                    }
                }
            }
        }
    }

    // Don't block nodes — the only exit from enclosed regions is often
    // through node junctions where lines meet.  We just need to know
    // whether ANY free-space path exists; validation handles the rest.

    // BFS from pos_a to pos_b through free space
    let ax = pos_a.x.round() as i32;
    let ay = pos_a.y.round() as i32;
    let bx = pos_b.x.round() as i32;
    let by = pos_b.y.round() as i32;

    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    let start = PixelCoord::new(ax, ay);
    let goal = PixelCoord::new(bx, by);
    visited.insert(start);
    queue.push_back(start);

    while let Some(cur) = queue.pop_front() {
        for dy in -1..=1i32 {
            for dx in -1..=1i32 {
                if dx == 0 && dy == 0 { continue; }
                let nx = cur.x + dx;
                let ny = cur.y + dy;
                if nx < 0 || ny < 0 || nx >= bs_i || ny >= bs_i { continue; }
                let neighbor = PixelCoord::new(nx, ny);
                if nx == bx && ny == by { return true; }
                if !*blocked.get(nx, ny).unwrap_or(&true) && !visited.contains(&neighbor) {
                    visited.insert(neighbor);
                    queue.push_back(neighbor);
                }
            }
        }
    }
    false
}
