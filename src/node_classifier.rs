use crate::components::label_components;
use crate::morphology::generate_skeleton;
use crate::types::{GameState, Grid, PixelCoord};
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
