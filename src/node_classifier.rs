use crate::components::label_components;
use crate::morphology::{build_line_buffer_grid, generate_skeleton};
use crate::types::{GameState, Grid, Point, scale};
use std::collections::{HashMap, HashSet, VecDeque};

#[derive(Debug, Clone)]
pub struct NodeClassification {
    pub active_nodes: HashSet<usize>,
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
    let mut legal_pairs = Vec::new();

    // Phase 1: Identify active nodes (< 3 connections)
    for node in &state.nodes {
        if node.connection_count < 3 {
            active_nodes.insert(node.id);
        }
    }

    // Phase 2: Skeleton component reachability (already computed)
    let components = &state.skeleton_cache.components;
    let mut node_to_skeleton_comps: HashMap<usize, Vec<usize>> = HashMap::new();
    for (comp_id, meta) in components {
        for &node_id in &meta.accessible_nodes {
            node_to_skeleton_comps
                .entry(node_id)
                .or_default()
                .push(*comp_id);
        }
    }

    // Phase 3: Freespace connected components — single O(N) flood fill
    // replaces O(n²) per-pair BFS calls.
    let s = scale(state.board_size);
    let freespace_blocked = build_line_buffer_grid(&state.lines, 3.0 * s, state.board_size);
    let freespace_labels = label_freespace_components(&freespace_blocked);

    // Assign each active node to its freespace component
    let mut node_to_freespace: HashMap<usize, u32> = HashMap::new();
    for node in &state.nodes {
        if !active_nodes.contains(&node.id) {
            continue;
        }
        if let Some(comp) = find_nearest_free_component(&freespace_labels, &freespace_blocked, &node.position) {
            node_to_freespace.insert(node.id, comp);
        }
    }

    // Phase 4: Generate legal pairs — O(1) per pair via component lookups
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

            let shares_skeleton = if let Some(comps_a) = node_to_skeleton_comps.get(&node_a.id) {
                if let Some(comps_b) = node_to_skeleton_comps.get(&node_b.id) {
                    comps_a.iter().any(|c| comps_b.contains(c))
                } else {
                    false
                }
            } else {
                false
            };

            let shares_freespace = match (
                node_to_freespace.get(&node_a.id),
                node_to_freespace.get(&node_b.id),
            ) {
                (Some(a), Some(b)) => a == b,
                _ => false,
            };

            if shares_skeleton || shares_freespace {
                legal_pairs.push((node_a.id, node_b.id));
            }
        }
    }

    // Phase 5: Self-loop candidates
    let self_loop_candidates: Vec<usize> = state
        .nodes
        .iter()
        .filter(|n| n.connection_count <= 1 && active_nodes.contains(&n.id))
        .map(|n| n.id)
        .collect();

    NodeClassification {
        active_nodes,
        legal_pairs,
        self_loop_candidates,
    }
}

/// Single O(N) flood fill to label all freespace connected components.
/// Uses flat Grid<u32> — no HashSet overhead.
fn label_freespace_components(blocked: &Grid<bool>) -> Grid<u32> {
    let w = blocked.width;
    let h = blocked.height;
    let mut labels = Grid::new(w, h, 0u32);
    let mut next_label = 1u32;
    let mut queue = VecDeque::new();

    for sy in 0..h {
        for sx in 0..w {
            if blocked.data[sy * w + sx] || labels.data[sy * w + sx] != 0 {
                continue;
            }
            // Flood fill from (sx, sy)
            labels.data[sy * w + sx] = next_label;
            queue.push_back((sx as i32, sy as i32));

            while let Some((cx, cy)) = queue.pop_front() {
                for dy in -1..=1i32 {
                    for dx in -1..=1i32 {
                        if dx == 0 && dy == 0 { continue; }
                        let nx = cx + dx;
                        let ny = cy + dy;
                        if nx < 0 || ny < 0 || nx >= w as i32 || ny >= h as i32 {
                            continue;
                        }
                        let ni = ny as usize * w + nx as usize;
                        if !blocked.data[ni] && labels.data[ni] == 0 {
                            labels.data[ni] = next_label;
                            queue.push_back((nx, ny));
                        }
                    }
                }
            }
            next_label += 1;
        }
    }
    labels
}

/// Find the freespace component a node belongs to by searching outward
/// from its position (nodes sit on lines, so the node pixel is often blocked).
fn find_nearest_free_component(
    labels: &Grid<u32>,
    _blocked: &Grid<bool>,
    pos: &Point,
) -> Option<u32> {
    let cx = pos.x.round() as i32;
    let cy = pos.y.round() as i32;
    for r in 0i32..40 {
        for dy in -r..=r {
            for dx in -r..=r {
                if r > 0 && dx.abs() != r && dy.abs() != r {
                    continue;
                }
                let x = cx + dx;
                let y = cy + dy;
                if let Some(&label) = labels.get(x, y) {
                    if label > 0 {
                        return Some(label);
                    }
                }
            }
        }
    }
    None
}
