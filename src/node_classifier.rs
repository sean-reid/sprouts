use crate::components::label_components;
use crate::morphology::generate_skeleton;
use crate::types::GameState;
use std::collections::{HashMap, HashSet};

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

    // Identify active nodes: can reach at least one other active node
    for node in &state.nodes {
        if dead_nodes.contains(&node.id) {
            continue;
        }

        let mut can_reach_other = false;

        // First try component-based analysis (fast)
        if let Some(node_comps) = node_to_components.get(&node.id) {
            for &comp_id in node_comps {
                if let Some(meta) = components.get(&comp_id) {
                    for &other_node_id in &meta.accessible_nodes {
                        if other_node_id != node.id && !dead_nodes.contains(&other_node_id) {
                            if let Some(other_node) = state.nodes.iter().find(|n| n.id == other_node_id) {
                                if other_node.connection_count < 3 {
                                    can_reach_other = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if can_reach_other {
                    break;
                }
            }
        }

        // Fallback: pathfinding for tight spaces where skeleton might fragment
        if !can_reach_other {
            for other_node in &state.nodes {
                if other_node.id == node.id || other_node.connection_count >= 3 || dead_nodes.contains(&other_node.id) {
                    continue;
                }
                if let Some(_path) = crate::pathfinding::find_path_on_skeleton(state, node.id, other_node.id) {
                    can_reach_other = true;
                    break;
                }
            }
        }

        if can_reach_other {
            active_nodes.insert(node.id);
        } else {
            dead_nodes.insert(node.id);
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

            if shares_component {
                legal_pairs.push((node_a.id, node_b.id));
            } else {
                if let Some(_path) = crate::pathfinding::find_path_on_skeleton(state, node_a.id, node_b.id) {
                    legal_pairs.push((node_a.id, node_b.id));
                }
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
