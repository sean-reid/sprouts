use crate::types::{BoundingBox, ComponentMetadata, GameState, Grid};
use std::collections::{HashMap, HashSet};

/// Flat-array union-find for efficient pixel-level component labeling.
pub struct UnionFind {
    parent: Vec<u32>,
    rank: Vec<u8>,
    width: usize,
}

impl UnionFind {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            parent: vec![u32::MAX; width * height],
            rank: vec![0; width * height],
            width,
        }
    }

    pub fn make_set(&mut self, x: usize, y: usize) {
        let i = y * self.width + x;
        self.parent[i] = i as u32;
    }

    pub fn find(&mut self, x: usize, y: usize) -> Option<u32> {
        let i = y * self.width + x;
        if self.parent[i] == u32::MAX {
            return None;
        }
        let root = self.find_by_idx(i);
        Some(root)
    }

    fn find_by_idx(&mut self, i: usize) -> u32 {
        // Iterative path compression (avoids stack overflow on long chains)
        let mut root = i;
        while self.parent[root] != root as u32 {
            root = self.parent[root] as usize;
        }
        let mut curr = i;
        while curr != root {
            let next = self.parent[curr] as usize;
            self.parent[curr] = root as u32;
            curr = next;
        }
        root as u32
    }

    pub fn union(&mut self, x1: usize, y1: usize, x2: usize, y2: usize) {
        let r1 = match self.find(x1, y1) {
            Some(r) => r as usize,
            None => return,
        };
        let r2 = match self.find(x2, y2) {
            Some(r) => r as usize,
            None => return,
        };
        if r1 == r2 {
            return;
        }
        if self.rank[r1] < self.rank[r2] {
            self.parent[r1] = r2 as u32;
        } else if self.rank[r1] > self.rank[r2] {
            self.parent[r2] = r1 as u32;
        } else {
            self.parent[r2] = r1 as u32;
            self.rank[r1] += 1;
        }
    }
}

pub fn label_components(
    skeleton: &Grid<bool>,
    state: &GameState,
) -> (Grid<u32>, HashMap<usize, ComponentMetadata>) {
    let w = skeleton.width;
    let h = skeleton.height;
    let mut uf = UnionFind::new(w, h);

    // Create sets for all skeleton pixels
    for y in 0..h {
        for x in 0..w {
            if skeleton.data[y * w + x] {
                uf.make_set(x, y);
            }
        }
    }

    // Union adjacent skeleton pixels (8-connectivity, only check already-visited neighbors)
    for y in 0..h {
        for x in 0..w {
            if !skeleton.data[y * w + x] {
                continue;
            }
            if x > 0 && y > 0 && skeleton.data[(y - 1) * w + (x - 1)] {
                uf.union(x, y, x - 1, y - 1);
            }
            if y > 0 && skeleton.data[(y - 1) * w + x] {
                uf.union(x, y, x, y - 1);
            }
            if x + 1 < w && y > 0 && skeleton.data[(y - 1) * w + (x + 1)] {
                uf.union(x, y, x + 1, y - 1);
            }
            if x > 0 && skeleton.data[y * w + (x - 1)] {
                uf.union(x, y, x - 1, y);
            }
        }
    }

    // Build component labels grid (0 = not skeleton, 1+ = component ID)
    let mut component_labels = Grid::new(w, h, 0u32);
    let mut root_to_id: HashMap<u32, usize> = HashMap::new();
    let mut next_id = 0usize;

    for y in 0..h {
        for x in 0..w {
            if !skeleton.data[y * w + x] {
                continue;
            }
            if let Some(root) = uf.find(x, y) {
                let id = *root_to_id.entry(root).or_insert_with(|| {
                    next_id += 1;
                    next_id // 1-based
                });
                component_labels.set(x as i32, y as i32, id as u32);
            }
        }
    }

    // Compute component metadata
    let mut components: HashMap<usize, ComponentMetadata> = HashMap::new();

    for y in 0..h {
        for x in 0..w {
            let comp_id = component_labels.data[y * w + x] as usize;
            if comp_id == 0 {
                continue;
            }
            let meta = components.entry(comp_id).or_insert_with(|| ComponentMetadata {
                pixel_count: 0,
                accessible_nodes: HashSet::new(),
                bounding_box: BoundingBox::new(),
                max_internal_distance: 0.0,
            });
            meta.pixel_count += 1;
            meta.bounding_box.update(x as i32, y as i32);
        }
    }

    // Assign nodes to the largest nearby component.
    for node in &state.nodes {
        let node_x = node.position.x.round() as i32;
        let node_y = node.position.y.round() as i32;

        let board_size_f = state.board_size as f64;
        let dist_to_left = node.position.x;
        let dist_to_right = board_size_f - node.position.x;
        let dist_to_top = node.position.y;
        let dist_to_bottom = board_size_f - node.position.y;
        let min_border_dist = dist_to_left
            .min(dist_to_right)
            .min(dist_to_top)
            .min(dist_to_bottom);

        let initial_search_radius: i32 = if min_border_dist < 15.0 {
            40
        } else if min_border_dist < 30.0 {
            30
        } else {
            20
        };

        let max_search_radius: i32 = if min_border_dist < 20.0 { 400 } else { 300 };

        let mut found_components: HashMap<usize, i32> = HashMap::new();

        // Phase 1: initial search box
        for dy in -initial_search_radius..=initial_search_radius {
            for dx in -initial_search_radius..=initial_search_radius {
                let px = node_x + dx;
                let py = node_y + dy;
                let comp_id = component_labels.get(px, py).copied().unwrap_or(0) as usize;
                if comp_id > 0 {
                    let dist = dx.abs().max(dy.abs());
                    found_components
                        .entry(comp_id)
                        .and_modify(|d| *d = (*d).min(dist))
                        .or_insert(dist);
                }
            }
        }

        // Phase 2: expand if nothing found
        if found_components.is_empty() {
            for radius in (initial_search_radius + 1)..=max_search_radius {
                for dy in -radius..=radius {
                    for dx in -radius..=radius {
                        if dx.abs() != radius && dy.abs() != radius {
                            continue;
                        }
                        let px = node_x + dx;
                        let py = node_y + dy;
                        let comp_id =
                            component_labels.get(px, py).copied().unwrap_or(0) as usize;
                        if comp_id > 0 {
                            found_components.entry(comp_id).or_insert(radius);
                        }
                    }
                }
                if !found_components.is_empty() {
                    break;
                }
            }
        }

        // Pick the best component: prefer the largest among those within
        // reasonable distance of the closest hit.
        let best_comp = if found_components.is_empty() {
            // Fallback: scan grid for closest component pixel
            let mut min_dist = f64::INFINITY;
            let mut closest = None;
            for y in 0..h {
                for x in 0..w {
                    let comp_id = component_labels.data[y * w + x] as usize;
                    if comp_id == 0 {
                        continue;
                    }
                    let dx = x as f64 - node.position.x;
                    let dy = y as f64 - node.position.y;
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist < min_dist {
                        min_dist = dist;
                        closest = Some(comp_id);
                    }
                }
            }
            closest
        } else {
            let min_dist = found_components.values().copied().min().unwrap_or(0);
            let threshold = (min_dist * 2).max(initial_search_radius);
            found_components
                .iter()
                .filter(|(_, &dist)| dist <= threshold)
                .max_by_key(|(comp_id, _)| {
                    components.get(comp_id).map(|m| m.pixel_count).unwrap_or(0)
                })
                .map(|(&comp_id, _)| comp_id)
        };

        if let Some(comp_id) = best_comp {
            if let Some(meta) = components.get_mut(&comp_id) {
                meta.accessible_nodes.insert(node.id);
            }
        }
    }

    // Compute max internal distances using bounding box diagonal (O(1) per component)
    for meta in components.values_mut() {
        if meta.accessible_nodes.len() == 1 {
            let bb = &meta.bounding_box;
            let dx = (bb.max_x - bb.min_x) as f64;
            let dy = (bb.max_y - bb.min_y) as f64;
            meta.max_internal_distance = (dx * dx + dy * dy).sqrt();
        }
    }

    (component_labels, components)
}
