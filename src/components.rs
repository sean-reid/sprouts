use crate::types::{BoundingBox, ComponentMetadata, GameState, Grid, PixelCoord};
use std::collections::{HashMap, HashSet};

pub struct UnionFind {
    parent: HashMap<PixelCoord, PixelCoord>,
    rank: HashMap<PixelCoord, usize>,
}

impl UnionFind {
    pub fn new() -> Self {
        Self {
            parent: HashMap::new(),
            rank: HashMap::new(),
        }
    }

    pub fn make_set(&mut self, coord: PixelCoord) {
        self.parent.insert(coord, coord);
        self.rank.insert(coord, 0);
    }

    pub fn find(&mut self, coord: PixelCoord) -> Option<PixelCoord> {
        if !self.parent.contains_key(&coord) {
            return None;
        }

        let parent = *self.parent.get(&coord)?;
        if parent != coord {
            let root = self.find(parent)?;
            self.parent.insert(coord, root);
            Some(root)
        } else {
            Some(coord)
        }
    }

    pub fn union(&mut self, a: PixelCoord, b: PixelCoord) {
        let root_a = match self.find(a) {
            Some(r) => r,
            None => return,
        };
        let root_b = match self.find(b) {
            Some(r) => r,
            None => return,
        };

        if root_a == root_b {
            return;
        }

        let rank_a = *self.rank.get(&root_a).unwrap_or(&0);
        let rank_b = *self.rank.get(&root_b).unwrap_or(&0);

        if rank_a < rank_b {
            self.parent.insert(root_a, root_b);
        } else if rank_a > rank_b {
            self.parent.insert(root_b, root_a);
        } else {
            self.parent.insert(root_b, root_a);
            self.rank.insert(root_a, rank_a + 1);
        }
    }
}

pub fn label_components(
    skeleton: &Grid<bool>,
    state: &GameState,
) -> (HashMap<PixelCoord, usize>, HashMap<usize, ComponentMetadata>) {
    let mut uf = UnionFind::new();

    // Create sets for all skeleton pixels
    for y in 0..skeleton.height {
        for x in 0..skeleton.width {
            if skeleton.data[y * skeleton.width + x] {
                let coord = PixelCoord::new(x as i32, y as i32);
                uf.make_set(coord);
            }
        }
    }

    // Union adjacent skeleton pixels (8-connectivity)
    for y in 0..skeleton.height {
        for x in 0..skeleton.width {
            if !skeleton.data[y * skeleton.width + x] {
                continue;
            }

            let current = PixelCoord::new(x as i32, y as i32);

            // Check 4 already-visited neighbors (top-left, top, top-right, left)
            let neighbors = [
                PixelCoord::new(x as i32 - 1, y as i32 - 1),
                PixelCoord::new(x as i32, y as i32 - 1),
                PixelCoord::new(x as i32 + 1, y as i32 - 1),
                PixelCoord::new(x as i32 - 1, y as i32),
            ];

            for neighbor in &neighbors {
                if skeleton.in_bounds(neighbor.x, neighbor.y)
                    && *skeleton.get(neighbor.x, neighbor.y).unwrap()
                {
                    uf.union(current, *neighbor);
                }
            }
        }
    }

    // Build component mapping
    let mut component_labels: HashMap<PixelCoord, usize> = HashMap::new();
    let mut root_to_id: HashMap<PixelCoord, usize> = HashMap::new();
    let mut next_id = 0;

    for y in 0..skeleton.height {
        for x in 0..skeleton.width {
            if !skeleton.data[y * skeleton.width + x] {
                continue;
            }

            let coord = PixelCoord::new(x as i32, y as i32);
            if let Some(root) = uf.find(coord) {
                let id = *root_to_id.entry(root).or_insert_with(|| {
                    let id = next_id;
                    next_id += 1;
                    id
                });
                component_labels.insert(coord, id);
            }
        }
    }

    // Compute component metadata
    let mut components: HashMap<usize, ComponentMetadata> = HashMap::new();

    for (coord, &comp_id) in &component_labels {
        let meta = components.entry(comp_id).or_insert_with(|| ComponentMetadata {
            pixel_count: 0,
            accessible_nodes: HashSet::new(),
            bounding_box: BoundingBox::new(),
            max_internal_distance: 0.0,
        });

        meta.pixel_count += 1;
        meta.bounding_box.update(coord.x, coord.y);
    }

    // Assign nodes to components
    for node in &state.nodes {
        let node_x = node.position.x.round() as i32;
        let node_y = node.position.y.round() as i32;

        // Search in progressively larger radii to find any nearby skeleton
        let mut assigned = false;
        
        // First try immediate vicinity (fast)
        for dy in -20..=20 {
            for dx in -20..=20 {
                let px = node_x + dx;
                let py = node_y + dy;

                if let Some(&comp_id) = component_labels.get(&PixelCoord::new(px, py)) {
                    if let Some(meta) = components.get_mut(&comp_id) {
                        meta.accessible_nodes.insert(node.id);
                        assigned = true;
                        break;
                    }
                }
            }
            if assigned {
                break;
            }
        }
        
        // If not found nearby, search in larger radius
        if !assigned {
            for radius in 21..=200 {  // Increased from 100 to 200
                for dy in -radius..=radius {
                    for dx in -radius..=radius {
                        // Only check perimeter for efficiency
                        if (dx as i32).abs() != radius && (dy as i32).abs() != radius {
                            continue;
                        }
                        
                        let px = node_x + dx;
                        let py = node_y + dy;

                        if let Some(&comp_id) = component_labels.get(&PixelCoord::new(px, py)) {
                            if let Some(meta) = components.get_mut(&comp_id) {
                                meta.accessible_nodes.insert(node.id);
                                assigned = true;
                                break;
                            }
                        }
                    }
                    if assigned {
                        break;
                    }
                }
                if assigned {
                    break;
                }
            }
        }

        // If node not found near skeleton, find closest component
        if !assigned {
            let mut min_dist = f64::INFINITY;
            let mut closest_comp = None;

            for (coord, &comp_id) in &component_labels {
                let dx = coord.x as f64 - node.position.x;
                let dy = coord.y as f64 - node.position.y;
                let dist = (dx * dx + dy * dy).sqrt();

                if dist < min_dist {
                    min_dist = dist;
                    closest_comp = Some(comp_id);
                }
            }

            if let Some(comp_id) = closest_comp {
                if let Some(meta) = components.get_mut(&comp_id) {
                    meta.accessible_nodes.insert(node.id);
                }
            }
        }
    }

    // Compute max internal distances for self-loop detection
    for (comp_id, meta) in &mut components {
        if meta.accessible_nodes.len() == 1 {
            // Single node component - compute max internal distance
            let pixels: Vec<_> = component_labels
                .iter()
                .filter(|(_, &id)| id == *comp_id)
                .map(|(coord, _)| *coord)
                .collect();

            let mut max_dist: f64 = 0.0;
            for i in 0..pixels.len() {
                for j in i + 1..pixels.len() {
                    let dx = (pixels[i].x - pixels[j].x) as f64;
                    let dy = (pixels[i].y - pixels[j].y) as f64;
                    let dist = (dx * dx + dy * dy).sqrt();
                    max_dist = max_dist.max(dist);
                }
            }
            meta.max_internal_distance = max_dist;
        }
    }

    (component_labels, components)
}
