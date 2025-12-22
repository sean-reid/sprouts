use crate::types::Point;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct GridCoord {
    x: i32,
    y: i32,
}

pub struct SpatialIndex {
    grid_size: f64,
    cells: HashMap<GridCoord, Vec<usize>>,
}

impl SpatialIndex {
    pub fn new(grid_size: f64) -> Self {
        Self {
            grid_size,
            cells: HashMap::new(),
        }
    }

    pub fn insert_segment(&mut self, segment_id: usize, p1: &Point, p2: &Point) {
        let cells = self.get_cells_for_segment(p1, p2);
        for cell in cells {
            self.cells.entry(cell).or_insert_with(Vec::new).push(segment_id);
        }
    }

    pub fn query_nearby_segments(&self, p1: &Point, p2: &Point) -> Vec<usize> {
        let cells = self.get_cells_for_segment(p1, p2);
        let mut segments = Vec::new();

        for cell in cells {
            if let Some(cell_segments) = self.cells.get(&cell) {
                segments.extend_from_slice(cell_segments);
            }
        }

        // Remove duplicates
        segments.sort();
        segments.dedup();
        segments
    }

    fn get_cells_for_segment(&self, p1: &Point, p2: &Point) -> Vec<GridCoord> {
        let mut cells = Vec::new();

        let x1 = (p1.x / self.grid_size).floor() as i32;
        let y1 = (p1.y / self.grid_size).floor() as i32;
        let x2 = (p2.x / self.grid_size).floor() as i32;
        let y2 = (p2.y / self.grid_size).floor() as i32;

        let min_x = x1.min(x2);
        let max_x = x1.max(x2);
        let min_y = y1.min(y2);
        let max_y = y1.max(y2);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                cells.push(GridCoord { x, y });
            }
        }

        cells
    }
}

pub fn build_spatial_index(lines: &[crate::types::Line]) -> SpatialIndex {
    let mut index = SpatialIndex::new(50.0);

    for line in lines {
        for i in 1..line.polyline.len() {
            index.insert_segment(line.id, &line.polyline[i - 1], &line.polyline[i]);
        }
    }

    index
}
