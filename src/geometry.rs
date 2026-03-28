use crate::types::Point;

pub fn distance_squared(p1: &Point, p2: &Point) -> f64 {
    let dx = p1.x - p2.x;
    let dy = p1.y - p2.y;
    dx * dx + dy * dy
}

pub fn distance(p1: &Point, p2: &Point) -> f64 {
    distance_squared(p1, p2).sqrt()
}

pub fn polyline_length(points: &[Point]) -> f64 {
    let mut length = 0.0;
    for i in 1..points.len() {
        length += distance(&points[i - 1], &points[i]);
    }
    length
}

/// Find closest point on a polyline to a given point
pub fn closest_point_on_polyline(polyline: &[Point], target: &Point) -> (Point, f64) {
    if polyline.is_empty() {
        return (*target, 0.0);
    }
    let mut min_dist = f64::INFINITY;
    let mut closest = polyline[0];

    for i in 1..polyline.len() {
        let (pt, dist) = closest_point_on_segment(&polyline[i - 1], &polyline[i], target);
        if dist < min_dist {
            min_dist = dist;
            closest = pt;
        }
    }

    (closest, min_dist)
}

/// Find closest point on a line segment to a given point
pub fn closest_point_on_segment(a: &Point, b: &Point, p: &Point) -> (Point, f64) {
    let ab_x = b.x - a.x;
    let ab_y = b.y - a.y;
    let ap_x = p.x - a.x;
    let ap_y = p.y - a.y;

    let ab_len_sq = ab_x * ab_x + ab_y * ab_y;

    if ab_len_sq < 1e-10 {
        return (*a, distance(a, p));
    }

    let t = ((ap_x * ab_x + ap_y * ab_y) / ab_len_sq).clamp(0.0, 1.0);

    let closest = Point {
        x: a.x + t * ab_x,
        y: a.y + t * ab_y,
    };

    (closest, distance(&closest, p))
}

/// Check if two line segments intersect (excluding endpoints)
pub fn segments_intersect(a1: &Point, a2: &Point, b1: &Point, b2: &Point) -> bool {
    fn ccw(a: &Point, b: &Point, c: &Point) -> bool {
        (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x)
    }

    ccw(a1, b1, b2) != ccw(a2, b1, b2) && ccw(a1, a2, b1) != ccw(a1, a2, b2)
}

/// Check if a polyline intersects with any segment in a list
pub fn polyline_intersects_segments(
    polyline: &[Point],
    segments: &[(Point, Point)],
) -> bool {
    for i in 1..polyline.len() {
        let seg_a = &polyline[i - 1];
        let seg_b = &polyline[i];

        for (other_a, other_b) in segments {
            if segments_intersect(seg_a, seg_b, other_a, other_b) {
                return true;
            }
        }
    }
    false
}

/// Check if a point is on a polyline within tolerance
pub fn point_on_polyline(polyline: &[Point], point: &Point, tolerance: f64) -> bool {
    let (_, dist) = closest_point_on_polyline(polyline, point);
    dist <= tolerance
}

/// Extract a segment from a polyline starting at a given position for a given length
pub fn extract_line_segment(polyline: &[Point], start_pos: Point, length: f64) -> Vec<Point> {
    let mut result = vec![start_pos];
    let mut remaining = length;
    
    // Find the starting point in the polyline
    let mut start_idx = 0;
    let mut min_dist = f64::INFINITY;
    
    for i in 0..polyline.len() {
        let dist = distance(&polyline[i], &start_pos);
        if dist < min_dist {
            min_dist = dist;
            start_idx = i;
        }
    }
    
    // Walk along the polyline from start
    let mut current_idx = start_idx;
    
    while remaining > 0.0 && current_idx + 1 < polyline.len() {
        let seg_len = distance(&polyline[current_idx], &polyline[current_idx + 1]);
        
        if seg_len <= remaining {
            result.push(polyline[current_idx + 1]);
            remaining -= seg_len;
            current_idx += 1;
        } else {
            // Partial segment
            let t = remaining / seg_len;
            let x = polyline[current_idx].x + t * (polyline[current_idx + 1].x - polyline[current_idx].x);
            let y = polyline[current_idx].y + t * (polyline[current_idx + 1].y - polyline[current_idx].y);
            result.push(Point::new(x, y));
            break;
        }
    }
    
    result
}

/// Douglas-Peucker polyline decimation
pub fn decimate_polyline(points: &[Point], epsilon: f64) -> Vec<Point> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let mut max_dist = 0.0;
    let mut max_idx = 0;

    for i in 1..points.len() - 1 {
        let (_, dist) = closest_point_on_segment(&points[0], &points[points.len() - 1], &points[i]);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }

    if max_dist > epsilon {
        let left = decimate_polyline(&points[..=max_idx], epsilon);
        let right = decimate_polyline(&points[max_idx..], epsilon);

        let mut result = left;
        result.extend_from_slice(&right[1..]);
        result
    } else {
        vec![points[0], points[points.len() - 1]]
    }
}

/// Resample a polyline to have approximately uniform spacing
pub fn resample_polyline(points: &[Point], target_spacing: f64) -> Vec<Point> {
    if points.is_empty() {
        return vec![];
    }

    let mut result = vec![points[0]];
    let mut accumulated = 0.0;

    for i in 1..points.len() {
        let seg_len = distance(&points[i - 1], &points[i]);
        accumulated += seg_len;

        while accumulated >= target_spacing {
            // Interpolate point
            let excess = accumulated - target_spacing;
            let t = 1.0 - (excess / seg_len);
            let x = points[i - 1].x + t * (points[i].x - points[i - 1].x);
            let y = points[i - 1].y + t * (points[i].y - points[i - 1].y);
            result.push(Point::new(x, y));
            accumulated = excess;
        }
    }

    // Ensure last point is included
    if result.last().map_or(true, |p| distance(p, &points[points.len() - 1]) > 1.0) {
        result.push(points[points.len() - 1]);
    }

    result
}

/// Apply Chaikin's corner cutting algorithm for smooth organic curves
pub fn smooth_path_chaikin(points: &[Point], iterations: usize) -> Vec<Point> {
    if points.len() < 3 { return points.to_vec(); }
    let mut current = points.to_vec();
    
    // Store original endpoints to prevent "shrinking" away from nodes
    let start_node = points[0];
    let end_node = points[points.len() - 1];

    for _ in 0..iterations {
        let mut next = Vec::new();
        next.push(current[0]); // Keep start
        for i in 0..current.len() - 1 {
            let p0 = &current[i];
            let p1 = &current[i + 1];
            next.push(Point::new(0.75 * p0.x + 0.25 * p1.x, 0.75 * p0.y + 0.25 * p1.y));
            next.push(Point::new(0.25 * p0.x + 0.75 * p1.x, 0.25 * p0.y + 0.75 * p1.y));
        }
        next.push(current[current.len() - 1]); // Keep end
        current = next;
    }

    // Force exact endpoint alignment
    if let Some(first) = current.first_mut() { *first = start_node; }
    if let Some(last) = current.last_mut() { *last = end_node; }

    current
}

/// Remove doublebacks: only shortcut when the direct line saves significant
/// path length (>40%), preserving intentional curves.
pub fn shortcut_path(points: &[Point], distance_transform: &crate::types::Grid<u8>, min_clearance: u8) -> Vec<Point> {
    if points.len() <= 2 {
        return points.to_vec();
    }
    let mut result = vec![points[0]];
    let mut i = 0;
    while i < points.len() - 1 {
        let mut best_j = i + 1;
        // Try progressively larger skips
        for j in (i + 2..points.len()).rev() {
            let direct_dist = distance(&points[i], &points[j]);
            // Measure the sub-path length from i to j
            let mut sub_len = 0.0;
            for k in i..j {
                sub_len += distance(&points[k], &points[k + 1]);
            }
            // Only shortcut if the direct line saves >20% — this preserves
            // gentle curves (which add ~10% length) while eliminating
            // doublebacks and jagged detours.
            if direct_dist < sub_len * 0.8
                && line_clears_obstacles(&points[i], &points[j], distance_transform, min_clearance)
            {
                best_j = j;
                break;
            }
        }
        result.push(points[best_j]);
        i = best_j;
    }
    result
}

/// Check that every pixel along a straight line has sufficient clearance.
fn line_clears_obstacles(a: &Point, b: &Point, distance_transform: &crate::types::Grid<u8>, min_clearance: u8) -> bool {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1.0 { return true; }
    let steps = (len * 2.0).ceil() as usize; // sample every ~0.5px
    for s in 0..=steps {
        let t = s as f64 / steps as f64;
        let x = (a.x + t * dx).round() as i32;
        let y = (a.y + t * dy).round() as i32;
        let dist = distance_transform.get(x, y).copied().unwrap_or(0);
        if dist < min_clearance {
            return false;
        }
    }
    true
}
