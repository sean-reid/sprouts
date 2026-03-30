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

/// Distance from a point (px, py) to a line segment (x1,y1)-(x2,y2).
pub fn point_to_segment_distance(px: f64, py: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-10 {
        return ((px - x1).powi(2) + (py - y1).powi(2)).sqrt();
    }
    let t = ((px - x1) * dx + (py - y1) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let cx = x1 + t * dx;
    let cy = y1 + t * dy;
    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
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
    if result.last().is_none_or(|p| distance(p, &points[points.len() - 1]) > 1.0) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Point;

    fn p(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    // --- point_to_segment_distance ---

    #[test]
    fn point_on_segment() {
        let d = point_to_segment_distance(5.0, 0.0, 0.0, 0.0, 10.0, 0.0);
        assert!(d.abs() < 1e-9);
    }

    #[test]
    fn point_at_endpoint() {
        let d = point_to_segment_distance(0.0, 0.0, 0.0, 0.0, 10.0, 0.0);
        assert!(d.abs() < 1e-9);
    }

    #[test]
    fn point_perpendicular() {
        let d = point_to_segment_distance(5.0, 3.0, 0.0, 0.0, 10.0, 0.0);
        assert!((d - 3.0).abs() < 1e-9);
    }

    #[test]
    fn zero_length_segment() {
        let d = point_to_segment_distance(3.0, 4.0, 0.0, 0.0, 0.0, 0.0);
        assert!((d - 5.0).abs() < 1e-9);
    }

    // --- distance / distance_squared ---

    #[test]
    fn distance_basic() {
        assert!((distance(&p(0.0, 0.0), &p(3.0, 4.0)) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn distance_squared_basic() {
        assert!((distance_squared(&p(0.0, 0.0), &p(3.0, 4.0)) - 25.0).abs() < 1e-9);
    }

    // --- polyline_length ---

    #[test]
    fn polyline_length_empty() {
        assert!(polyline_length(&[]).abs() < 1e-9);
    }

    #[test]
    fn polyline_length_single_point() {
        assert!(polyline_length(&[p(1.0, 1.0)]).abs() < 1e-9);
    }

    #[test]
    fn polyline_length_multi_segment() {
        let pts = vec![p(0.0, 0.0), p(3.0, 0.0), p(3.0, 4.0)];
        assert!((polyline_length(&pts) - 7.0).abs() < 1e-9);
    }

    // --- closest_point_on_segment ---

    #[test]
    fn closest_at_start_endpoint() {
        let (pt, d) = closest_point_on_segment(&p(0.0, 0.0), &p(10.0, 0.0), &p(-5.0, 0.0));
        assert!((pt.x - 0.0).abs() < 1e-9);
        assert!((d - 5.0).abs() < 1e-9);
    }

    #[test]
    fn closest_at_end_endpoint() {
        let (pt, d) = closest_point_on_segment(&p(0.0, 0.0), &p(10.0, 0.0), &p(15.0, 0.0));
        assert!((pt.x - 10.0).abs() < 1e-9);
        assert!((d - 5.0).abs() < 1e-9);
    }

    #[test]
    fn closest_midpoint_perpendicular() {
        let (pt, d) = closest_point_on_segment(&p(0.0, 0.0), &p(10.0, 0.0), &p(5.0, 7.0));
        assert!((pt.x - 5.0).abs() < 1e-9);
        assert!((pt.y - 0.0).abs() < 1e-9);
        assert!((d - 7.0).abs() < 1e-9);
    }

    // --- segments_intersect ---

    #[test]
    fn crossing_segments() {
        assert!(segments_intersect(&p(0.0, 0.0), &p(10.0, 10.0), &p(0.0, 10.0), &p(10.0, 0.0)));
    }

    #[test]
    fn parallel_segments() {
        assert!(!segments_intersect(&p(0.0, 0.0), &p(10.0, 0.0), &p(0.0, 1.0), &p(10.0, 1.0)));
    }

    #[test]
    fn non_crossing_segments() {
        assert!(!segments_intersect(&p(0.0, 0.0), &p(1.0, 0.0), &p(2.0, 0.0), &p(3.0, 0.0)));
    }

    #[test]
    fn t_intersection() {
        // Vertical segment crossing horizontal at midpoint
        let result = segments_intersect(&p(0.0, 0.0), &p(10.0, 0.0), &p(5.0, -5.0), &p(5.0, 5.0));
        assert!(result);
    }

    // --- decimate_polyline ---

    #[test]
    fn decimate_straight_line() {
        let pts = vec![p(0.0, 0.0), p(1.0, 0.0), p(2.0, 0.0), p(3.0, 0.0)];
        let result = decimate_polyline(&pts, 0.1);
        assert_eq!(result.len(), 2);
        assert!((result[0].x - 0.0).abs() < 1e-9);
        assert!((result[1].x - 3.0).abs() < 1e-9);
    }

    #[test]
    fn decimate_l_shape_keeps_corner() {
        let pts = vec![p(0.0, 0.0), p(10.0, 0.0), p(10.0, 10.0)];
        let result = decimate_polyline(&pts, 1.0);
        assert!(result.len() >= 3); // corner must be kept
    }

    // --- resample_polyline ---

    #[test]
    fn resample_uniform_spacing() {
        let pts = vec![p(0.0, 0.0), p(100.0, 0.0)];
        let result = resample_polyline(&pts, 25.0);
        // Should produce approximately 5 points (0, 25, 50, 75, 100)
        assert!(result.len() >= 4);
        // Check spacing is roughly uniform
        for i in 1..result.len() - 1 {
            let d = distance(&result[i - 1], &result[i]);
            assert!((d - 25.0).abs() < 1.0);
        }
    }

    #[test]
    fn resample_empty() {
        assert!(resample_polyline(&[], 10.0).is_empty());
    }

    // --- smooth_path_chaikin ---

    #[test]
    fn chaikin_preserves_endpoints() {
        let pts = vec![p(0.0, 0.0), p(50.0, 50.0), p(100.0, 0.0)];
        let result = smooth_path_chaikin(&pts, 3);
        assert!((result.first().unwrap().x - 0.0).abs() < 1e-9);
        assert!((result.first().unwrap().y - 0.0).abs() < 1e-9);
        assert!((result.last().unwrap().x - 100.0).abs() < 1e-9);
        assert!((result.last().unwrap().y - 0.0).abs() < 1e-9);
    }

    #[test]
    fn chaikin_increases_point_count() {
        let pts = vec![p(0.0, 0.0), p(50.0, 50.0), p(100.0, 0.0)];
        let result = smooth_path_chaikin(&pts, 2);
        assert!(result.len() > pts.len());
    }

    #[test]
    fn chaikin_short_path_unchanged() {
        let pts = vec![p(0.0, 0.0), p(10.0, 10.0)];
        let result = smooth_path_chaikin(&pts, 3);
        assert_eq!(result.len(), 2);
    }
}
