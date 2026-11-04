use crate::exact::snap_to_grid;
use kurbo::{CubicBez, ParamCurve, ParamCurveDeriv, Vec2};
use serde::{Deserialize, Serialize};
use tsify::Tsify;

use crate::types::{scale, Point, MIN_PATH_CLEARANCE_AI};

// A corner is a turn sharper than 60 degrees between the stroke directions
// measured over CORNER_SPAN accuracies on either side, so jitter between
// close samples does not read as one.
const CORNER_COS: f64 = 0.5;
const CORNER_SPAN: f64 = 4.0;
const REFIT_ITERATIONS: usize = 4;
const TANGENT_WINDOW: usize = 2;
const TANGENT_SPAN: f64 = 4.0;
const NEAREST_SAMPLES: usize = 16;
const NEAREST_SPACING: f64 = 4.0;
// Sample counts come from page-supplied coordinates, so they are capped to
// keep a huge or non-finite point from turning into an endless loop.
const MAX_SAMPLES: usize = 4096;
const NEAREST_NEWTON_STEPS: usize = 8;

/// Fit accuracy and flattening tolerance for every curve on a board: a
/// quarter of the tightest clearance validation tests.
pub fn tolerance(board_size: usize) -> f64 {
    MIN_PATH_CLEARANCE_AI * scale(board_size) / 4.0
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Tsify)]
pub struct Cubic {
    pub p0: Point,
    pub p1: Point,
    pub p2: Point,
    pub p3: Point,
}

impl Cubic {
    pub fn line(a: Point, b: Point) -> Self {
        let at = |t: f64| Point::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        Self {
            p0: a,
            p1: at(1.0 / 3.0),
            p2: at(2.0 / 3.0),
            p3: b,
        }
    }

    pub fn eval(&self, t: f64) -> Point {
        from_kurbo(self.to_kurbo().eval(t))
    }

    fn to_kurbo(self) -> CubicBez {
        CubicBez::new(
            to_kurbo(self.p0),
            to_kurbo(self.p1),
            to_kurbo(self.p2),
            to_kurbo(self.p3),
        )
    }

    fn from_kurbo(c: CubicBez) -> Self {
        Self {
            p0: from_kurbo(c.p0),
            p1: from_kurbo(c.p1),
            p2: from_kurbo(c.p2),
            p3: from_kurbo(c.p3),
        }
    }
}

/// Fits a cubic spline to a polyline. Every vertex ends up within `accuracy`
/// of the spline and the spline within `accuracy` of the polyline; the
/// endpoints are kept exactly and joins inside a smooth run share a tangent.
pub fn fit(points: &[Point], accuracy: f64) -> Vec<Cubic> {
    let mut distinct: Vec<kurbo::Point> = Vec::with_capacity(points.len());
    for &p in points {
        let p = to_kurbo(p);
        if distinct.last().is_none_or(|last| last.distance(p) > 1e-9) {
            distinct.push(p);
        }
    }
    let mut out = Vec::new();
    match distinct.as_slice() {
        [] => {}
        [p] => out.push(Cubic::line(from_kurbo(*p), from_kurbo(*p))),
        pts => {
            let mut start = 0;
            for i in corners(pts, accuracy) {
                fit_run(&pts[start..=i], accuracy, &mut out);
                start = i;
            }
            fit_run(&pts[start..], accuracy, &mut out);
        }
    }
    // Control points live on the grid, so a stored curve refits to itself
    // on every platform.
    for c in &mut out {
        c.p0 = snap_to_grid(c.p0);
        c.p1 = snap_to_grid(c.p1);
        c.p2 = snap_to_grid(c.p2);
        c.p3 = snap_to_grid(c.p3);
    }
    out
}

// Indices of the vertices where the turn is sharpest among its neighbours
// and sharper than the corner threshold.
fn corners(pts: &[kurbo::Point], accuracy: f64) -> Vec<usize> {
    let span = CORNER_SPAN * accuracy;
    let turn: Vec<f64> = (0..pts.len()).map(|i| turn_cos(pts, i, span)).collect();
    (1..pts.len() - 1)
        .filter(|&i| turn[i] < CORNER_COS && turn[i] <= turn[i - 1] && turn[i] < turn[i + 1])
        .collect()
}

// Cosine of the turn at vertex i between the arm reaching back at least
// `span` and the arm reaching forward the same; 1 near the stroke ends,
// where an arm shorter than half the span would only measure jitter.
fn turn_cos(pts: &[kurbo::Point], i: usize, span: f64) -> f64 {
    let reach = |step: isize| {
        let mut j = i as isize;
        while (j + step) >= 0 && ((j + step) as usize) < pts.len() {
            j += step;
            if pts[j as usize].distance(pts[i]) >= span {
                break;
            }
        }
        pts[j as usize]
    };
    let u = pts[i] - reach(-1);
    let v = reach(1) - pts[i];
    if crate::exact::length(u.x, u.y) < span / 2.0 || crate::exact::length(v.x, v.y) < span / 2.0 {
        return 1.0;
    }
    u.dot(v) / (crate::exact::length(u.x, u.y) * crate::exact::length(v.x, v.y))
}

fn fit_run(pts: &[kurbo::Point], accuracy: f64, out: &mut Vec<Cubic>) {
    let span = TANGENT_SPAN * accuracy;
    let start_tangent = tangent_from(pts, 0, 1, span);
    let end_tangent = tangent_from(pts, pts.len() - 1, -1, span);
    fit_cubic(pts, start_tangent, end_tangent, span, accuracy, out);
}

// The principal direction of the points within `span` of the vertex, and
// at least the next two, so jitter on close samples does not tilt the
// tangent. Oriented along the stroke.
fn tangent_from(pts: &[kurbo::Point], at: usize, step: isize, span: f64) -> Vec2 {
    let mut window = vec![pts[at]];
    let mut i = at as isize + step;
    while i >= 0 && (i as usize) < pts.len() {
        let p = pts[i as usize];
        if window.len() > TANGENT_WINDOW && p.distance(pts[at]) > span {
            break;
        }
        window.push(p);
        i += step;
    }
    let n = window.len() as f64;
    let mean = window.iter().fold(Vec2::ZERO, |acc, p| acc + p.to_vec2()) / n;
    let (mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0);
    for p in &window {
        let d = p.to_vec2() - mean;
        sxx += d.x * d.x;
        sxy += d.x * d.y;
        syy += d.y * d.y;
    }
    // The major eigenvector of the covariance, without trigonometry.
    let half = (sxx - syy) * 0.5;
    let root = (half * half + sxy * sxy).sqrt();
    let raw = if half >= 0.0 {
        Vec2::new(half + root, sxy)
    } else {
        Vec2::new(sxy, root - half)
    };
    let len = crate::exact::length(raw.x, raw.y);
    let direction = if len > 0.0 {
        raw / len
    } else {
        Vec2::new(1.0, 0.0)
    };
    let toward = window[window.len() - 1] - window[0];
    if direction.dot(toward) < 0.0 {
        -direction
    } else {
        direction
    }
}

fn shared_tangent(pts: &[kurbo::Point], split: usize, span: f64) -> Vec2 {
    let back = tangent_from(pts, split, -1, span);
    let center = back - tangent_from(pts, split, 1, span);
    let len = crate::exact::length(center.x, center.y);
    if len * len < 1e-12 {
        back
    } else {
        center / len
    }
}

// Schneider's fit: least squares on chord-length parameters, a few Newton
// steps on the parameters when close, else split at the worst vertex with a
// shared tangent.
fn fit_cubic(
    pts: &[kurbo::Point],
    start_tangent: Vec2,
    end_tangent: Vec2,
    span: f64,
    accuracy: f64,
    out: &mut Vec<Cubic>,
) {
    if pts.len() == 2 {
        out.push(Cubic::line(from_kurbo(pts[0]), from_kurbo(pts[1])));
        return;
    }
    let mut u = chord_parameters(pts);
    let mut bez = least_squares(pts, &u, start_tangent, end_tangent);
    let (mut error, mut split) = vertex_error(pts, &bez, &u);
    if error >= accuracy && error < accuracy * 4.0 {
        for _ in 0..REFIT_ITERATIONS {
            reparameterize(pts, &mut u, &bez);
            bez = least_squares(pts, &u, start_tangent, end_tangent);
            (error, split) = vertex_error(pts, &bez, &u);
            if error < accuracy {
                break;
            }
        }
    }
    if error < accuracy {
        match stray_vertex(pts, &bez, accuracy) {
            None => {
                out.push(Cubic::from_kurbo(bez));
                return;
            }
            Some(i) => split = i,
        }
    }
    let center = shared_tangent(pts, split, span);
    fit_cubic(&pts[..=split], start_tangent, center, span, accuracy, out);
    fit_cubic(&pts[split..], -center, end_tangent, span, accuracy, out);
}

fn chord_parameters(pts: &[kurbo::Point]) -> Vec<f64> {
    let mut u = Vec::with_capacity(pts.len());
    u.push(0.0);
    for w in pts.windows(2) {
        u.push(u[u.len() - 1] + w[0].distance(w[1]));
    }
    let total = u[u.len() - 1];
    for v in &mut u {
        *v /= total;
    }
    u
}

fn bernstein(u: f64) -> [f64; 4] {
    let v = 1.0 - u;
    [v * v * v, 3.0 * u * v * v, 3.0 * u * u * v, u * u * u]
}

fn least_squares(pts: &[kurbo::Point], u: &[f64], t1: Vec2, t2: Vec2) -> CubicBez {
    let first = pts[0];
    let last = pts[pts.len() - 1];
    let (mut c00, mut c01, mut c11, mut x0, mut x1) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (p, &ui) in pts.iter().zip(u) {
        let b = bernstein(ui);
        let a0 = t1 * b[1];
        let a1 = t2 * b[2];
        c00 += a0.dot(a0);
        c01 += a0.dot(a1);
        c11 += a1.dot(a1);
        let rest = first.to_vec2() * (b[0] + b[1]) + last.to_vec2() * (b[2] + b[3]);
        let tmp = p.to_vec2() - rest;
        x0 += a0.dot(tmp);
        x1 += a1.dot(tmp);
    }
    let det = c00 * c11 - c01 * c01;
    let (mut alpha1, mut alpha2) = if det.abs() > 1e-12 {
        ((x0 * c11 - x1 * c01) / det, (c00 * x1 - c01 * x0) / det)
    } else {
        (0.0, 0.0)
    };
    let chord = first.distance(last);
    if alpha1 < 1e-6 * chord || alpha2 < 1e-6 * chord {
        alpha1 = chord / 3.0;
        alpha2 = chord / 3.0;
    }
    CubicBez::new(first, first + t1 * alpha1, last + t2 * alpha2, last)
}

// Distance from each vertex to the curve point Newton reaches from the
// vertex's own parameter: a true curve point, so never under the real
// distance, and linear in the stroke length.
fn vertex_error(pts: &[kurbo::Point], bez: &CubicBez, u: &[f64]) -> (f64, usize) {
    let d1 = bez.deriv();
    let dd = d1.deriv();
    let mut worst = (0.0, pts.len() / 2);
    for (i, p) in pts.iter().enumerate().take(pts.len() - 1).skip(1) {
        let t = newton_foot(bez, &d1, &dd, *p, u[i]);
        let d = bez.eval(t).distance(*p);
        if d > worst.0 {
            worst = (d, i);
        }
    }
    worst
}

// Newton on (c(t) - p) . c'(t) from a starting parameter.
fn newton_foot(
    bez: &CubicBez,
    d1: &kurbo::QuadBez,
    dd: &kurbo::Line,
    p: kurbo::Point,
    start: f64,
) -> f64 {
    let mut t = start;
    for _ in 0..NEAREST_NEWTON_STEPS {
        let q = bez.eval(t) - p;
        let v = d1.eval(t).to_vec2();
        let slope = v.hypot2() + q.dot(dd.eval(t).to_vec2());
        if slope.abs() < 1e-12 {
            break;
        }
        let next = (t - q.dot(v) / slope).clamp(0.0, 1.0);
        let moved = (next - t).abs();
        t = next;
        if moved < 1e-12 {
            break;
        }
    }
    t
}

// Samples every few units of the control polygon, then Newton on
// (c(t) - p) . c'(t) polishes every local minimum among them, so a basin
// between samples on a curve that bends back on itself is not lost.
fn nearest_on(bez: &CubicBez, p: kurbo::Point) -> (f64, kurbo::Point) {
    let length = bez.p0.distance(bez.p1) + bez.p1.distance(bez.p2) + bez.p2.distance(bez.p3);
    let samples = ((length / NEAREST_SPACING).ceil() as usize).clamp(NEAREST_SAMPLES, MAX_SAMPLES);
    let d2: Vec<f64> = (0..=samples)
        .map(|k| bez.eval(k as f64 / samples as f64).distance_squared(p))
        .collect();
    let d1 = bez.deriv();
    let dd = d1.deriv();
    let mut best: Option<(f64, f64)> = None;
    let mut consider = |t: f64, d2: f64| {
        if best.is_none_or(|(_, b)| d2 < b) {
            best = Some((t, d2));
        }
    };
    for k in 0..=samples {
        let before = if k > 0 { d2[k - 1] } else { f64::INFINITY };
        let after = if k < samples {
            d2[k + 1]
        } else {
            f64::INFINITY
        };
        if d2[k] > before || d2[k] > after {
            continue;
        }
        let start = k as f64 / samples as f64;
        consider(start, d2[k]);
        let t = newton_foot(bez, &d1, &dd, p, start);
        consider(t, bez.eval(t).distance_squared(p));
    }
    let (t, _) = best.expect("at least one sample is a local minimum");
    (t, bez.eval(t))
}

fn reparameterize(pts: &[kurbo::Point], u: &mut [f64], bez: &CubicBez) {
    let d1 = bez.deriv();
    let d2 = d1.deriv();
    for (p, ui) in pts.iter().zip(u.iter_mut()) {
        let q = bez.eval(*ui) - *p;
        let q1 = d1.eval(*ui).to_vec2();
        let q2 = d2.eval(*ui).to_vec2();
        let denominator = q1.hypot2() + q.dot(q2);
        if denominator.abs() > 1e-12 {
            *ui = (*ui - q.dot(q1) / denominator).clamp(0.0, 1.0);
        }
    }
}

// The vertex test says nothing about the curve between vertices, so the
// curve is sampled at a quarter of the accuracy and each sample measured
// against the polyline; the vertex nearest the worst sample is the split.
fn stray_vertex(pts: &[kurbo::Point], bez: &CubicBez, accuracy: f64) -> Option<usize> {
    let step = accuracy / 4.0;
    let bound = accuracy - step / 2.0;
    let length = bez.p0.distance(bez.p1) + bez.p1.distance(bez.p2) + bez.p2.distance(bez.p3);
    let samples = ((length / step).ceil() as usize).clamp(1, MAX_SAMPLES * 16);
    let mut cursor = 0;
    let mut worst: Option<(f64, kurbo::Point)> = None;
    for k in 0..=samples {
        let q = bez.eval(k as f64 / samples as f64);
        let d = polyline_distance(pts, q, &mut cursor, bound);
        if d > bound && worst.is_none_or(|(w, _)| d > w) {
            worst = Some((d, q));
        }
    }
    let (_, q) = worst?;
    (1..pts.len() - 1).min_by(|&a, &b| pts[a].distance(q).total_cmp(&pts[b].distance(q)))
}

// Samples walk along the curve, so the nearest polyline segment is almost
// always near the last one; a full scan only runs when the window misses.
fn polyline_distance(pts: &[kurbo::Point], p: kurbo::Point, cursor: &mut usize, bound: f64) -> f64 {
    let last = pts.len() - 2;
    let nearest = |range: std::ops::RangeInclusive<usize>| {
        range
            .map(|j| (segment_distance(pts[j], pts[j + 1], p), j))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .expect("a polyline has at least one segment")
    };
    let (d, j) = nearest(cursor.saturating_sub(2)..=(*cursor + 8).min(last));
    if d <= bound {
        *cursor = j;
        return d;
    }
    let (d, j) = nearest(0..=last);
    *cursor = j;
    d
}

fn segment_distance(a: kurbo::Point, b: kurbo::Point, p: kurbo::Point) -> f64 {
    let ab = b - a;
    let len2 = ab.hypot2();
    if len2 < 1e-18 {
        return a.distance(p);
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    (a + ab * t).distance(p)
}

/// Polyline within `tolerance` of the spline, starting at its first point.
pub fn flatten(segments: &[Cubic], tolerance: f64) -> Vec<Point> {
    let mut out = Vec::new();
    let Some(first) = segments.first() else {
        return out;
    };
    out.push(snap_to_grid(first.p0));
    for c in segments {
        flatten_cubic([c.p0, c.p1, c.p2, c.p3], tolerance * tolerance, 0, &mut out);
    }
    out
}

/// Deepest halving of one cubic while flattening.
const FLATTEN_DEPTH: u32 = 16;

fn flatten_cubic(c: [Point; 4], tolerance2: f64, depth: u32, out: &mut Vec<Point>) {
    let [p0, p1, p2, p3] = c;
    let off = |p: Point| {
        let (dx, dy) = (p3.x - p0.x, p3.y - p0.y);
        let len2 = dx * dx + dy * dy;
        let (ex, ey) = (p.x - p0.x, p.y - p0.y);
        if len2 == 0.0 {
            return ex * ex + ey * ey;
        }
        let cross = dx * ey - dy * ex;
        cross * cross / len2
    };
    let ends_close = (p3.x - p0.x) * (p3.x - p0.x) + (p3.y - p0.y) * (p3.y - p0.y) <= tolerance2;
    let flat = if ends_close {
        [p1, p2]
            .iter()
            .all(|p| (p.x - p0.x) * (p.x - p0.x) + (p.y - p0.y) * (p.y - p0.y) <= tolerance2)
    } else {
        off(p1) <= tolerance2
            && off(p2) <= tolerance2
            && within_chord(p0, p3, p1)
            && within_chord(p0, p3, p2)
    };
    if flat || depth == FLATTEN_DEPTH {
        out.push(snap_to_grid(p3));
        return;
    }
    let mid = |a: Point, b: Point| Point::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    let q0 = mid(p0, p1);
    let q1 = mid(p1, p2);
    let q2 = mid(p2, p3);
    let r0 = mid(q0, q1);
    let r1 = mid(q1, q2);
    let s = mid(r0, r1);
    flatten_cubic([p0, q0, r0, s], tolerance2, depth + 1, out);
    flatten_cubic([s, r1, q2, p3], tolerance2, depth + 1, out);
}

/// Whether `p` projects onto the chord from `a` to `b` rather than past
/// an end, so the chord distance is the curve's deviation.
fn within_chord(a: Point, b: Point, p: Point) -> bool {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let t = (p.x - a.x) * dx + (p.y - a.y) * dy;
    t >= 0.0 && t <= dx * dx + dy * dy
}

pub struct Nearest {
    pub point: Point,
    pub distance: f64,
}

/// Closest point on the spline to the target.
pub fn nearest(segments: &[Cubic], target: &Point) -> Option<Nearest> {
    let at = to_kurbo(*target);
    let mut best: Option<(f64, kurbo::Point)> = None;
    for c in segments {
        let (_, candidate) = nearest_on(&c.to_kurbo(), at);
        let d2 = candidate.distance_squared(at);
        if best.is_none_or(|(b, _)| d2 < b) {
            best = Some((d2, candidate));
        }
    }
    best.map(|(d2, point)| Nearest {
        point: from_kurbo(point),
        distance: d2.sqrt(),
    })
}

fn to_kurbo(p: Point) -> kurbo::Point {
    kurbo::Point::new(p.x, p.y)
}

fn from_kurbo(p: kurbo::Point) -> Point {
    Point::new(p.x, p.y)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::GRID;
    use crate::geometry::{closest_point_on_polyline, polyline_length};
    use proptest::prelude::*;

    fn p(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    const DENSE: usize = 2048;

    fn dense(segments: &[Cubic]) -> Vec<Point> {
        segments
            .iter()
            .flat_map(|c| (0..=DENSE).map(move |k| c.eval(k as f64 / DENSE as f64)))
            .collect()
    }

    // Brute force: the best of the dense samples on each cubic, then a
    // ternary search between its neighbours, independent of `nearest`.
    fn distance_to_curve(segments: &[Cubic], target: &Point) -> f64 {
        segments
            .iter()
            .map(|c| {
                let at = |t: f64| c.eval(t).distance_to(target);
                let k = (0..=DENSE)
                    .min_by(|&a, &b| {
                        at(a as f64 / DENSE as f64).total_cmp(&at(b as f64 / DENSE as f64))
                    })
                    .unwrap();
                let mut lo = k.saturating_sub(1) as f64 / DENSE as f64;
                let mut hi = (k + 1).min(DENSE) as f64 / DENSE as f64;
                for _ in 0..80 {
                    let m1 = lo + (hi - lo) / 3.0;
                    let m2 = hi - (hi - lo) / 3.0;
                    if at(m1) < at(m2) {
                        hi = m2;
                    } else {
                        lo = m1;
                    }
                }
                at((lo + hi) / 2.0)
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn max_vertex_deviation(points: &[Point], segments: &[Cubic]) -> f64 {
        points
            .iter()
            .map(|v| distance_to_curve(segments, v))
            .fold(0.0, f64::max)
    }

    fn max_curve_deviation(points: &[Point], segments: &[Cubic]) -> f64 {
        dense(segments)
            .iter()
            .map(|q| closest_point_on_polyline(points, q).1)
            .fold(0.0, f64::max)
    }

    fn arc(radius: f64, steps: usize) -> Vec<Point> {
        (0..=steps)
            .map(|i| {
                let a = std::f64::consts::PI * i as f64 / steps as f64;
                p(500.0 + radius * a.cos(), 500.0 + radius * a.sin())
            })
            .collect()
    }

    #[test]
    fn tolerance_is_a_quarter_of_the_ai_clearance_scaled() {
        assert!((tolerance(1000) - 1.5).abs() < 1e-12);
        assert!((tolerance(500) - 0.75).abs() < 1e-12);
    }

    #[test]
    fn a_straight_stroke_fits_to_one_straight_segment() {
        let points: Vec<Point> = (0..=30)
            .map(|i| p(100.0 + 20.0 * i as f64, 200.0 + 10.0 * i as f64))
            .collect();
        let segments = fit(&points, 1.5);
        assert_eq!(segments.len(), 1);
        let c = segments[0];
        assert_eq!(c.p0, points[0]);
        assert_eq!(c.p3, points[30]);
        for control in [c.p1, c.p2] {
            let (_, d) = closest_point_on_polyline(&points, &control);
            assert!(d < 1e-6, "control point {control:?} is {d} off the line");
        }
    }

    #[test]
    fn a_jittery_straight_stroke_still_fits_to_one_segment() {
        let points: Vec<Point> = (0..=40)
            .map(|i| {
                let wobble = if i % 2 == 0 { 0.6 } else { -0.6 };
                p(100.0 + 15.0 * i as f64, 300.0 + wobble)
            })
            .collect();
        let segments = fit(&points, 1.5);
        assert_eq!(segments.len(), 1);
        assert!(max_vertex_deviation(&points, &segments) <= 1.5);
    }

    // Samples a world pixel apart with half a pixel of noise, the spacing a
    // stroke has at 4x zoom.
    #[test]
    fn a_fine_noisy_stroke_does_not_shatter_into_a_polyline() {
        let mut seed: u64 = 0x2545_f491_4f6c_dd1d;
        let mut noise = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 11) as f64 / (1u64 << 53) as f64) - 0.5
        };
        let steps = 628;
        let points: Vec<Point> = (0..=steps)
            .map(|i| {
                let a = std::f64::consts::PI * i as f64 / steps as f64;
                p(
                    500.0 + 200.0 * a.cos() + noise(),
                    500.0 + 200.0 * a.sin() + noise(),
                )
            })
            .collect();
        let segments = fit(&points, 1.5);
        assert!(segments.len() <= 8, "{} segments", segments.len());
        assert!(max_vertex_deviation(&points, &segments) <= 1.5);
        assert!(max_curve_deviation(&points, &segments) <= 1.5);
    }

    // A thousand points a pixel apart with half a pixel of noise, the
    // longest stroke 4x zoom produces across the board. Prints the time so a
    // run with --nocapture gives the figure; the bound only catches a return
    // to quadratic work.
    #[test]
    fn fitting_a_thousand_point_noisy_stroke_stays_cheap() {
        let mut seed: u64 = 0x9e37_79b9_7f4a_7c15;
        let mut noise = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 11) as f64 / (1u64 << 53) as f64) - 0.5
        };
        let points: Vec<Point> = (0..1000)
            .map(|i| {
                let t = i as f64 / 999.0;
                p(
                    20.0 + 960.0 * t + noise(),
                    500.0 + 200.0 * (t * 4.0).sin() + noise(),
                )
            })
            .collect();
        let started = std::time::Instant::now();
        let segments = fit(&points, 1.5);
        let elapsed = started.elapsed();
        println!(
            "fit of 1000 points: {} segments in {elapsed:?}",
            segments.len()
        );
        assert!(max_vertex_deviation(&points, &segments) <= 1.5);
        assert!(elapsed.as_millis() < 250, "fit took {elapsed:?}");
    }

    #[test]
    fn absurd_coordinates_terminate() {
        let huge = fit(
            &[p(0.0, 0.0), p(1e300, 0.0), p(1e300, 1e300), p(0.0, 1e300)],
            1.5,
        );
        assert!(!huge.is_empty());
        let spike = vec![Cubic {
            p0: p(0.0, 0.0),
            p1: p(f64::INFINITY, 0.0),
            p2: p(f64::NAN, 0.0),
            p3: p(100.0, 0.0),
        }];
        nearest(&spike, &p(50.0, 10.0));
        flatten(&huge, 1.5);
    }

    #[test]
    fn a_semicircle_fits_in_a_few_segments_within_the_bound() {
        let points = arc(200.0, 60);
        let segments = fit(&points, 1.5);
        assert!(segments.len() <= 4, "{} segments", segments.len());
        assert!(max_vertex_deviation(&points, &segments) <= 1.5);
        assert!(max_curve_deviation(&points, &segments) <= 1.5);
    }

    #[test]
    fn a_sharp_corner_survives_the_fit() {
        let points = vec![p(100.0, 100.0), p(300.0, 100.0), p(300.0, 300.0)];
        let segments = fit(&points, 1.5);
        assert_eq!(segments.len(), 2);
        assert!(max_vertex_deviation(&points, &segments) <= 1.5);
        assert!(max_curve_deviation(&points, &segments) <= 1.5);
        assert!(distance_to_curve(&segments, &p(300.0, 100.0)) < 1e-6);
    }

    #[test]
    fn degenerate_input_does_not_panic() {
        assert!(fit(&[], 1.5).is_empty());
        let dot = fit(&[p(5.0, 5.0), p(5.0, 5.0)], 1.5);
        assert_eq!(dot.len(), 1);
        assert_eq!(flatten(&dot, 1.5).len(), 2);
        assert!(polyline_length(&flatten(&dot, 1.5)) < 1e-9);
        let hit = nearest(&dot, &p(0.0, 0.0)).unwrap();
        assert_eq!(hit.point, p(5.0, 5.0));
        assert!((hit.distance - 50f64.sqrt()).abs() < 1e-9);
        assert!(nearest(&[], &p(0.0, 0.0)).is_none());
    }

    #[test]
    fn segments_join_end_to_start_with_a_shared_tangent() {
        let points: Vec<Point> = (0..=80)
            .map(|i| {
                let t = i as f64 / 80.0;
                p(100.0 + 800.0 * t, 500.0 + 150.0 * (t * 9.0).sin())
            })
            .collect();
        let segments = fit(&points, 1.5);
        assert!(segments.len() > 1);
        for pair in segments.windows(2) {
            assert_eq!(pair[0].p3, pair[1].p0);
            let incoming = Vec2::new(pair[0].p3.x - pair[0].p2.x, pair[0].p3.y - pair[0].p2.y);
            let outgoing = Vec2::new(pair[1].p1.x - pair[1].p0.x, pair[1].p1.y - pair[1].p0.y);
            let cross = incoming.normalize().cross(outgoing.normalize());
            // The handles sit on the grid, a hair off the shared tangent.
            assert!(cross.abs() < 1e-5, "tangent break of {cross} at a join");
        }
    }

    // Validation measures against the flattening, so every point of the true
    // curve has to lie within the tolerance of it.
    #[test]
    fn the_true_curve_stays_within_the_tolerance_of_its_flattening() {
        let segments = fit(&arc(200.0, 60), 1.5);
        for tol in [0.1, 1.5, 6.0] {
            let flat = flatten(&segments, tol);
            assert_eq!(flat[0], segments[0].p0);
            assert_eq!(*flat.last().unwrap(), segments.last().unwrap().p3);
            for q in dense(&segments) {
                let (_, d) = closest_point_on_polyline(&flat, &q);
                assert!(
                    d <= tol,
                    "{q:?} is {d} off the flattening at tolerance {tol}"
                );
            }
        }
        assert!(flatten(&segments, 0.1).len() > flatten(&segments, 6.0).len());
    }

    #[test]
    fn nearest_finds_the_right_basin_on_a_curve_that_bends_back() {
        let segments = vec![Cubic {
            p0: p(0.0, 0.0),
            p1: p(400.0, 200.0),
            p2: p(-100.0, 200.0),
            p3: p(300.0, 0.0),
        }];
        for gx in 0..17 {
            for gy in 0..13 {
                let probe = p(-50.0 + 25.0 * gx as f64, -50.0 + 25.0 * gy as f64);
                let hit = nearest(&segments, &probe).unwrap();
                let truth = distance_to_curve(&segments, &probe);
                assert!(
                    (hit.distance - truth).abs() < 1e-6,
                    "{probe:?}: nearest {} vs brute force {truth}",
                    hit.distance
                );
            }
        }
    }

    #[test]
    fn nearest_returns_the_foot_of_the_perpendicular() {
        let segments = vec![Cubic::line(p(0.0, 0.0), p(100.0, 0.0))];
        let hit = nearest(&segments, &p(40.0, 30.0)).unwrap();
        assert!((hit.point.x - 40.0).abs() < 1e-6);
        assert!(hit.point.y.abs() < 1e-6);
        assert!((hit.distance - 30.0).abs() < 1e-6);
    }

    fn strokes() -> impl Strategy<Value = Vec<Point>> {
        (
            3usize..60,
            0.0..1000.0f64,
            0.0..1000.0f64,
            0.0..std::f64::consts::TAU,
            proptest::collection::vec((-0.6..0.6f64, 0.8..14.0f64, -1.5..1.5f64), 60),
        )
            .prop_map(|(n, x, y, heading, steps)| {
                let mut points = vec![p(x, y)];
                let mut heading = heading;
                for (turn, step, jitter) in steps.into_iter().take(n - 1) {
                    heading += turn;
                    let last = points[points.len() - 1];
                    let (nx, ny) = (-heading.sin(), heading.cos());
                    points.push(p(
                        last.x + heading.cos() * step + nx * jitter,
                        last.y + heading.sin() * step + ny * jitter,
                    ));
                }
                points
            })
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(300))]

        #[test]
        fn the_fit_never_strays_past_its_bound(points in strokes(), accuracy in prop_oneof![Just(0.75), Just(1.5), Just(3.0)]) {
            let segments = fit(&points, accuracy);
            prop_assert!(!segments.is_empty());
            prop_assert_eq!(segments[0].p0, snap_to_grid(points[0]));
            prop_assert_eq!(
                segments[segments.len() - 1].p3,
                snap_to_grid(points[points.len() - 1])
            );
            for pair in segments.windows(2) {
                prop_assert_eq!(pair[0].p3, pair[1].p0);
            }
            let vertex = max_vertex_deviation(&points, &segments);
            // Control points on the grid move the curve by under a grid step.
            prop_assert!(vertex <= accuracy + GRID, "vertex deviation {} > {}", vertex, accuracy);
            let curve = max_curve_deviation(&points, &segments);
            prop_assert!(curve <= accuracy + GRID, "curve deviation {} > {}", curve, accuracy);
        }
    }
}
