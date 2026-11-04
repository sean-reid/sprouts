//! Geometry that rounds the same on every platform and build profile:
//! sums, products, divisions and square roots only, no hypot, trig or
//! pow, and a grid to round results to before they reach stored geometry.

use crate::types::Point;

/// Coordinates stored or compared across devices are rounded to this
/// many board units.
pub const GRID: f64 = 1.0 / 1_048_576.0;

/// `p` rounded to the grid.
pub fn snap_to_grid(p: Point) -> Point {
    Point::new((p.x / GRID).round() * GRID, (p.y / GRID).round() * GRID)
}

/// Length of the vector (dx, dy).
pub fn length(dx: f64, dy: f64) -> f64 {
    (dx * dx + dy * dy).sqrt()
}

pub fn distance_squared(a: Point, b: Point) -> f64 {
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)
}

pub fn distance(a: Point, b: Point) -> f64 {
    distance_squared(a, b).sqrt()
}

/// Screen-side test: positive when `q` is on the right of the heading
/// `o` to `p`, where y grows downward; zero when collinear.
pub fn cross(o: Point, p: Point, q: Point) -> f64 {
    (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x)
}

/// FNV-1a over f64 bits, for comparing geometry across platforms.
pub struct Fnv(pub u64);

impl Default for Fnv {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Fnv {
    pub fn feed(&mut self, x: f64) {
        for byte in x.to_bits().to_le_bytes() {
            self.0 ^= byte as u64;
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    pub fn points(&mut self, pts: &[Point]) {
        for p in pts {
            self.feed(p.x);
            self.feed(p.y);
        }
    }

    pub fn hex(&self) -> String {
        format!("{:#018x}", self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The helpers over a fixed grid of inputs hash to the value a Mac
    /// records; a platform rounding any of them differently fails here.
    const HELPERS_HASH: u64 = 0x0582_7e99_d8a9_f4b5;

    #[test]
    fn helpers_round_the_same_on_every_platform() {
        let mut fnv = Fnv::default();
        for i in 0..64 {
            let a = Point::new(i as f64 * 13.37 + 0.001, 1000.0 - i as f64 * 7.77);
            let b = Point::new(500.0 + (i as f64).sqrt() * 3.3, i as f64 * i as f64 / 11.0);
            let o = Point::new(333.3, 666.6);
            fnv.feed(length(a.x - b.x, a.y - b.y));
            fnv.feed(distance_squared(a, b));
            fnv.feed(distance(a, b));
            fnv.feed(cross(o, a, b));
            fnv.points(&[snap_to_grid(a), snap_to_grid(b)]);
        }
        assert_eq!(fnv.0, HELPERS_HASH, "helpers hash {}", fnv.hex());
    }
}
