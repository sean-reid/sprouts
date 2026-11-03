//! Relaxes the drawn geometry after each move so a crowded board keeps room
//! to draw: nodes apart, lines off nodes and off each other, everything in
//! from the edge, with the face model held fixed.
//!
//! Every node position and cubic control point is pushed by its shortfalls
//! for a fixed number of rounds. A round stands only when every line still
//! embeds to the move the model holds, no crossing appears and the least
//! clearance on the board does not fall. The new node of a line stays at a
//! fixed parameter of its curve, so moving the curve moves the node.

use crate::curve::{self, Cubic};
use crate::embedding;
use crate::face::Move as FaceMove;
use crate::geometry::{closest_point_on_segment, segment_intersection, segments_intersect};
use crate::types::{
    ai_node_clearance, node_disc, scale, GameState, Line, Move, Point, MIN_NODE_SPACING,
    MIN_PATH_CLEARANCE,
};

const ROUNDS: usize = 24;
/// Share of a shortfall each side of a pair closes per round.
const STEP: f64 = 0.5;
/// Longest move of one point in one round, in line clearances.
const STRIDE: f64 = 0.5;
/// Farthest any point moves in one settle, in node spacings.
const MAX_SHIFT: f64 = 1.0;
/// Curve samples this far apart, in line clearances.
const SAMPLE: f64 = 1.0;
/// Within this many line clearances of a node two lines meet there may
/// sit as close as they like.
const SHARED: f64 = 3.0;
/// Two samples of one line closer than this along it, in line clearances,
/// are neighbours rather than a hairpin.
const SELF_GAP: f64 = 3.0;
/// A round that moves nothing farther than this ends the settle.
const CONVERGED: f64 = 1e-3;
const NEAREST_STEPS: usize = 64;
const NEAREST_REFINE: usize = 40;
const SMOOTH_JOIN: f64 = 1e-6;

/// Distances the board is relaxed toward, in board units.
struct Targets {
    node: f64,
    line_node: f64,
    line: f64,
    edge_node: f64,
    edge_line: f64,
    board: f64,
}

impl Targets {
    fn for_board(board_size: usize) -> Self {
        let s = scale(board_size);
        Self {
            node: (MIN_NODE_SPACING * s).max(2.0 * node_disc(board_size)),
            line_node: ai_node_clearance(board_size),
            line: MIN_PATH_CLEARANCE * s,
            edge_node: node_disc(board_size),
            edge_line: MIN_PATH_CLEARANCE * s,
            board: board_size as f64,
        }
    }
}

#[derive(Clone, Copy)]
struct Sample {
    at: Point,
    seg: usize,
    t: f64,
    arc: f64,
}

/// One line as a chain of cubics: `pts` holds 3n + 1 control points, the
/// first and last being its end nodes.
#[derive(Clone)]
struct Chain {
    from: usize,
    to: usize,
    pts: Vec<Point>,
    seg: usize,
    t: f64,
    smooth: Vec<bool>,
    samples: Vec<Sample>,
}

#[derive(Clone)]
struct Layout {
    initial: usize,
    nodes: Vec<Point>,
    lines: Vec<Chain>,
}

/// A pair measured for clearance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Tightest {
    Nothing,
    Nodes(usize, usize),
    LineNode(usize, usize),
    Lines(usize, usize),
    Hairpin(usize),
    NodeEdge(usize),
    LineEdge(usize),
}

/// What one pass over the board measured: the least clearance as a share
/// of its target, the pair that set it, and every pair short of target.
struct Survey {
    least: f64,
    at: Tightest,
    short: Vec<Tightest>,
}

/// Displacements gathered over one round: a sum per node, a weighted sum
/// per control point.
struct Push {
    nodes: Vec<Point>,
    ctrl: Vec<Vec<(f64, f64, f64)>>,
}

fn sub(a: Point, b: Point) -> Point {
    Point::new(a.x - b.x, a.y - b.y)
}

fn add(a: Point, b: Point) -> Point {
    Point::new(a.x + b.x, a.y + b.y)
}

fn mul(a: Point, k: f64) -> Point {
    Point::new(a.x * k, a.y * k)
}

fn norm(a: Point) -> f64 {
    a.x.hypot(a.y)
}

fn bernstein(t: f64) -> [f64; 4] {
    let u = 1.0 - t;
    [u * u * u, 3.0 * t * u * u, 3.0 * t * t * u, t * t * t]
}

impl Chain {
    fn from_line(line: &crate::types::Line, targets: &Targets) -> Option<Self> {
        let first = line.segments.first()?;
        let mut pts = vec![first.p0];
        for c in &line.segments {
            pts.extend([c.p1, c.p2, c.p3]);
        }
        let mut chain = Self {
            from: line.from_node,
            to: line.to_node,
            pts,
            seg: 0,
            t: 0.0,
            smooth: Vec::new(),
            samples: Vec::new(),
        };
        let (seg, t) = chain.locate(&line.new_node_pos);
        chain.seg = seg;
        chain.t = t;
        chain.smooth = (1..line.segments.len())
            .map(|j| {
                let a = sub(chain.pts[3 * j], chain.pts[3 * j - 1]);
                let b = sub(chain.pts[3 * j + 1], chain.pts[3 * j]);
                let (la, lb) = (norm(a), norm(b));
                la > 0.0 && lb > 0.0 && (a.x * b.y - a.y * b.x).abs() / (la * lb) < SMOOTH_JOIN
            })
            .collect();
        chain.resample(targets);
        Some(chain)
    }

    fn segments(&self) -> usize {
        (self.pts.len() - 1) / 3
    }

    fn cubic(&self, j: usize) -> Cubic {
        Cubic {
            p0: self.pts[3 * j],
            p1: self.pts[3 * j + 1],
            p2: self.pts[3 * j + 2],
            p3: self.pts[3 * j + 3],
        }
    }

    fn cubics(&self) -> Vec<Cubic> {
        (0..self.segments()).map(|j| self.cubic(j)).collect()
    }

    fn eval(&self, seg: usize, t: f64) -> Point {
        self.cubic(seg).eval(t)
    }

    /// The cubic and parameter of the point of the chain nearest `p`.
    fn locate(&self, p: &Point) -> (usize, f64) {
        let mut best = (f64::INFINITY, 0, 0.0);
        for j in 0..self.segments() {
            let c = self.cubic(j);
            for i in 0..=NEAREST_STEPS {
                let t = i as f64 / NEAREST_STEPS as f64;
                let d = c.eval(t).distance_to(p);
                if d < best.0 {
                    best = (d, j, t);
                }
            }
        }
        let (_, j, coarse) = best;
        let c = self.cubic(j);
        let span = 1.0 / NEAREST_STEPS as f64;
        let (mut lo, mut hi) = ((coarse - span).max(0.0), (coarse + span).min(1.0));
        for _ in 0..NEAREST_REFINE {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            if c.eval(m1).distance_to(p) < c.eval(m2).distance_to(p) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        (j, (lo + hi) / 2.0)
    }

    fn resample(&mut self, targets: &Targets) {
        let spacing = SAMPLE * targets.line;
        let mut samples = Vec::new();
        let mut arc = 0.0;
        for j in 0..self.segments() {
            let c = self.cubic(j);
            let length =
                c.p0.distance_to(&c.p1) + c.p1.distance_to(&c.p2) + c.p2.distance_to(&c.p3);
            let steps = ((length / spacing).ceil() as usize).clamp(2, 64);
            for i in 0..=steps {
                if j > 0 && i == 0 {
                    continue;
                }
                let t = i as f64 / steps as f64;
                let at = c.eval(t);
                if let Some(last) = samples.last() {
                    let last: &Sample = last;
                    arc += last.at.distance_to(&at);
                }
                samples.push(Sample { at, seg: j, t, arc });
            }
        }
        self.samples = samples;
    }

    /// Nearest point of the sample polyline to `p`: the point, its
    /// distance, its cubic and parameter.
    fn nearest(&self, p: &Point) -> (Point, f64, usize, f64) {
        let mut best = (self.samples[0].at, f64::INFINITY, 0, 0.0);
        for w in self.samples.windows(2) {
            let (q, d) = closest_point_on_segment(&w[0].at, &w[1].at, p);
            if d < best.1 {
                let span = w[0].at.distance_to(&w[1].at);
                let u = if span > 0.0 {
                    q.distance_to(&w[0].at) / span
                } else {
                    0.0
                };
                let (seg, t) = if w[0].seg == w[1].seg {
                    (w[0].seg, w[0].t + (w[1].t - w[0].t) * u)
                } else if u < 0.5 {
                    (w[0].seg, w[0].t)
                } else {
                    (w[1].seg, w[1].t)
                };
                best = (q, d, seg, t);
            }
        }
        best
    }

    fn bounds(&self) -> (Point, Point) {
        let mut lo = Point::new(f64::INFINITY, f64::INFINITY);
        let mut hi = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for s in &self.samples {
            lo = Point::new(lo.x.min(s.at.x), lo.y.min(s.at.y));
            hi = Point::new(hi.x.max(s.at.x), hi.y.max(s.at.y));
        }
        (lo, hi)
    }
}

fn boxes_apart((alo, ahi): (Point, Point), (blo, bhi): (Point, Point), gap: f64) -> bool {
    alo.x > bhi.x + gap || blo.x > ahi.x + gap || alo.y > bhi.y + gap || blo.y > ahi.y + gap
}

impl Layout {
    fn from_state(state: &GameState, targets: &Targets) -> Option<Self> {
        let mut nodes = vec![Point::new(0.0, 0.0); state.nodes.len()];
        for node in &state.nodes {
            *nodes.get_mut(node.id)? = node.position;
        }
        let lines = state
            .lines
            .iter()
            .map(|line| Chain::from_line(line, targets))
            .collect::<Option<Vec<_>>>()?;
        Some(Self {
            initial: state.initial_node_count,
            nodes,
            lines,
        })
    }

    fn mid(&self, k: usize) -> usize {
        self.initial + k
    }

    /// Lands `f` on the control points under parameter `t` of cubic `seg`
    /// of line `k`; a share that lands on an end node moves that node.
    fn push_curve(&self, push: &mut Push, k: usize, seg: usize, t: f64, f: Point) {
        let chain = &self.lines[k];
        let last = chain.pts.len() - 1;
        let weights = bernstein(t);
        // Moving each control point by its weight moves the curve point by
        // the sum of the squares, so the push is scaled to land in full.
        let f = mul(f, 1.0 / weights.iter().map(|w| w * w).sum::<f64>());
        for (i, w) in weights.into_iter().enumerate() {
            if w < 1e-12 {
                continue;
            }
            let idx = 3 * seg + i;
            if idx == 0 {
                self.push_node(push, chain.from, mul(f, w));
            } else if idx == last {
                self.push_node(push, chain.to, mul(f, w));
            } else {
                let slot = &mut push.ctrl[k][idx];
                slot.0 += f.x * w;
                slot.1 += f.y * w;
                slot.2 += w;
            }
        }
    }

    /// A starting node takes the push itself; a node made by a move sits
    /// on its line, so the line takes it.
    fn push_node(&self, push: &mut Push, id: usize, f: Point) {
        if id < self.initial {
            push.nodes[id] = add(push.nodes[id], f);
        } else {
            let k = id - self.initial;
            let (seg, t) = (self.lines[k].seg, self.lines[k].t);
            self.push_curve(push, k, seg, t, f);
        }
    }

    /// Nodes two lines meet at, within whose reach they may sit close.
    fn shared(&self, k: usize, l: usize) -> Vec<usize> {
        let a = &self.lines[k];
        let b = &self.lines[l];
        [a.from, a.to, self.mid(k), b.from, b.to, self.mid(l)]
            .into_iter()
            .filter(|&v| [a.from, a.to, self.mid(k)].contains(&v))
            .filter(|&v| [b.from, b.to, self.mid(l)].contains(&v))
            .collect()
    }

    fn near_any(&self, p: &Point, ids: &[usize], radius: f64) -> bool {
        ids.iter().any(|&v| self.nodes[v].distance_to(p) < radius)
    }

    /// Measures every pair on the board, gathering the pushes to close the
    /// shortfalls when asked.
    fn survey(&self, targets: &Targets, mut push: Option<&mut Push>) -> Survey {
        let mut least = (f64::INFINITY, Tightest::Nothing);
        let mut shorts = Vec::new();
        let mut short = |d: f64, target: f64, at: Tightest| -> Option<f64> {
            if d / target < least.0 {
                least = (d / target, at);
            }
            if d < target {
                shorts.push(at);
                Some(STEP * (target - d) / 2.0)
            } else {
                None
            }
        };
        let away = |from: Point, to: Point, f: f64| -> Point {
            let d = sub(to, from);
            let len = norm(d);
            if len > 1e-12 {
                mul(d, f / len)
            } else {
                Point::new(f, 0.0)
            }
        };

        let n = self.nodes.len();
        for i in 0..n {
            for j in i + 1..n {
                let d = self.nodes[i].distance_to(&self.nodes[j]);
                if let (Some(f), Some(push)) = (
                    short(d, targets.node, Tightest::Nodes(i, j)),
                    push.as_deref_mut(),
                ) {
                    let dir = away(self.nodes[j], self.nodes[i], f);
                    self.push_node(push, i, dir);
                    self.push_node(push, j, mul(dir, -1.0));
                }
            }
            let p = self.nodes[i];
            for (d, dir) in edge_gaps(p, targets.board) {
                if let (Some(f), Some(push)) = (
                    short(d, targets.edge_node, Tightest::NodeEdge(i)),
                    push.as_deref_mut(),
                ) {
                    self.push_node(push, i, mul(dir, f));
                }
            }
        }

        // Samples and nodes outside a line's box, grown by the target, are
        // clear of it without a distance test; the work per round is then
        // bounded by the pairs that can be short.
        let boxes: Vec<(Point, Point)> = self.lines.iter().map(Chain::bounds).collect();
        let outside = |p: &Point, (lo, hi): (Point, Point), gap: f64| {
            p.x < lo.x - gap || p.x > hi.x + gap || p.y < lo.y - gap || p.y > hi.y + gap
        };
        for (k, chain) in self.lines.iter().enumerate() {
            for i in 0..n {
                if i == chain.from || i == chain.to || i == self.mid(k) {
                    continue;
                }
                let p = self.nodes[i];
                if outside(&p, boxes[k], targets.line_node) {
                    continue;
                }
                let (q, d, seg, t) = chain.nearest(&p);
                if let (Some(f), Some(push)) = (
                    short(d, targets.line_node, Tightest::LineNode(k, i)),
                    push.as_deref_mut(),
                ) {
                    let dir = away(q, p, f);
                    self.push_node(push, i, dir);
                    self.push_curve(push, k, seg, t, mul(dir, -1.0));
                }
            }
            for s in &chain.samples {
                for (d, dir) in edge_gaps(s.at, targets.board) {
                    if let (Some(f), Some(push)) = (
                        short(d, targets.edge_line, Tightest::LineEdge(k)),
                        push.as_deref_mut(),
                    ) {
                        self.push_curve(push, k, s.seg, s.t, mul(dir, f));
                    }
                }
            }
            let gap = SELF_GAP * targets.line;
            let reach = SHARED * targets.line;
            let own = [chain.from];
            for (i, s) in chain.samples.iter().enumerate() {
                if chain.from == chain.to && self.near_any(&s.at, &own, reach) {
                    continue;
                }
                for w in chain.samples[i + 1..].windows(2) {
                    if w[0].arc - s.arc < gap {
                        continue;
                    }
                    if chain.from == chain.to && self.near_any(&w[0].at, &own, reach) {
                        continue;
                    }
                    let (q, d) = closest_point_on_segment(&w[0].at, &w[1].at, &s.at);
                    if let (Some(f), Some(push)) = (
                        short(d, targets.line, Tightest::Hairpin(k)),
                        push.as_deref_mut(),
                    ) {
                        let dir = away(q, s.at, f);
                        self.push_curve(push, k, s.seg, s.t, dir);
                        self.push_curve(push, k, w[0].seg, w[0].t, mul(dir, -1.0));
                    }
                }
            }
        }

        let reach = SHARED * targets.line;
        for k in 0..self.lines.len() {
            for l in k + 1..self.lines.len() {
                if boxes_apart(boxes[k], boxes[l], targets.line) {
                    continue;
                }
                let shared = self.shared(k, l);
                for (a, b) in [(k, l), (l, k)] {
                    for s in &self.lines[a].samples {
                        if outside(&s.at, boxes[b], targets.line)
                            || self.near_any(&s.at, &shared, reach)
                        {
                            continue;
                        }
                        let (q, d, seg, t) = self.lines[b].nearest(&s.at);
                        if self.near_any(&q, &shared, reach) {
                            continue;
                        }
                        if let (Some(f), Some(push)) = (
                            short(d, targets.line, Tightest::Lines(k, l)),
                            push.as_deref_mut(),
                        ) {
                            let dir = away(q, s.at, f);
                            self.push_curve(push, a, s.seg, s.t, dir);
                            self.push_curve(push, b, seg, t, mul(dir, -1.0));
                        }
                    }
                }
            }
        }
        drop(short);
        shorts.sort_unstable();
        shorts.dedup();
        Survey {
            least: least.0,
            at: least.1,
            short: shorts,
        }
    }

    /// Crossings between the flattenings of distinct lines and within each
    /// line, not counting where lines meet at a node they share.
    fn crossings(&self, tolerance: f64) -> usize {
        let flats: Vec<Vec<Point>> = self
            .lines
            .iter()
            .map(|c| curve::flatten(&c.cubics(), tolerance))
            .collect();
        let touch = 2.0 * tolerance;
        let crosses = |a: &[Point], i: usize, b: &[Point], j: usize, meet: &[usize]| -> bool {
            segments_intersect(&a[i - 1], &a[i], &b[j - 1], &b[j])
                && segment_intersection(&a[i - 1], &a[i], &b[j - 1], &b[j])
                    .is_none_or(|at| !self.near_any(&at, meet, touch))
        };
        let boxes: Vec<(Point, Point)> = flats
            .iter()
            .map(|flat| {
                flat.iter().fold(
                    (
                        Point::new(f64::INFINITY, f64::INFINITY),
                        Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
                    ),
                    |(lo, hi), p| {
                        (
                            Point::new(lo.x.min(p.x), lo.y.min(p.y)),
                            Point::new(hi.x.max(p.x), hi.y.max(p.y)),
                        )
                    },
                )
            })
            .collect();
        let apart = |k: usize, l: usize| {
            let ((alo, ahi), (blo, bhi)) = (boxes[k], boxes[l]);
            alo.x > bhi.x || blo.x > ahi.x || alo.y > bhi.y || blo.y > ahi.y
        };
        let mut count = 0;
        for (k, a) in flats.iter().enumerate() {
            let own = [self.lines[k].from, self.lines[k].to];
            for i in 1..a.len() {
                for j in i + 2..a.len() {
                    if crosses(a, i, a, j, &own) {
                        count += 1;
                    }
                }
            }
            for (l, b) in flats.iter().enumerate().skip(k + 1) {
                if apart(k, l) {
                    continue;
                }
                let shared = self.shared(k, l);
                for i in 1..a.len() {
                    for j in 1..b.len() {
                        if crosses(a, i, b, j, &shared) {
                            count += 1;
                        }
                    }
                }
            }
        }
        count
    }

    /// Whether every line, replayed onto this geometry, still reads as the
    /// move the model holds for it.
    fn holds(&self, state: &GameState, original: &[FaceMove]) -> bool {
        let mut check =
            GameState::with_nodes(self.nodes[..self.initial].to_vec(), state.board_size);
        for (k, chain) in self.lines.iter().enumerate() {
            let mov = Move {
                from_node: chain.from,
                to_node: chain.to,
                polyline: Vec::new(),
                segments: chain.cubics(),
                new_node_pos: self.nodes[self.mid(k)],
                player: state.lines[k].player,
            };
            let topology = match embedding::embed(&check, &mov) {
                Ok(topology) if topology == original[k] => topology,
                _ => return false,
            };
            if check.apply_embedded(mov, &topology).is_err() {
                return false;
            }
        }
        true
    }

    /// Moves the free points by the gathered pushes, capped per round and
    /// against where the settle began; returns the longest move made.
    fn apply(&mut self, push: &Push, origin: &Layout, stride: f64, shift: f64) -> f64 {
        let mut longest: f64 = 0.0;
        let mut place = |p: &mut Point, from: Point, delta: Point| {
            let len = norm(delta);
            let delta = if len > stride {
                mul(delta, stride / len)
            } else {
                delta
            };
            let next = within(add(*p, delta), from, shift);
            longest = longest.max(next.distance_to(p));
            *p = next;
        };
        for i in 0..self.initial {
            place(&mut self.nodes[i], origin.nodes[i], push.nodes[i]);
        }
        for (k, chain) in self.lines.iter_mut().enumerate() {
            let last = chain.pts.len() - 1;
            for idx in 1..last {
                let (fx, fy, w) = push.ctrl[k][idx];
                let delta = mul(Point::new(fx, fy), 1.0 / w.max(1.0));
                place(&mut chain.pts[idx], origin.lines[k].pts[idx], delta);
            }
        }
        longest
    }

    /// Carries each line's ends along with its nodes, keeps smooth joins
    /// smooth and puts every made node back on its curve, in move order so
    /// a line always ends on a node already placed. A line that has not
    /// moved keeps its node where the move put it. The control points
    /// carried along stay within `shift` of where the settle began.
    fn rebuild(&mut self, targets: &Targets, origin: &Layout, shift: f64) {
        for k in 0..self.lines.len() {
            let (from, to) = (self.nodes[self.lines[k].from], self.nodes[self.lines[k].to]);
            let id = self.mid(k);
            let chain = &mut self.lines[k];
            let last = chain.pts.len() - 1;
            let d0 = sub(from, chain.pts[0]);
            chain.pts[0] = from;
            chain.pts[1] = within(add(chain.pts[1], d0), origin.lines[k].pts[1], shift);
            let dn = sub(to, chain.pts[last]);
            chain.pts[last] = to;
            chain.pts[last - 1] = within(
                add(chain.pts[last - 1], dn),
                origin.lines[k].pts[last - 1],
                shift,
            );
            for j in 1..chain.segments() {
                if !chain.smooth[j - 1] {
                    continue;
                }
                let joint = chain.pts[3 * j];
                let a = chain.pts[3 * j - 1];
                let b = chain.pts[3 * j + 1];
                let dir = sub(b, a);
                let len = norm(dir);
                if len < 1e-12 {
                    continue;
                }
                let dir = mul(dir, 1.0 / len);
                chain.pts[3 * j - 1] = within(
                    sub(joint, mul(dir, joint.distance_to(&a))),
                    origin.lines[k].pts[3 * j - 1],
                    shift,
                );
                chain.pts[3 * j + 1] = within(
                    add(joint, mul(dir, joint.distance_to(&b))),
                    origin.lines[k].pts[3 * j + 1],
                    shift,
                );
            }
            let mid = if chain.pts == origin.lines[k].pts {
                origin.nodes[id]
            } else {
                chain.eval(chain.seg, chain.t)
            };
            chain.resample(targets);
            self.nodes[id] = mid;
        }
    }
}

/// `p`, pulled back onto the circle of radius `shift` about `from` when
/// it lies outside it.
fn within(p: Point, from: Point, shift: f64) -> Point {
    let off = sub(p, from);
    let far = norm(off);
    if far > shift {
        add(from, mul(off, shift / far))
    } else {
        p
    }
}

/// Distance and inward direction to each of the four board edges.
fn edge_gaps(p: Point, board: f64) -> [(f64, Point); 4] {
    [
        (p.x, Point::new(1.0, 0.0)),
        (board - p.x, Point::new(-1.0, 0.0)),
        (p.y, Point::new(0.0, 1.0)),
        (board - p.y, Point::new(0.0, -1.0)),
    ]
}

/// The moves the model holds, in the order they were made.
fn journal(state: &GameState) -> Vec<FaceMove> {
    let mut face = state.face.clone();
    let mut out = Vec::new();
    while let Some(mv) = face.undo() {
        out.push(mv);
    }
    out.reverse();
    out
}

/// The least clearance on the board as a share of its target, and the
/// pair that sets it; infinite when there is nothing to measure.
#[cfg(any(test, feature = "bench"))]
pub fn least_clearance(state: &GameState) -> (f64, Tightest) {
    let targets = Targets::for_board(state.board_size);
    Layout::from_state(state, &targets).map_or((f64::INFINITY, Tightest::Nothing), |layout| {
        let survey = layout.survey(&targets, None);
        (survey.least, survey.at)
    })
}

/// What one settle changed, so an undo can put the board back as it was
/// before the move it settled.
#[derive(Clone, Debug, Default)]
pub struct Undo {
    /// Lines on the board when the settle ran; an undo of that move
    /// leaves one fewer.
    pub board: usize,
    nodes: Vec<(usize, Point)>,
    lines: Vec<(usize, Line)>,
}

impl Undo {
    /// Puts back every item the settle changed that the board still has.
    pub fn restore(self, state: &mut GameState) {
        for (id, position) in self.nodes {
            if let Some(node) = state.nodes.iter_mut().find(|n| n.id == id) {
                node.position = position;
            }
        }
        for (k, was) in self.lines {
            if let Some(line) = state.lines.get_mut(k) {
                line.segments = was.segments;
                line.polyline = was.polyline;
                line.flat = was.flat;
                line.new_node_pos = was.new_node_pos;
            }
        }
    }
}

/// Relaxes the position in place and records what moved on the state's
/// settle stack. Lines whose curve moves get the flattening of the new
/// curve as their stroke.
pub fn settle(state: &mut GameState) {
    let undo = relax(state);
    state.settles.push(undo);
}

fn relax(state: &mut GameState) -> Undo {
    let mut undo = Undo {
        board: state.lines.len(),
        ..Undo::default()
    };
    if state.lines.is_empty() {
        return undo;
    }
    let targets = Targets::for_board(state.board_size);
    let Some(origin) = Layout::from_state(state, &targets) else {
        return undo;
    };
    let original = journal(state);
    if original.len() != state.lines.len() {
        return undo;
    }
    let tolerance = curve::tolerance(state.board_size);
    let mut layout = origin.clone();
    let Survey {
        mut least,
        mut short,
        ..
    } = layout.survey(&targets, None);
    if least >= 1.0 {
        return undo;
    }
    let mut crossings = layout.crossings(tolerance);
    let mut stride = STRIDE * targets.line;
    let shift = MAX_SHIFT * MIN_NODE_SPACING * scale(state.board_size);
    for _ in 0..ROUNDS {
        let mut push = Push {
            nodes: vec![Point::new(0.0, 0.0); layout.nodes.len()],
            ctrl: layout
                .lines
                .iter()
                .map(|c| vec![(0.0, 0.0, 0.0); c.pts.len()])
                .collect(),
        };
        layout.survey(&targets, Some(&mut push));
        let mut next = layout.clone();
        let longest = next.apply(&push, &origin, stride, shift);
        next.rebuild(&targets, &origin, shift);
        let after = next.survey(&targets, None);
        let crossed = next.crossings(tolerance);
        // A pair at target stays at target: a round may close shortfalls
        // but not open one.
        let no_new_short = after.short.iter().all(|pair| short.contains(pair));
        if after.least + 1e-9 >= least
            && no_new_short
            && crossed <= crossings
            && next.holds(state, &original)
        {
            layout = next;
            least = after.least;
            short = after.short;
            crossings = crossed;
            if longest < CONVERGED {
                break;
            }
        } else {
            stride /= 2.0;
            if stride < CONVERGED {
                break;
            }
        }
    }
    let changed: Vec<bool> = layout
        .lines
        .iter()
        .zip(&origin.lines)
        .map(|(now, was)| now.pts != was.pts)
        .collect();
    for (k, chain) in layout.lines.iter().enumerate() {
        if !changed[k] {
            continue;
        }
        let line = &mut state.lines[k];
        undo.lines.push((k, line.clone()));
        line.segments = chain.cubics();
        line.flat = curve::flatten(&line.segments, tolerance);
        line.polyline = line.flat.clone();
        line.new_node_pos = layout.nodes[layout.mid(k)];
    }
    for node in &mut state.nodes {
        if node.id >= layout.initial && !changed[node.id - layout.initial] {
            continue;
        }
        let at = layout.nodes[node.id];
        if at != node.position {
            undo.nodes.push((node.id, node.position));
            node.position = at;
        }
    }
    undo
}

#[cfg(test)]
pub(crate) mod tests;
