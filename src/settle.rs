//! Relaxes the drawn geometry after each move so the board is as roomy as
//! its topology allows: nodes apart, lines off nodes and off each other,
//! everything in from the edge, with the face model held fixed.
//!
//! Each pair on the board has a clearance ratio, its distance over the
//! clearance the rules ask of it. Every round pushes apart the pairs whose
//! ratio sits within HEADROOM of the least one, by a stride that grows
//! while rounds stand and halves when one is refused. A round stands when
//! the least ratio rises, or holds while the pairs near it even out, and
//! every line still embeds to the move the model holds with no new
//! crossing. The new node of a line stays at a fixed parameter of its
//! curve, so moving the curve moves the node.

use crate::curve::{self, Cubic};
use crate::embedding;
use crate::face::Move as FaceMove;
use crate::geometry::{closest_point_on_segment, segment_intersection, segments_intersect};
use crate::types::{
    ai_node_clearance, grid, node_disc, scale, GameState, Line, Move, Point, MIN_NODE_SPACING,
    MIN_PATH_CLEARANCE,
};

/// Rounds per settle at most; a round that moves nothing ends it early.
pub const ROUNDS: usize = 64;
/// Pair distances measured plus lines carried, summed over the rounds,
/// at which a settle stops whatever the board. The model and crossing
/// checks each round are not counted; ROUNDS bounds them.
pub const WORK_CAP: usize = 2_000_000;
/// Share of a shortfall each side of a pair closes per round.
const STEP: f64 = 1.0;
/// Pairs this far above the least ratio are still pushed, so the board
/// evens out around its tightest spot rather than at it alone.
const HEADROOM: f64 = 1.0;
/// Longest move of one point in one round, in node spacings: where the
/// stride starts, and the most it grows to.
const STRIDE_START: f64 = 1.0;
pub const STRIDE_MAX: f64 = 2.0;
/// Growth of the stride after a round that stands.
const STRIDE_GROW: f64 = 1.5;
/// Curve samples this far apart, in line clearances.
const SAMPLE: f64 = 1.0;
/// Within this many line clearances of a node two lines meet there may
/// sit as close as they like.
const SHARED: f64 = 3.0;
/// Two samples of one line closer than this along it, in line clearances,
/// are neighbours rather than a hairpin.
const SELF_GAP: f64 = 3.0;
/// Two samples of one line count as a hairpin only when their distance is
/// under this share of their distance along it; a straight or gently
/// curved line has no hairpin.
const FOLD: f64 = 0.5;
const EPS: f64 = 1e-9;
const NEAREST_STEPS: usize = 64;
const NEAREST_REFINE: usize = 40;
/// Sine of the angle under which a join between two cubics counts as
/// smooth; wide enough that rounding the handles to the grid keeps a join
/// smooth from one settle to the next.
const SMOOTH_JOIN: f64 = 0.05;

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

/// What one pass over the board measured: the least clearance ratio, the
/// pair that set it, and the squared shortfalls below the level surveyed.
struct Survey {
    least: f64,
    at: Tightest,
    energy: f64,
    /// Pair distances measured.
    tests: usize,
}

/// Displacements gathered over one round: a sum per node, a weighted sum
/// per control point.
struct Push {
    nodes: Vec<Point>,
    ctrl: Vec<Vec<(f64, f64, f64)>>,
}

impl Push {
    fn empty(layout: &Layout) -> Self {
        Self {
            nodes: vec![Point::new(0.0, 0.0); layout.nodes.len()],
            ctrl: layout
                .lines
                .iter()
                .map(|c| vec![(0.0, 0.0, 0.0); c.pts.len()])
                .collect(),
        }
    }
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
    crate::exact::length(a.x, a.y)
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
        chain.read(&line.new_node_pos, targets);
        Some(chain)
    }

    /// Reads the node's place on the chain and which joins are smooth
    /// from the control points as they are, and resamples.
    fn read(&mut self, node: &Point, targets: &Targets) {
        let (seg, t) = self.locate(node);
        self.seg = seg;
        self.t = t;
        self.smooth = (1..self.segments())
            .map(|j| {
                let a = sub(self.pts[3 * j], self.pts[3 * j - 1]);
                let b = sub(self.pts[3 * j + 1], self.pts[3 * j]);
                let (la, lb) = (norm(a), norm(b));
                la > 0.0 && lb > 0.0 && (a.x * b.y - a.y * b.x).abs() / (la * lb) < SMOOTH_JOIN
            })
            .collect();
        self.resample(targets);
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

    /// Measures every pair on the board against `level` times its
    /// clearance, gathering the pushes to close the shortfalls when asked;
    /// an infinite level measures every pair and skips none.
    fn survey(&self, targets: &Targets, level: f64, mut push: Option<&mut Push>) -> Survey {
        let mut least = (f64::INFINITY, Tightest::Nothing);
        let mut energy = 0.0;
        let mut tests = 0;
        let mut short = |d: f64, target: f64, at: Tightest| -> Option<f64> {
            tests += 1;
            let ratio = d / target;
            if ratio < least.0 {
                least = (ratio, at);
            }
            if ratio < level {
                if level.is_finite() {
                    energy += (level - ratio) * (level - ratio);
                }
                Some(STEP * (level * target - d) / 2.0)
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

        // Samples and nodes outside a line's box, grown by the level's
        // clearance, are clear of it without a distance test; the work per
        // round is then bounded by the pairs that can be short.
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
                if outside(&p, boxes[k], level * targets.line_node) {
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
                    let arc = w[0].arc - s.arc;
                    if arc < gap {
                        continue;
                    }
                    if chain.from == chain.to && self.near_any(&w[0].at, &own, reach) {
                        continue;
                    }
                    let (q, d) = closest_point_on_segment(&w[0].at, &w[1].at, &s.at);
                    if d >= FOLD * arc {
                        continue;
                    }
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
                if boxes_apart(boxes[k], boxes[l], level * targets.line) {
                    continue;
                }
                let shared = self.shared(k, l);
                for (a, b) in [(k, l), (l, k)] {
                    for s in &self.lines[a].samples {
                        if outside(&s.at, boxes[b], level * targets.line)
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
        Survey {
            least: least.0,
            at: least.1,
            energy,
            tests,
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

    /// The guard a round must pass: no more crossings than the board had
    /// and every line still reading as its move.
    fn keeps(
        &self,
        state: &GameState,
        original: &[FaceMove],
        before: usize,
        crossed: usize,
    ) -> bool {
        crossed <= before && self.holds(state, original)
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

    /// Moves the free points by the gathered pushes, each capped at the
    /// stride; returns the longest move made.
    fn apply(&mut self, push: &Push, stride: f64) -> f64 {
        let mut longest: f64 = 0.0;
        let mut place = |p: &mut Point, delta: Point| {
            let len = norm(delta);
            let delta = if len > stride {
                mul(delta, stride / len)
            } else {
                delta
            };
            longest = longest.max(norm(delta));
            *p = add(*p, delta);
        };
        for i in 0..self.initial {
            place(&mut self.nodes[i], push.nodes[i]);
        }
        for (k, chain) in self.lines.iter_mut().enumerate() {
            let last = chain.pts.len() - 1;
            for idx in 1..last {
                let (fx, fy, w) = push.ctrl[k][idx];
                place(
                    &mut chain.pts[idx],
                    mul(Point::new(fx, fy), 1.0 / w.max(1.0)),
                );
            }
        }
        longest
    }

    /// Rounds every free point and made node to the grid; the end nodes
    /// already sit on it. Each made node keeps its place along its curve,
    /// so a step too short to change a cell changes nothing.
    fn quantise(&mut self, grid: f64, targets: &Targets) {
        for p in &mut self.nodes {
            *p = crate::exact::snap_to_unit(*p, grid);
        }
        for chain in &mut self.lines {
            for p in &mut chain.pts {
                *p = crate::exact::snap_to_unit(*p, grid);
            }
            chain.resample(targets);
        }
    }

    /// The longest distance any node or control point sits from where it
    /// is in `other`.
    fn farthest_from(&self, other: &Layout) -> f64 {
        let mut worst: f64 = 0.0;
        for (a, b) in self.nodes.iter().zip(&other.nodes) {
            worst = worst.max(a.distance_to(b));
        }
        for (a, b) in self.lines.iter().zip(&other.lines) {
            for (p, q) in a.pts.iter().zip(&b.pts) {
                worst = worst.max(p.distance_to(q));
            }
        }
        worst
    }

    /// Carries each line along with its end nodes, the control points
    /// sharing the two ends' moves by their place along the chain, keeps
    /// smooth joins smooth and puts every made node back on its curve, in
    /// move order so a line always ends on a node already placed. A line
    /// that has not moved keeps its node where the move put it.
    fn rebuild(&mut self, targets: &Targets, origin: &Layout) {
        for k in 0..self.lines.len() {
            let (from, to) = (self.nodes[self.lines[k].from], self.nodes[self.lines[k].to]);
            let id = self.mid(k);
            let chain = &mut self.lines[k];
            let last = chain.pts.len() - 1;
            let d0 = sub(from, chain.pts[0]);
            let dn = sub(to, chain.pts[last]);
            for (i, p) in chain.pts.iter_mut().enumerate() {
                let u = i as f64 / last as f64;
                *p = add(*p, add(mul(d0, 1.0 - u), mul(dn, u)));
            }
            chain.pts[0] = from;
            chain.pts[last] = to;
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
                chain.pts[3 * j - 1] = sub(joint, mul(dir, joint.distance_to(&a)));
                chain.pts[3 * j + 1] = add(joint, mul(dir, joint.distance_to(&b)));
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
        let survey = layout.survey(&targets, f64::INFINITY, None);
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
    /// Whether the settle moved nothing.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty() && self.lines.is_empty()
    }

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
/// curve as their stroke. A game at settle version 0 moves nothing and
/// records an empty settle, so the stack stays paired with the moves.
pub fn settle(state: &mut GameState) {
    let undo = if state.settle_version == 0 {
        Undo {
            board: state.lines.len(),
            ..Undo::default()
        }
    } else {
        relax(state, None)
    };
    state.settles.push(undo);
}

/// Settles like `settle` and returns a hash of the layout after every
/// round, accepted or refused, for finding where two platforms part.
#[cfg(test)]
pub(crate) fn settle_traced(state: &mut GameState) -> Vec<u64> {
    let mut trace = Vec::new();
    let undo = relax(state, Some(&mut trace));
    state.settles.push(undo);
    trace
}

/// A 64-bit fold of every coordinate of a layout.
#[cfg(test)]
fn layout_hash(layout: &Layout) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    let mut fold = |v: f64| h = (h ^ v.to_bits()).wrapping_mul(0x100000001b3);
    for p in &layout.nodes {
        fold(p.x);
        fold(p.y);
    }
    for chain in &layout.lines {
        for p in &chain.pts {
            fold(p.x);
            fold(p.y);
        }
    }
    h
}

fn relax(
    state: &mut GameState,
    #[allow(unused_mut, unused_variables)] mut trace: Option<&mut Vec<u64>>,
) -> Undo {
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
    let mut least = layout.survey(&targets, f64::INFINITY, None).least;
    if !least.is_finite() {
        return undo;
    }
    let mut crossings = layout.crossings(tolerance);
    let unit = MIN_NODE_SPACING * scale(state.board_size);
    let mut stride = STRIDE_START * unit;
    // A collapsed stride restarts after the least ratio rose, since a short
    // accepted step leaves it too small to try the long ones; a stride
    // under half a cell moves nothing, so it ends the cycle, and a step of
    // one cell that only evens the board shrinks the stride rather than
    // growing it, or the settle would crawl a cell a round.
    let cell = grid(state.board_size);
    let floor = cell / 2.0;
    let mut progressed = false;
    let mut work = 0usize;
    for _ in 0..ROUNDS {
        if work >= WORK_CAP {
            break;
        }
        let level = least + HEADROOM;
        let mut push = Push::empty(&layout);
        let now = layout.survey(&targets, level, Some(&mut push));
        let mut next = layout.clone();
        next.apply(&push, stride);
        next.rebuild(&targets, &origin);
        next.quantise(grid(state.board_size), &targets);
        // A step that rounds away to nothing counts as refused, and shorter
        // ones would round away too, so the stride collapses at once.
        let moved = next.farthest_from(&layout);
        let after = next.survey(&targets, level, None);
        work += now.tests + after.tests + state.lines.len();
        let roomier = after.least > least + EPS;
        let evener = after.least + EPS >= least && after.energy <= now.energy + EPS;
        let crossed = next.crossings(tolerance);
        #[cfg(test)]
        if let Some(trace) = trace.as_deref_mut() {
            trace.push(layout_hash(&next));
        }
        if moved > 0.0 && (roomier || evener) && next.keeps(state, &original, crossings, crossed) {
            layout = next;
            least = after.least;
            crossings = crossed;
            progressed |= roomier;
            stride = if roomier || moved > 1.5 * cell {
                (stride * STRIDE_GROW).min(STRIDE_MAX * unit)
            } else {
                stride / 2.0
            };
            if stride < floor {
                if !progressed {
                    break;
                }
                progressed = false;
                stride = STRIDE_START * unit;
            }
        } else {
            stride = if moved > 0.0 { stride / 2.0 } else { 0.0 };
            if stride < floor {
                if !progressed {
                    break;
                }
                progressed = false;
                stride = STRIDE_START * unit;
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

#[cfg(any(test, feature = "hash"))]
pub mod golden;

#[cfg(test)]
pub(crate) mod tests;
