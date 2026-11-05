//! A seeded random game at each settle version, hashed after every move,
//! so one number says whether a build settles as macOS did when the
//! values below were recorded. Only sums, products, divisions and square
//! roots go into the strokes, which every platform rounds the same.

#[cfg(test)]
use crate::curve;
use crate::protocol::{self, Outcome};
#[cfg(test)]
use crate::settle;
use crate::types::{GameState, Player, Point};

/// The settled board of the seeded 7-node game under version 1, which
/// records and links at that version replay through. A change means
/// version 1 settles differently, which no change may do.
#[cfg(test)]
pub const VERSION_1_GOLDEN: u64 = 0xd77fd8c18d5f65f0;

/// The settled board of the seeded 7-node game under the current version.
/// A change means the settle moved: bump SETTLE_VERSION in
/// src/protocol.rs, keep this version's settle for its records, and
/// record the new value under the new name.
#[cfg(test)]
pub const VERSION_2_GOLDEN: u64 = 0x137fff0f11fe5435;

/// The settled board after each move of the seeded 9-node game under
/// version 1, recorded on macOS. A mismatch names the first move another
/// platform settles differently.
#[cfg(test)]
pub const VERSION_1_SEEDED_GAME: &[u64] = &[
    0x3d7fa941984d9c88,
    0xe8949eda727182fd,
    0x30c2983333a567da,
    0x04754f96acb3953b,
    0x80cb3662d3eac334,
    0xbb3cc3744bc2c9a0,
    0xf1d0c3108dcd6725,
    0x0777b6443ff308e3,
    0x339c593f9e2c0ec9,
];

/// The settled board after each move of the seeded 9-node game under the
/// current version, recorded on macOS.
#[cfg(test)]
pub const VERSION_2_SEEDED_GAME: &[u64] = &[
    0x2343da5810eafb04,
    0x611d4db5d89dd035,
    0x6261118092e6dcde,
    0x8f113bc531349297,
    0xfa061dda0d14287e,
    0x4a5b42ff59118015,
    0x035b5e369557e961,
    0xe855b1d6029d60ec,
    0xe260c75be34ba54c,
];

/// Picks for a random game from a small multiplicative generator, so the
/// games are fixed across runs.
pub fn seeded_picks(seed: u64, count: usize) -> Vec<(usize, usize, f64, f64)> {
    let mut x = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    let mut next = || {
        x = x
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (x >> 11) as f64 / (1u64 << 53) as f64
    };
    (0..count)
        .map(|_| {
            let a = (next() * 64.0) as usize;
            let b = (next() * 64.0) as usize;
            (a, b, next(), next())
        })
        .collect()
}

/// Forty unit directions a fortieth of a turn apart, from one rotation of
/// fixed cosine and sine, so no platform's sine or cosine is involved.
fn compass() -> Vec<Point> {
    const C: f64 = 0.987_688_340_595_137_8;
    const S: f64 = 0.156_434_465_040_230_87;
    let mut out = Vec::with_capacity(40);
    let mut d = Point::new(-1.0, 0.0);
    for _ in 0..40 {
        out.push(d);
        d = Point::new(d.x * C - d.y * S, d.x * S + d.y * C);
    }
    out
}

/// A chord bent through a control point, or a loop round a node, from
/// the indices a random game picks, as the page would send it; none when
/// the engine refuses it.
pub fn random_move(state: &GameState, pick: (usize, usize, f64, f64)) -> Option<protocol::Move> {
    let n = state.nodes.len();
    let (a, b, bend, side) = (pick.0 % n, pick.1 % n, pick.2, pick.3);
    let from = state.nodes[a].position;
    let to = state.nodes[b].position;
    let polyline: Vec<Point> = if a == b {
        let radius = 30.0 + 40.0 * bend;
        let compass = compass();
        let start = (side * 40.0) as usize % 40;
        (0..=40)
            .map(|i| {
                let d = compass[(start + i) % 40];
                Point::new(from.x + radius + radius * d.x, from.y + radius * d.y)
            })
            .collect()
    } else {
        let d = Point::new(to.x - from.x, to.y - from.y);
        let len = (d.x * d.x + d.y * d.y).sqrt();
        let normal = Point::new(-d.y / len, d.x / len);
        let bulge = (side - 0.5) * 2.0 * bend * len * 0.6;
        let ctrl = Point::new(
            (from.x + to.x) / 2.0 + normal.x * bulge,
            (from.y + to.y) / 2.0 + normal.y * bulge,
        );
        (0..=40)
            .map(|i| {
                let t = i as f64 / 40.0;
                let u = 1.0 - t;
                Point::new(
                    u * u * from.x + 2.0 * u * t * ctrl.x + t * t * to.x,
                    u * u * from.y + 2.0 * u * t * ctrl.y + t * t * to.y,
                )
            })
            .collect()
    };
    let wire = protocol::Move {
        from_node: a,
        to_node: b,
        polyline: polyline.clone(),
        new_node_pos: polyline[20],
        segments: None,
    };
    let entered = protocol::enter(wire.clone(), state).ok()?;
    protocol::check(state, &entered).ok()?;
    Some(wire)
}

/// A fresh 1000 board of `nodes` nodes settling at `version`, the human
/// to move.
fn board(nodes: usize, version: u8) -> GameState {
    let mut state =
        GameState::with_nodes_at(crate::types::opening_layout(nodes, 1000), 1000, version);
    state.current_player = Player::Human;
    state
}

/// A random game of `nodes` nodes played through the protocol, settling
/// after each move at `version`, with the moves as the engine applied
/// them.
#[cfg(test)]
pub fn play_random(
    nodes: usize,
    picks: &[(usize, usize, f64, f64)],
    version: u8,
) -> (GameState, Vec<protocol::Move>) {
    let mut state = board(nodes, version);
    let mut played = Vec::new();
    for pick in picks {
        if state.face.is_game_over() {
            break;
        }
        if let Some(mov) = random_move(&state, *pick) {
            if let Outcome::Applied { mov, .. } = protocol::apply(&mut state, mov) {
                played.push(mov);
            }
        }
    }
    (state, played)
}

/// Every coordinate on the board, bit for bit.
pub fn bits(state: &GameState) -> Vec<u64> {
    let mut out = Vec::new();
    for node in &state.nodes {
        out.extend([node.position.x.to_bits(), node.position.y.to_bits()]);
    }
    for line in &state.lines {
        for c in &line.segments {
            for p in [c.p0, c.p1, c.p2, c.p3] {
                out.extend([p.x.to_bits(), p.y.to_bits()]);
            }
        }
        out.extend([line.new_node_pos.x.to_bits(), line.new_node_pos.y.to_bits()]);
        for p in &line.polyline {
            out.extend([p.x.to_bits(), p.y.to_bits()]);
        }
    }
    out
}

/// FNV-1a over every coordinate on the board, through the shared hasher.
pub fn hash(state: &GameState) -> u64 {
    let mut fnv = crate::exact::Fnv::default();
    for b in bits(state) {
        fnv.feed(f64::from_bits(b));
    }
    fnv.0
}

/// The settled board after each move of the seeded game of `nodes` nodes
/// under settle `version`, 40 picks long.
pub fn seeded_hashes(nodes: usize, version: u8) -> Vec<u64> {
    let mut state = board(nodes, version);
    let mut hashes = Vec::new();
    for pick in seeded_picks(nodes as u64, 40) {
        if state.face.is_game_over() {
            break;
        }
        if let Some(mov) = random_move(&state, pick) {
            if let Outcome::Applied { .. } = protocol::apply(&mut state, mov) {
                hashes.push(hash(&state));
            }
        }
    }
    hashes
}

#[cfg(test)]
fn fold_points<'a>(points: impl Iterator<Item = &'a Point>) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    for p in points {
        h = (h ^ p.x.to_bits()).wrapping_mul(0x100000001b3);
        h = (h ^ p.y.to_bits()).wrapping_mul(0x100000001b3);
    }
    h
}

#[cfg(test)]
fn fold_cubics(cubics: &[curve::Cubic]) -> u64 {
    let points: Vec<Point> = cubics
        .iter()
        .flat_map(|c| [c.p0, c.p1, c.p2, c.p3])
        .collect();
    fold_points(points.iter())
}

/// Hashes along the pipeline of applied move `index` of the seeded game
/// of `nodes` nodes under `version`: the stroke as generated, the fit as
/// kurbo returns it, the cubics and node after grid rounding, the board
/// before the settle and after each of its rounds, and the board after
/// it, so one run on another platform shows the first stage that differs.
#[cfg(test)]
pub fn stages(nodes: usize, index: usize, version: u8) -> Vec<(String, u64)> {
    let mut state = board(nodes, version);
    let mut applied = 0;
    for pick in seeded_picks(nodes as u64, 40) {
        if state.face.is_game_over() {
            break;
        }
        let Some(wire) = random_move(&state, pick) else {
            continue;
        };
        if applied < index {
            if let Outcome::Applied { .. } = protocol::apply(&mut state, wire) {
                applied += 1;
            }
            continue;
        }
        let mut out = vec![
            ("board before".to_string(), hash(&state)),
            ("stroke".to_string(), fold_points(wire.polyline.iter())),
        ];
        let fit = curve::fit(&wire.polyline, curve::tolerance(state.board_size));
        out.push(("fit".to_string(), fold_cubics(&fit)));
        let Ok(entered) = protocol::enter(wire, &state) else {
            out.push(("enter".to_string(), 0));
            return out;
        };
        out.push(("rounded cubics".to_string(), fold_cubics(&entered.segments)));
        out.push((
            "rounded node".to_string(),
            fold_points(std::iter::once(&entered.new_node_pos)),
        ));
        if state.apply_move(entered).is_err() {
            out.push(("apply".to_string(), 0));
            return out;
        }
        out.push(("applied".to_string(), hash(&state)));
        for (round, h) in settle::settle_traced(&mut state).into_iter().enumerate() {
            out.push((format!("round {round}"), h));
        }
        out.push(("settled".to_string(), hash(&state)));
        return out;
    }
    Vec::new()
}

/// `stages` as one line of text.
#[cfg(test)]
pub fn stage_report(nodes: usize, index: usize, version: u8) -> String {
    stages(nodes, index, version)
        .iter()
        .map(|(name, h)| format!("{name} {h:#x}"))
        .collect::<Vec<_>>()
        .join(", ")
}
