//! A seeded random game at the current settle version, hashed after every
//! move, so one number says whether a build settles as macOS did when the
//! values below were recorded. Only sums, products, divisions and square
//! roots go into the strokes, which every platform rounds the same.

#[cfg(test)]
use crate::curve;
use crate::protocol::{self, Outcome};
#[cfg(test)]
use crate::settle;
use crate::types::{GameState, Player, Point};

/// The settled board of the seeded 7-node game. A change means version 1
/// settles differently: bump SETTLE_VERSION in src/protocol.rs and record
/// the new value.
#[cfg(test)]
pub const VERSION_1_GOLDEN: u64 = 0xa032cb53618d8318;

/// The settled board after each move of the seeded 9-node game, recorded
/// on macOS. A mismatch names the first move another platform settles
/// differently; a mismatch after a change to the settle means bumping
/// SETTLE_VERSION and recording the new values.
#[cfg(test)]
pub const VERSION_1_SEEDED_GAME: &[u64] = &[
    0x7bb01be736536569,
    0x7a06061ba002b013,
    0xa9fe3221b9b96035,
    0x475be4ad2b1b4a3e,
    0xa19c5ada51285ec,
    0x98127e76221c6345,
    0x3d62a2f5e2c6e0bc,
    0x56516138c9de2925,
    0x759c44b36b8bcc5a,
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

/// A random game of `nodes` nodes played through the protocol, settling
/// after each move, with the moves as the engine applied them.
#[cfg(test)]
pub fn play_random(
    nodes: usize,
    picks: &[(usize, usize, f64, f64)],
) -> (GameState, Vec<protocol::Move>) {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
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

/// A 64-bit fold of every coordinate on the board.
pub fn hash(state: &GameState) -> u64 {
    bits(state).iter().fold(0xcbf29ce484222325u64, |h, b| {
        (h ^ b).wrapping_mul(0x100000001b3)
    })
}

/// The settled board after each move of the seeded game of `nodes` nodes,
/// 40 picks long.
pub fn seeded_hashes(nodes: usize) -> Vec<u64> {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
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
/// of `nodes` nodes: the stroke as generated, the fit as kurbo returns
/// it, the cubics and node after grid rounding, the board before the
/// settle and after each of its rounds, and the board after it, so one
/// run on another platform shows the first stage that differs.
#[cfg(test)]
pub fn stages(nodes: usize, index: usize) -> Vec<(String, u64)> {
    let mut state = GameState::with_board_size(nodes, 1000);
    state.current_player = Player::Human;
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
pub fn stage_report(nodes: usize, index: usize) -> String {
    stages(nodes, index)
        .iter()
        .map(|(name, h)| format!("{name} {h:#x}"))
        .collect::<Vec<_>>()
        .join(", ")
}
