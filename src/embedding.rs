//! Reads a drawn line into the face model: the corner it leaves at each
//! end, and for a split, which other boundary pieces end up on its left.
//!
//! Angles are counterclockwise on screen, where y grows downward, so the
//! face model's rotations match what the player sees. The model puts a
//! self-loop's departing half-edge right before its returning one in the
//! rotation, so a loop is read in whichever direction leaves the node's
//! other spoke outside the turn from departure to return, and
//! counterclockwise when the loop is alone at its node. Geometry is the
//! flattening of each line's spline, the same polyline validation tests
//! against.

use std::cmp::Ordering;
use std::f64::consts::TAU;

use crate::curve;
use crate::face::{Component, HalfEdgeId, IllegalMove, Move as FaceMove, NodeId};
use crate::geometry::{closest_point_on_segment, point_in_polygon, signed_area};
use crate::types::{scale, GameState, Move, Point, MIN_PATH_CLEARANCE};
use crate::validation::Rejection;

/// The face-model move a drawn line makes, or why it cannot be placed.
pub fn embed(state: &GameState, mov: &Move) -> Result<FaceMove, Rejection> {
    let face = &state.face;
    let (a, b) = (mov.from_node, mov.to_node);
    if a >= face.node_count() {
        return Err(Rejection::missing_node(a, "Source"));
    }
    if b >= face.node_count() {
        return Err(Rejection::missing_node(b, "Target"));
    }
    let flat = mov.flatten(curve::tolerance(state.board_size));
    let forward = a != b || new_loop_runs_forward(state, a, &flat);
    let curve = run(&flat, forward);
    let back = run(&flat, !forward);
    let at_a = corner(state, a, &curve);
    let at_b = corner(state, b, &back);
    let own = face
        .component_of(a, at_a)
        .map_err(|e| rejection(state, e, mov))?;
    let other = face
        .component_of(b, at_b)
        .map_err(|e| rejection(state, e, mov))?;
    let enclosed = if own == other {
        enclosed_by(state, a, at_a, b, at_b, &curve, own)
    } else {
        Vec::new()
    };
    let topology = FaceMove {
        a,
        at_a,
        b,
        at_b,
        enclosed,
    };
    face.check(&topology)
        .map_err(|e| rejection(state, e, mov))?;
    Ok(topology)
}

fn rejection(state: &GameState, why: IllegalMove, mov: &Move) -> Rejection {
    let (a, b) = (mov.from_node, mov.to_node);
    match why {
        IllegalMove::Degree if a == b => Rejection::self_loop_full(a),
        IllegalMove::Degree if state.face.degree(a) == 3 => Rejection::node_full(a, "Source"),
        IllegalMove::Degree => Rejection::node_full(b, "Target"),
        IllegalMove::Node if a >= state.face.node_count() => Rejection::missing_node(a, "Source"),
        IllegalMove::Node => Rejection::missing_node(b, "Target"),
        IllegalMove::Corner | IllegalMove::Face | IllegalMove::Enclosed => {
            Rejection::crossing_somewhere(&mov.polyline)
        }
    }
}

/// Pieces of the split face on the left of the new line: those inside the
/// closed walk it completes when that walk runs counterclockwise, those
/// outside it when it runs clockwise.
fn enclosed_by(
    state: &GameState,
    a: NodeId,
    at_a: Option<HalfEdgeId>,
    b: NodeId,
    at_b: Option<HalfEdgeId>,
    curve: &[Point],
    own: Component,
) -> Vec<NodeId> {
    let face = &state.face;
    let mut polygon = curve.to_vec();
    if let (Some(at_a), Some(at_b)) = (at_a, at_b) {
        if a != b {
            let rotation = face.rotation(a);
            let at = rotation.iter().position(|&h| h == at_a).expect("own spoke");
            let arriving = rotation[(at + 1) % rotation.len()] ^ 1;
            for h in face.cycle(at_b) {
                polygon.extend(half_edge_path(state, h));
                if h == arriving {
                    break;
                }
            }
        }
    }
    let counterclockwise = signed_area(&polygon) >= 0.0;
    let f = face.corner_face(a, at_a).expect("own corner");
    face.components(f)
        .iter()
        .copied()
        .filter(|&comp| comp != own)
        .filter(|&comp| point_in_polygon(&polygon, &probe(state, comp)) == counterclockwise)
        .map(|comp| face.lowest_node(comp))
        .collect()
}

/// A point on the piece, off every other piece of its face.
fn probe(state: &GameState, comp: Component) -> Point {
    let v = match comp {
        Component::Node(v) => v,
        Component::Cycle(start) => state.face.origin(start),
    };
    state.nodes[v].position
}

/// The spoke of `v` the new line follows in the rotation, read from the
/// direction `outward` leaves `v` in.
fn corner(state: &GameState, v: NodeId, outward: &[Point]) -> Option<HalfEdgeId> {
    let rotation = state.face.rotation(v);
    let center = state.nodes[v].position;
    let radius = MIN_PATH_CLEARANCE * scale(state.board_size);
    let heading = leaving_angle(center, outward, radius);
    rotation.iter().copied().min_by(|&x, &y| {
        let turn_to = |h| {
            turn(
                leaving_angle(center, &half_edge_path(state, h), radius),
                heading,
            )
        };
        turn_to(x)
            .partial_cmp(&turn_to(y))
            .unwrap_or(Ordering::Equal)
    })
}

/// Counterclockwise angle from `from` to `to`, in [0, 2π).
fn turn(from: f64, to: f64) -> f64 {
    (to - from).rem_euclid(TAU)
}

/// Direction `path` leaves `center` in, read where it first clears `radius`.
fn leaving_angle(center: Point, path: &[Point], radius: f64) -> f64 {
    let farther = |p: &&Point, q: &&Point| {
        p.distance_to(&center)
            .partial_cmp(&q.distance_to(&center))
            .unwrap_or(Ordering::Equal)
    };
    let far = path
        .iter()
        .find(|p| p.distance_to(&center) > radius)
        .or_else(|| path.iter().max_by(farther));
    far.map_or(0.0, |p| (center.y - p.y).atan2(p.x - center.x))
}

/// Geometry of half-edge `h`, from its origin to its head.
fn half_edge_path(state: &GameState, h: HalfEdgeId) -> Vec<Point> {
    let line = &state.lines[h / 4];
    let curve = line_path(state, h / 4);
    let (to_mid, from_mid) = split_at(&curve, &line.new_node_pos);
    match h % 4 {
        0 => to_mid,
        1 => to_mid.into_iter().rev().collect(),
        2 => from_mid,
        _ => from_mid.into_iter().rev().collect(),
    }
}

/// Geometry of line `k` from the model's `a` end to its `b` end. A chord
/// runs as drawn. A self-loop runs the way that puts its two ends in the
/// model's order around the node's other spoke, if there is one.
fn line_path(state: &GameState, k: usize) -> Vec<Point> {
    let line = &state.lines[k];
    if line.from_node != line.to_node {
        return line.flat.clone();
    }
    let a = line.from_node;
    let rotation = state.face.rotation(a);
    let forward = match rotation.iter().copied().find(|&h| h / 4 != k) {
        None => signed_area(&line.flat) >= 0.0,
        Some(spoke) => {
            let turns_from_start = |h: HalfEdgeId| {
                let at = |x| rotation.iter().position(|&y| y == x).expect("own spoke");
                (at(h) + rotation.len() - at(4 * k)) % rotation.len()
            };
            let model_ends_before_spoke = turns_from_start(4 * k + 3) < turns_from_start(spoke);
            model_ends_before_spoke == ends_before_spoke(state, a, &line.flat, spoke)
        }
    };
    run(&line.flat, forward)
}

/// Whether a self-loop being drawn at `a` runs as the model will hold it.
fn new_loop_runs_forward(state: &GameState, a: NodeId, flat: &[Point]) -> bool {
    match state.face.rotation(a).first() {
        None => signed_area(flat) >= 0.0,
        Some(&spoke) => ends_before_spoke(state, a, flat, spoke),
    }
}

/// Turning counterclockwise from the direction `flat` leaves `a` in,
/// whether its returning end comes up before `spoke` does.
fn ends_before_spoke(state: &GameState, a: NodeId, flat: &[Point], spoke: HalfEdgeId) -> bool {
    let center = state.nodes[a].position;
    let radius = MIN_PATH_CLEARANCE * scale(state.board_size);
    let start = leaving_angle(center, flat, radius);
    let end = leaving_angle(center, &run(flat, false), radius);
    let other = leaving_angle(center, &half_edge_path(state, spoke), radius);
    turn(start, end) < turn(start, other)
}

fn run(path: &[Point], forward: bool) -> Vec<Point> {
    if forward {
        path.to_vec()
    } else {
        path.iter().rev().copied().collect()
    }
}

/// The polyline up to the new node and the polyline on from it.
fn split_at(curve: &[Point], mid: &Point) -> (Vec<Point>, Vec<Point>) {
    let nearest = (1..curve.len()).min_by(|&i, &j| {
        let d = |k: usize| closest_point_on_segment(&curve[k - 1], &curve[k], mid).1;
        d(i).partial_cmp(&d(j)).unwrap_or(Ordering::Equal)
    });
    let Some(i) = nearest else {
        return (curve.to_vec(), curve.to_vec());
    };
    let mut head = curve[..i].to_vec();
    head.push(*mid);
    let mut tail = vec![*mid];
    tail.extend_from_slice(&curve[i..]);
    (head, tail)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Player;
    use crate::validation::{BoardLocation, RejectionKind};

    fn p(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    /// Three nodes in a row at y = 500: 0 at x = 200, 1 at 500, 2 at 800.
    fn row() -> GameState {
        let mut state = GameState::new(3);
        for (node, x) in state.nodes.iter_mut().zip([200.0, 500.0, 800.0]) {
            node.position = p(x, 500.0);
        }
        state
    }

    fn mov(from: usize, to: usize, polyline: Vec<Point>, new_node: Point) -> Move {
        Move::new(from, to, polyline, new_node, Player::Human, 1000)
    }

    /// A circle about `center` swept all the way round from `start`.
    fn circle(start: Point, center: Point, clockwise: bool) -> Vec<Point> {
        let radius = start.distance_to(&center);
        let from = (start.y - center.y).atan2(start.x - center.x);
        let steps = 48;
        (0..=steps)
            .map(|i| {
                let t = i as f64 / steps as f64 * TAU;
                let t = if clockwise { -t } else { t };
                p(
                    center.x + radius * (from + t).cos(),
                    center.y + radius * (from + t).sin(),
                )
            })
            .collect()
    }

    fn straight(a: Point, b: Point) -> Vec<Point> {
        (0..=10)
            .map(|i| {
                let t = i as f64 / 10.0;
                p(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
            })
            .collect()
    }

    #[test]
    fn loop_around_one_node_encloses_it_whichever_way_it_is_drawn() {
        for clockwise in [false, true] {
            let mut state = row();
            let loop_line = mov(
                0,
                0,
                circle(p(200.0, 500.0), p(400.0, 500.0), clockwise),
                p(600.0, 500.0),
            );
            let topology = embed(&state, &loop_line).unwrap();
            assert_eq!(
                topology,
                FaceMove {
                    a: 0,
                    at_a: None,
                    b: 0,
                    at_b: None,
                    enclosed: vec![1],
                }
            );
            state.apply_move(loop_line).unwrap();
            let face = &state.face;
            let inside = face.corner_face(1, None).unwrap();
            assert_ne!(inside, face.corner_face(2, None).unwrap());
            assert!(face.components(inside).contains(&Component::Node(1)));
            assert!(face.moves().iter().all(|m| !(m.a == 1 && m.b == 2)));
        }
    }

    #[test]
    fn loop_around_nothing_encloses_nothing_whichever_way_it_is_drawn() {
        for clockwise in [false, true] {
            let mut state = row();
            let loop_line = mov(
                0,
                0,
                circle(p(200.0, 500.0), p(260.0, 500.0), clockwise),
                p(320.0, 500.0),
            );
            let topology = embed(&state, &loop_line).unwrap();
            assert_eq!(topology.a, 0);
            assert_eq!(topology.at_a, None);
            assert_eq!(topology.enclosed, Vec::<NodeId>::new());
            state.apply_move(loop_line).unwrap();
            let face = &state.face;
            let outside = face.corner_face(1, None).unwrap();
            assert_eq!(outside, face.corner_face(2, None).unwrap());
            assert_ne!(outside, face.face_of(0));
            assert!(face.components(face.face_of(0)).len() == 1);
            assert!(face.moves().iter().any(|m| m.a == 1 && m.b == 2));
        }
    }

    #[test]
    fn chord_between_two_pieces_of_one_face_merges_them() {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                1,
                straight(p(200.0, 500.0), p(500.0, 500.0)),
                p(350.0, 500.0),
            ))
            .unwrap();
        assert_eq!(state.face.components(0).len(), 2);
        let chord = mov(
            2,
            1,
            straight(p(800.0, 500.0), p(500.0, 500.0)),
            p(650.0, 500.0),
        );
        let topology = embed(&state, &chord).unwrap();
        assert_eq!(topology.at_a, None);
        assert_eq!(topology.at_b, Some(3));
        assert!(topology.enclosed.is_empty());
        state.apply_move(chord).unwrap();
        assert_eq!(state.face.face_count(), 1);
        assert_eq!(state.face.components(0).len(), 1);
    }

    #[test]
    fn chord_within_one_piece_splits_the_face() {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                1,
                straight(p(200.0, 500.0), p(500.0, 500.0)),
                p(350.0, 500.0),
            ))
            .unwrap();
        let bow = mov(
            0,
            1,
            vec![
                p(200.0, 500.0),
                p(250.0, 300.0),
                p(450.0, 300.0),
                p(500.0, 500.0),
            ],
            p(250.0, 300.0),
        );
        state.apply_move(bow).unwrap();
        assert_eq!(state.face.face_count(), 2);
        let between = |x: usize, y: usize| {
            state
                .face
                .moves()
                .iter()
                .filter(|m| m.a == x.min(y) && m.b == x.max(y))
                .count()
        };
        assert_eq!(between(2, 3), 1);
        assert_eq!(between(2, 4), 1);
        assert_eq!(between(3, 4), 3);
    }

    /// Node 1 at the centre with lines in from node 0 on its left and node
    /// 2 on its right, so spoke 3 points left and spoke 7 points right.
    /// Node 3 sits above, untouched.
    fn crossroads() -> GameState {
        let mut state = GameState::new(4);
        for (node, at) in state.nodes.iter_mut().zip([
            p(200.0, 500.0),
            p(500.0, 500.0),
            p(800.0, 500.0),
            p(500.0, 200.0),
        ]) {
            node.position = at;
        }
        state
            .apply_move(mov(
                0,
                1,
                straight(p(200.0, 500.0), p(500.0, 500.0)),
                p(350.0, 500.0),
            ))
            .unwrap();
        state
            .apply_move(mov(
                2,
                1,
                straight(p(800.0, 500.0), p(500.0, 500.0)),
                p(650.0, 500.0),
            ))
            .unwrap();
        state
    }

    #[test]
    fn third_line_at_a_node_takes_the_gap_it_leaves_through() {
        let state = crossroads();
        let up = mov(
            1,
            3,
            straight(p(500.0, 500.0), p(500.0, 200.0)),
            p(500.0, 350.0),
        );
        assert_eq!(embed(&state, &up).unwrap().at_a, Some(7));
        let down = mov(
            1,
            3,
            vec![
                p(500.0, 500.0),
                p(500.0, 700.0),
                p(900.0, 700.0),
                p(900.0, 100.0),
                p(500.0, 200.0),
            ],
            p(900.0, 700.0),
        );
        assert_eq!(embed(&state, &down).unwrap().at_a, Some(3));
    }

    #[test]
    fn a_line_between_faces_is_a_crossing_over_the_whole_path() {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                0,
                circle(p(200.0, 500.0), p(400.0, 500.0), false),
                p(600.0, 500.0),
            ))
            .unwrap();
        let escape = mov(
            1,
            2,
            straight(p(500.0, 500.0), p(800.0, 500.0)),
            p(650.0, 500.0),
        );
        let rejection = embed(&state, &escape).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::PathCrossesLine);
        assert_eq!(rejection.at, BoardLocation::Segments { first: 0, last: 9 });
    }

    #[test]
    fn a_loop_on_a_node_with_two_lines_is_refused_as_full() {
        let state = crossroads();
        let lp = mov(
            1,
            1,
            vec![
                p(500.0, 500.0),
                p(600.0, 400.0),
                p(500.0, 300.0),
                p(400.0, 400.0),
                p(500.0, 500.0),
            ],
            p(500.0, 300.0),
        );
        let rejection = embed(&state, &lp).unwrap_err();
        assert_eq!(rejection.kind, RejectionKind::NodeFull);
        assert_eq!(rejection.at, BoardLocation::Node { id: 1 });
    }

    /// Node 1 with one line in from node 0 and a loop drawn from node 1
    /// around `center`: around its own line when the centre is left of it,
    /// away from it when the centre is right of it.
    fn loop_at_node_one(center: Point, clockwise: bool) -> GameState {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                1,
                straight(p(200.0, 500.0), p(500.0, 500.0)),
                p(350.0, 500.0),
            ))
            .unwrap();
        let ring = circle(p(500.0, 500.0), center, clockwise);
        let far = ring[24];
        let topology = embed(&state, &mov(1, 1, ring.clone(), far)).unwrap();
        assert_eq!(
            topology,
            FaceMove {
                a: 1,
                at_a: Some(3),
                b: 1,
                at_b: Some(3),
                enclosed: vec![2],
            }
        );
        state.apply_move(mov(1, 1, ring, far)).unwrap();
        state
    }

    #[test]
    fn loop_around_its_own_line_walls_off_the_rest_whichever_way_it_is_drawn() {
        for clockwise in [false, true] {
            let state = loop_at_node_one(p(300.0, 500.0), clockwise);
            let face = &state.face;
            assert_eq!(face.face_count(), 2);
            let outside = face.corner_face(2, None).unwrap();
            assert_eq!(outside, face.face_of(4));
            assert_ne!(outside, face.face_of(face.rotation(0)[0]));
            assert_ne!(outside, face.face_of(face.rotation(3)[0]));
            assert!(face.moves().iter().all(|m| m.b != 2 || m.a == 2));
        }
    }

    #[test]
    fn loop_away_from_its_own_line_rings_only_what_it_encloses() {
        for clockwise in [false, true] {
            let state = loop_at_node_one(p(660.0, 500.0), clockwise);
            let face = &state.face;
            assert_eq!(face.face_count(), 2);
            let inside = face.corner_face(2, None).unwrap();
            assert_eq!(inside, face.face_of(4));
            assert_ne!(inside, face.face_of(face.rotation(0)[0]));
            assert!(face.moves().iter().all(|m| m.b != 2 || m.a == 2));
        }
    }

    #[test]
    fn chord_enclosing_a_node_puts_it_on_one_side_whichever_way_it_runs() {
        let bow = vec![
            p(200.0, 500.0),
            p(200.0, 300.0),
            p(900.0, 300.0),
            p(900.0, 700.0),
            p(500.0, 700.0),
            p(500.0, 500.0),
        ];
        let mut sides = Vec::new();
        for (from, to, path) in [
            (0, 1, bow.clone()),
            (1, 0, bow.iter().rev().copied().collect()),
        ] {
            let mut state = row();
            state
                .apply_move(mov(
                    0,
                    1,
                    straight(p(200.0, 500.0), p(500.0, 500.0)),
                    p(350.0, 500.0),
                ))
                .unwrap();
            let chord = mov(from, to, path, p(900.0, 300.0));
            sides.push(embed(&state, &chord).unwrap().enclosed);
            state.apply_move(chord).unwrap();
            let face = &state.face;
            assert_eq!(face.face_count(), 2);
            let inside = face.corner_face(2, None).unwrap();
            let corners_inside = |v: NodeId| {
                face.rotation(v)
                    .iter()
                    .filter(|&&h| face.face_of(h) == inside)
                    .count()
            };
            assert_eq!(corners_inside(3), 1);
            assert_eq!(corners_inside(4), 1);
            assert_eq!(corners_inside(0), 1);
            assert_eq!(corners_inside(1), 1);
        }
        sides.sort();
        assert_eq!(sides, vec![vec![], vec![2]]);
    }

    #[test]
    fn chord_between_two_cycles_of_one_face_merges_them() {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                0,
                circle(p(200.0, 500.0), p(260.0, 500.0), false),
                p(320.0, 500.0),
            ))
            .unwrap();
        state
            .apply_move(mov(
                2,
                2,
                circle(p(800.0, 500.0), p(740.0, 500.0), false),
                p(680.0, 500.0),
            ))
            .unwrap();
        assert_eq!(state.face.face_count(), 3);
        assert_eq!(state.face.components(0).len(), 3);
        let over = mov(
            0,
            2,
            vec![
                p(200.0, 500.0),
                p(200.0, 300.0),
                p(800.0, 300.0),
                p(800.0, 500.0),
            ],
            p(500.0, 300.0),
        );
        let topology = embed(&state, &over).unwrap();
        assert!(topology.enclosed.is_empty());
        state.apply_move(over).unwrap();
        assert_eq!(state.face.face_count(), 3);
        assert_eq!(state.face.components(0).len(), 2);
    }

    #[test]
    fn a_refused_move_changes_nothing() {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                0,
                circle(p(200.0, 500.0), p(400.0, 500.0), false),
                p(600.0, 500.0),
            ))
            .unwrap();
        let before = state.face.clone();
        let escape = mov(
            1,
            2,
            straight(p(500.0, 500.0), p(800.0, 500.0)),
            p(650.0, 500.0),
        );
        assert!(state.apply_move(escape).is_err());
        assert_eq!(state.face, before);
        assert_eq!(state.lines.len(), 1);
        assert_eq!(state.nodes.len(), 4);
        assert_eq!(state.move_history.len(), 1);
        assert_eq!(state.current_player, Player::AI);
    }

    #[test]
    fn a_chord_reaches_a_looped_node_only_through_the_face_it_came_from() {
        for clockwise in [false, true] {
            let mut around = loop_at_node_one(p(300.0, 500.0), clockwise);
            let under = mov(
                2,
                4,
                vec![
                    p(800.0, 500.0),
                    p(800.0, 800.0),
                    p(100.0, 800.0),
                    p(100.0, 500.0),
                ],
                p(450.0, 800.0),
            );
            assert!(embed(&around, &under).unwrap().enclosed.is_empty());
            around.apply_move(under).unwrap();
            assert_eq!(around.face.face_count(), 2);

            let away = loop_at_node_one(p(660.0, 500.0), clockwise);
            let inside = mov(
                2,
                4,
                vec![p(800.0, 500.0), p(810.0, 450.0), p(820.0, 500.0)],
                p(810.0, 450.0),
            );
            assert!(embed(&away, &inside).is_ok());
            let through = mov(
                0,
                4,
                vec![
                    p(200.0, 500.0),
                    p(200.0, 800.0),
                    p(820.0, 800.0),
                    p(820.0, 500.0),
                ],
                p(500.0, 800.0),
            );
            let topology = embed(&away, &through).unwrap();
            assert_eq!((topology.at_a, topology.at_b), (Some(0), Some(5)));
            let mut away = away;
            away.apply_move(through).unwrap();
            assert_eq!(away.face.face_count(), 3);
        }
    }

    /// A loop alone at node 0, drawn either way; its node 3 sits at the
    /// loop's far right.
    fn looped_node_zero(clockwise: bool) -> GameState {
        let mut state = row();
        state
            .apply_move(mov(
                0,
                0,
                circle(p(200.0, 500.0), p(260.0, 500.0), clockwise),
                p(320.0, 500.0),
            ))
            .unwrap();
        state
    }

    fn over_the_loop() -> Move {
        mov(
            0,
            1,
            vec![
                p(200.0, 500.0),
                p(200.0, 300.0),
                p(500.0, 300.0),
                p(500.0, 500.0),
            ],
            p(350.0, 300.0),
        )
    }

    #[test]
    fn chords_at_a_looped_node_take_the_corner_they_leave_through() {
        for clockwise in [false, true] {
            let state = looped_node_zero(clockwise);
            let inside = mov(
                0,
                3,
                straight(p(200.0, 500.0), p(320.0, 500.0)),
                p(260.0, 500.0),
            );
            let topology = embed(&state, &inside).unwrap();
            assert_eq!((topology.at_a, topology.at_b), (Some(0), Some(2)));
            let topology = embed(&state, &over_the_loop()).unwrap();
            assert_eq!(topology.at_a, Some(3));
        }
    }

    #[test]
    fn a_boundary_walk_through_a_stored_loop_splits_the_right_region() {
        for clockwise in [false, true] {
            let mut state = looped_node_zero(clockwise);
            state.apply_move(over_the_loop()).unwrap();
            let down = mov(
                4,
                3,
                vec![p(350.0, 300.0), p(345.0, 400.0), p(320.0, 500.0)],
                p(345.0, 400.0),
            );
            state.apply_move(down).unwrap();
            let face = &state.face;
            assert_eq!(face.face_count(), 3);
            let outer = face.corner_face(2, None).unwrap();
            let hole = face.face_of(0);
            assert_ne!(hole, outer);
            let lens = (0..3).find(|&f| f != outer && f != hole).unwrap();
            let corners_in = |v: NodeId, f| {
                face.rotation(v)
                    .iter()
                    .filter(|&&h| face.face_of(h) == f)
                    .count()
            };
            assert_eq!(face.components(hole).len(), 1);
            assert_eq!(face.components(lens).len(), 1);
            assert_eq!(corners_in(3, hole), 1);
            assert_eq!(corners_in(3, lens), 1);
            assert_eq!(corners_in(3, outer), 1);
            assert_eq!(corners_in(4, lens), 1);
            assert_eq!(corners_in(4, outer), 2);
        }
    }
}
