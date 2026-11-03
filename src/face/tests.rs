use super::*;
use proptest::prelude::*;

fn rotation_next(m: &FaceModel, h: HalfEdgeId) -> HalfEdgeId {
    let twin = h ^ 1;
    let rot = m.rotation(m.origin(twin));
    let at = rot.iter().position(|&s| s == twin).unwrap();
    rot[(at + rot.len() - 1) % rot.len()]
}

/// Orbit id of every half-edge under the rotation-derived next map.
fn orbits(m: &FaceModel) -> Vec<usize> {
    let total = m.half_edges.len();
    let mut orbit = vec![usize::MAX; total];
    let mut count = 0;
    for start in 0..total {
        if orbit[start] != usize::MAX {
            continue;
        }
        let mut h = start;
        for _ in 0..=total {
            assert_eq!(orbit[h], usize::MAX, "half-edge {h} on two orbits");
            orbit[h] = count;
            h = rotation_next(m, h);
            if h == start {
                break;
            }
        }
        assert_eq!(h, start, "orbit from {start} does not close");
        count += 1;
    }
    orbit
}

fn check(m: &FaceModel) {
    let v = m.nodes.len();
    let e = m.half_edges.len() / 2;
    let f = m.faces.len();

    let mut parent: Vec<usize> = (0..v).collect();
    fn find(p: &mut [usize], x: usize) -> usize {
        if p[x] != x {
            p[x] = find(p, p[x]);
        }
        p[x]
    }
    for h in (0..m.half_edges.len()).step_by(2) {
        let (x, y) = (find(&mut parent, m.origin(h)), find(&mut parent, m.head(h)));
        parent[x] = y;
    }
    let c = (0..v).filter(|&x| find(&mut parent, x) == x).count();
    assert_eq!(v + f, e + 1 + c, "Euler: V={v} E={e} F={f} C={c}");

    for (id, node) in m.nodes.iter().enumerate() {
        let ends = m.half_edges.iter().filter(|h| h.origin == id).count();
        assert_eq!(ends, node.degree as usize, "node {id}");
        assert!(node.degree <= 3, "node {id}");
        for &h in node.rotation() {
            assert_eq!(m.origin(h), id);
        }
    }

    let orbit = orbits(m);
    let orbit_count = orbit.iter().max().map_or(0, |o| o + 1);
    let mut orbit_face = vec![usize::MAX; orbit_count];
    for (h, &o) in orbit.iter().enumerate() {
        if orbit_face[o] == usize::MAX {
            orbit_face[o] = m.face_of(h);
        }
        assert_eq!(m.face_of(h), orbit_face[o], "orbit of {h} spans two faces");
    }

    let mut listed = vec![false; orbit_count];
    let mut placed = vec![false; v];
    for (fid, face) in m.faces.iter().enumerate() {
        for comp in &face.components {
            match *comp {
                Component::Node(n) => {
                    assert_eq!(m.degree(n), 0);
                    assert_eq!(m.nodes[n].home, fid);
                    assert!(!placed[n], "node {n} listed twice");
                    placed[n] = true;
                }
                Component::Cycle(start) => {
                    let o = orbit[start];
                    assert!(!listed[o], "orbit of {start} listed twice");
                    listed[o] = true;
                    assert_eq!(orbit_face[o], fid, "orbit of {start}");
                }
            }
        }
    }
    assert!(listed.iter().all(|&l| l), "orbit on no face");
    for (n, &p) in placed.iter().enumerate() {
        assert_eq!(p, m.degree(n) == 0, "node {n}");
    }
}

/// The pieces of the move's face other than the ones holding its corners,
/// each as a half-edge or isolated node and the lowest node on it.
fn other_pieces(m: &FaceModel, mv: &Move) -> Vec<(Component, NodeId)> {
    let orbit = orbits(m);
    let face = m.corner_face(mv.a, mv.at_a).unwrap();
    let holds_corner = |comp: &Component| match *comp {
        Component::Node(v) => (mv.at_a.is_none() && v == mv.a) || (mv.at_b.is_none() && v == mv.b),
        Component::Cycle(start) => [mv.at_a, mv.at_b]
            .iter()
            .flatten()
            .any(|&h| orbit[h] == orbit[start]),
    };
    m.components(face)
        .iter()
        .filter(|comp| !holds_corner(comp))
        .map(|comp| match *comp {
            Component::Node(v) => (*comp, v),
            Component::Cycle(start) => {
                let lowest = (0..m.half_edges.len())
                    .filter(|&h| orbit[h] == orbit[start])
                    .map(|h| m.origin(h))
                    .min()
                    .unwrap();
                (*comp, lowest)
            }
        })
        .collect()
}

/// Each other piece of the split face sits on the side the move named.
fn check_sides(before: &FaceModel, mv: &Move, after: &FaceModel) {
    let h_am = after.half_edges.len() - 4;
    let left = after.face_of(h_am);
    let right = after.face_of(h_am + 3);
    let split = left != right;
    assert_eq!(after.face_count(), before.face_count() + usize::from(split));
    let pieces = other_pieces(before, mv);
    for (comp, rep) in pieces {
        let want = if mv.enclosed.contains(&rep) {
            left
        } else {
            right
        };
        let got = match comp {
            Component::Node(v) => after.corner_face(v, None).unwrap(),
            Component::Cycle(start) => after.face_of(start),
        };
        assert_eq!(got, want, "piece {rep} of {mv:?}");
    }
}

fn apply_checked(m: &mut FaceModel, mv: &Move) {
    let before = m.clone();
    m.apply(mv).unwrap();
    check(m);
    check_sides(&before, mv, m);
}

/// Plays `picks` out from `n` nodes. With `exhaustive`, every listed move at
/// every position is applied and undone first.
fn play(n: usize, picks: &[u32], exhaustive: bool) -> (FaceModel, Vec<FaceModel>, Vec<Move>) {
    let mut m = FaceModel::new(n);
    let mut snapshots = Vec::new();
    let mut played = Vec::new();
    for &pick in picks {
        let moves = m.moves();
        assert_eq!(moves, m.clone().moves());
        if moves.is_empty() {
            break;
        }
        if exhaustive {
            let before = m.clone();
            for mv in &moves {
                apply_checked(&mut m, mv);
                assert_eq!(m.undo().as_ref(), Some(mv));
                assert_eq!(m, before);
            }
        }
        let mv = moves[pick as usize % moves.len()].clone();
        snapshots.push(m.clone());
        apply_checked(&mut m, &mv);
        played.push(mv);
    }
    let mut replay = FaceModel::new(n);
    for mv in &played {
        replay.apply(mv).unwrap();
    }
    assert_eq!(replay, m);
    assert_eq!(replay.moves(), m.moves());
    (m, snapshots, played)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn random_games_keep_invariants(n in 1usize..=8, picks in prop::collection::vec(any::<u32>(), 24)) {
        let (m, _, played) = play(n, &picks, false);
        prop_assert!(m.is_game_over());
        prop_assert!(played.len() >= 2 * n, "{} moves from {n}", played.len());
        prop_assert!(played.len() < 3 * n, "{} moves from {n}", played.len());
    }

    #[test]
    fn undo_restores_equal_state(n in 1usize..=8, picks in prop::collection::vec(any::<u32>(), 24)) {
        let (mut m, snapshots, played) = play(n, &picks, false);
        prop_assert!(m.is_game_over());
        for (before, mv) in snapshots.iter().zip(&played).rev() {
            prop_assert_eq!(m.undo(), Some(mv.clone()));
            prop_assert_eq!(&m, before);
        }
        prop_assert_eq!(m.undo(), None);
    }

    #[test]
    fn every_listed_move_applies(n in 1usize..=6, picks in prop::collection::vec(any::<u32>(), 18)) {
        play(n, &picks, true);
    }

    #[test]
    fn check_agrees_with_apply(
        n in 1usize..=5,
        picks in prop::collection::vec(any::<u32>(), 8),
        guesses in prop::collection::vec(
            (0usize..8, prop::option::of(0usize..40), 0usize..8, prop::option::of(0usize..40), prop::collection::vec(0usize..8, 0..3)),
            24,
        ),
    ) {
        let (m, _, _) = play(n, &picks, false);
        for mv in m.moves() {
            prop_assert_eq!(m.check(&mv), Ok(()));
            prop_assert_eq!(m.clone().apply(&mv), Ok(()));
        }
        for (a, at_a, b, at_b, enclosed) in guesses {
            let mv = Move { a, at_a, b, at_b, enclosed };
            let checked = m.check(&mv);
            let mut applied = m.clone();
            prop_assert_eq!(applied.apply(&mv), checked);
            if checked.is_err() {
                prop_assert_eq!(&applied, &m);
            }
        }
    }
}

fn first_player_wins(m: &mut FaceModel) -> bool {
    for mv in m.moves() {
        m.apply(&mv).unwrap();
        let opponent_wins = first_player_wins(m);
        m.undo();
        if !opponent_wins {
            return true;
        }
    }
    false
}

#[test]
fn one_node_lasts_two_moves() {
    let mut m = FaceModel::new(1);
    let moves = m.moves();
    assert_eq!(moves.len(), 1);
    assert_eq!(moves[0].a, moves[0].b);
    m.apply(&moves[0]).unwrap();
    check(&m);
    assert_eq!(m.face_count(), 2);
    assert_eq!(m.moves().len(), 2);
    m.apply(&m.moves()[0]).unwrap();
    check(&m);
    assert!(m.is_game_over());
    assert_eq!(m.degree(0), 3);
    assert_eq!(m.degree(1), 3);
    assert_eq!(m.degree(2), 2);
}

#[test]
fn two_nodes_open_with_three_moves() {
    let m = FaceModel::new(2);
    let moves = m.moves();
    let loops = moves.iter().filter(|mv| mv.a == mv.b).count();
    assert_eq!(loops, 2);
    assert_eq!(moves.len(), 3);
    assert!(moves.iter().all(|mv| mv.enclosed.is_empty()));
}

#[test]
fn known_outcomes_for_small_games() {
    assert!(!first_player_wins(&mut FaceModel::new(1)));
    assert!(!first_player_wins(&mut FaceModel::new(2)));
    assert!(first_player_wins(&mut FaceModel::new(3)));
}

#[test]
#[ignore = "about 28 s in debug"]
fn four_nodes_are_a_first_player_win() {
    assert!(first_player_wins(&mut FaceModel::new(4)));
}

#[test]
fn loop_on_isolated_node_lists_one_side_only() {
    let m = FaceModel::new(3);
    let loops: Vec<_> = m.moves().into_iter().filter(|mv| mv.a == mv.b).collect();
    assert_eq!(loops.len(), 6);
    let on_zero: Vec<_> = loops.iter().filter(|mv| mv.a == 0).collect();
    assert_eq!(on_zero.len(), 2);
    assert_eq!(on_zero[0].enclosed, Vec::<NodeId>::new());
    assert_eq!(on_zero[1].enclosed, vec![2]);
}

#[test]
fn loop_encloses_the_chosen_component() {
    let mut m = FaceModel::new(3);
    let mv = Move {
        a: 0,
        at_a: None,
        b: 0,
        at_b: None,
        enclosed: vec![2],
    };
    apply_checked(&mut m, &mv);
    assert_eq!(m.face_count(), 2);
    let inside = m.face_of(0);
    assert_eq!(m.corner_face(2, None), Ok(inside));
    assert_ne!(m.corner_face(1, None), Ok(inside));
    assert!(m.components(inside).contains(&Component::Node(2)));
    assert_eq!(m.moves().iter().filter(|x| x.a == 1 && x.b == 2).count(), 0);
    assert_eq!(m.moves().iter().filter(|x| x.a == 0 && x.b == 2).count(), 1);
}

#[test]
fn loop_on_degree_one_node_offers_both_sides() {
    let mut m = FaceModel::new(3);
    m.apply(&Move {
        a: 0,
        at_a: None,
        b: 1,
        at_b: None,
        enclosed: vec![],
    })
    .unwrap();
    let loops: Vec<_> = m
        .moves()
        .into_iter()
        .filter(|mv| mv.a == 0 && mv.b == 0)
        .collect();
    assert_eq!(loops.len(), 2);
    assert_eq!(loops[1].enclosed, vec![2]);
    for mv in loops {
        let enclosed = !mv.enclosed.is_empty();
        apply_checked(&mut m, &mv);
        assert_eq!(m.face_count(), 2);
        let left = m.face_of(mv.at_a.unwrap());
        let inside = m.face_of(m.rotation(0)[1]);
        assert_ne!(left, inside);
        assert_eq!(m.corner_face(2, None) == Ok(inside), enclosed);
        m.undo();
    }
}

#[test]
fn four_cycle_offers_each_pair_in_both_faces() {
    let mut m = FaceModel::new(2);
    m.apply(&Move {
        a: 0,
        at_a: None,
        b: 1,
        at_b: None,
        enclosed: vec![],
    })
    .unwrap();
    let second: Vec<_> = m
        .moves()
        .into_iter()
        .filter(|mv| mv.a == 0 && mv.b == 1)
        .collect();
    assert_eq!(second.len(), 1);
    m.apply(&second[0]).unwrap();
    check(&m);
    assert_eq!(m.face_count(), 2);
    let third = m.moves();
    assert_eq!(third.len(), 12);
    let last: Vec<_> = third.iter().filter(|mv| mv.a == 2 && mv.b == 3).collect();
    assert_eq!(last.len(), 2);
    assert_ne!(last[0].at_a, last[1].at_a);
    assert_ne!(
        m.face_of(last[0].at_a.unwrap()),
        m.face_of(last[1].at_a.unwrap())
    );
    assert!(third.iter().all(|mv| mv.enclosed.is_empty()));
}

#[test]
fn apply_rejects_bad_moves() {
    let mut m = FaceModel::new(2);
    let bad = |a, at_a, b, at_b, enclosed: Vec<NodeId>| Move {
        a,
        at_a,
        b,
        at_b,
        enclosed,
    };
    assert_eq!(
        m.apply(&bad(0, None, 5, None, vec![])),
        Err(IllegalMove::Node)
    );
    assert_eq!(
        m.apply(&bad(0, Some(0), 1, None, vec![])),
        Err(IllegalMove::Corner)
    );
    assert_eq!(
        m.apply(&bad(0, None, 1, None, vec![1])),
        Err(IllegalMove::Enclosed)
    );
    assert_eq!(
        m.apply(&bad(0, None, 0, None, vec![0])),
        Err(IllegalMove::Enclosed)
    );
    m.apply(&bad(0, None, 0, None, vec![1])).unwrap();
    check(&m);
    assert_eq!(
        m.apply(&bad(0, Some(3), 1, None, vec![])),
        Err(IllegalMove::Face)
    );
    assert_eq!(
        m.apply(&bad(0, Some(0), 0, Some(0), vec![])),
        Err(IllegalMove::Degree)
    );
    assert_eq!(
        m.apply(&bad(0, Some(2), 2, Some(1), vec![])),
        Err(IllegalMove::Corner)
    );
    assert_eq!(m.edge_count(), 2);
}
