//! The live form of a region: its game with every dead node forgotten.
//!
//! A dead node never ends a move again, so the game depends only on the
//! live corners along each boundary in cyclic order, on which corners
//! share a node, and on which boundaries share a face. Boundaries with no
//! live corner go too, since enclosing one or not leaves the same game.
//! The pieces of boundaries joined by shared nodes and the faces form a
//! tree, as on the sphere, so the form is built as in `canonical`: a
//! breadth-first code of each piece from every corner, the faces round
//! it holding the sorted forms of the pieces inside, minimum over the
//! tree's centres and over both orientations, so a mirror image has the
//! same form.

use std::collections::HashMap;

use super::{Component, FaceId, FaceModel};

const NONE: u32 = u32::MAX;
const FACE: u32 = 0;
const PIECE: u32 = 1;

type Corner = usize;
type Piece = usize;

struct Corners {
    next: Vec<Corner>,
    prev: Vec<Corner>,
    partner: Vec<Option<Corner>>,
    lives: Vec<u32>,
    face: Vec<FaceId>,
}

impl Corners {
    /// The live corners of the faces from `frozen` on: one per half-edge
    /// out of a node with lives left, one per isolated node, boundary by
    /// boundary.
    fn of(m: &FaceModel, frozen: usize) -> Self {
        let mut out = Self {
            next: Vec::new(),
            prev: Vec::new(),
            partner: Vec::new(),
            lives: Vec::new(),
            face: Vec::new(),
        };
        let mut of_dart = vec![NONE as usize; m.half_edges.len()];
        for f in frozen..m.faces.len() {
            for comp in &m.faces[f].components {
                let first = out.lives.len();
                match *comp {
                    Component::Node(_) => out.push(3, f),
                    Component::Cycle(start) => {
                        for h in m.cycle(start) {
                            let degree = m.nodes[m.origin(h)].degree;
                            if degree < 3 {
                                of_dart[h] = out.lives.len();
                                out.push(3 - u32::from(degree), f);
                            }
                        }
                    }
                }
                let last = out.lives.len();
                if last > first {
                    out.next[last - 1] = first;
                    out.prev[first] = last - 1;
                }
            }
        }
        for (h, &c) in of_dart.iter().enumerate() {
            if c == NONE as usize {
                continue;
            }
            let node = &m.nodes[m.origin(h)];
            if node.degree == 2 {
                let other = of_dart[node.spokes[usize::from(node.spokes[0] == h)]];
                debug_assert_ne!(other, NONE as usize, "a live node's corners share a region");
                out.partner[c] = Some(other).filter(|&o| o != NONE as usize);
            }
        }
        out
    }

    fn push(&mut self, lives: u32, face: FaceId) {
        let c = self.lives.len();
        self.next.push(c + 1);
        self.prev.push(c.wrapping_sub(1));
        self.partner.push(None);
        self.lives.push(lives);
        self.face.push(face);
    }

    /// Corners joined by `next` and `partner`, numbered from zero.
    fn pieces(&self) -> (Vec<Piece>, usize) {
        let n = self.lives.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(parent: &mut [usize], mut v: usize) -> usize {
            while parent[v] != v {
                parent[v] = parent[parent[v]];
                v = parent[v];
            }
            v
        }
        for c in 0..n {
            for other in [Some(self.next[c]), self.partner[c]].into_iter().flatten() {
                let (a, b) = (find(&mut parent, c), find(&mut parent, other));
                parent[a] = b;
            }
        }
        let mut number = vec![NONE as usize; n];
        let mut count = 0;
        let mut of_corner = vec![0; n];
        for (c, slot) in of_corner.iter_mut().enumerate() {
            let root = find(&mut parent, c);
            if number[root] == NONE as usize {
                number[root] = count;
                count += 1;
            }
            *slot = number[root];
        }
        (of_corner, count)
    }
}

struct Tree<'a> {
    corners: &'a Corners,
    forward: bool,
    /// Corners of each piece, ascending.
    members: Vec<Vec<Corner>>,
    /// Faces round each piece, ascending.
    faces_of_piece: Vec<Vec<FaceId>>,
    /// Pieces touching each face, ascending; indexed by face id.
    pieces_of_face: Vec<Vec<Piece>>,
    face_memo: HashMap<(FaceId, Option<Piece>), Vec<u32>>,
    piece_memo: HashMap<(Piece, Option<FaceId>), Vec<u32>>,
}

impl<'a> Tree<'a> {
    fn new(corners: &'a Corners, forward: bool) -> Self {
        let (piece_of_corner, count) = corners.pieces();
        let faces = corners.face.iter().copied().max().map_or(0, |f| f + 1);
        let mut members = vec![Vec::new(); count];
        let mut faces_of_piece = vec![Vec::new(); count];
        let mut pieces_of_face = vec![Vec::new(); faces];
        for (c, &p) in piece_of_corner.iter().enumerate() {
            members[p].push(c);
            faces_of_piece[p].push(corners.face[c]);
            pieces_of_face[corners.face[c]].push(p);
        }
        for list in faces_of_piece.iter_mut().chain(pieces_of_face.iter_mut()) {
            list.sort_unstable();
            list.dedup();
        }
        Self {
            corners,
            forward,
            members,
            faces_of_piece,
            pieces_of_face,
            face_memo: HashMap::new(),
            piece_memo: HashMap::new(),
        }
    }

    fn step(&self, c: Corner) -> Corner {
        if self.forward {
            self.corners.next[c]
        } else {
            self.corners.prev[c]
        }
    }

    /// Breadth-first code of the piece holding `root`: for each corner
    /// in label order, the labels of the corner after it along its
    /// boundary and of its node's other corner, and its node's lives.
    fn code(&self, root: Corner, labels: &mut [u32], order: &mut Vec<Corner>, code: &mut Vec<u32>) {
        for &c in order.iter() {
            labels[c] = NONE;
        }
        order.clear();
        code.clear();
        order.push(root);
        labels[root] = 0;
        let mut i = 0;
        while i < order.len() {
            let c = order[i];
            for e in [Some(self.step(c)), self.corners.partner[c]] {
                let Some(e) = e else {
                    code.push(NONE);
                    continue;
                };
                if labels[e] == NONE {
                    labels[e] = order.len() as u32;
                    order.push(e);
                }
                code.push(labels[e]);
            }
            code.push(self.corners.lives[c]);
            i += 1;
        }
    }

    fn face(&mut self, f: FaceId, parent: Option<Piece>) -> Vec<u32> {
        if let Some(done) = self.face_memo.get(&(f, parent)) {
            return done.clone();
        }
        let mut inside: Vec<Vec<u32>> = self.pieces_of_face[f]
            .clone()
            .into_iter()
            .filter(|&p| Some(p) != parent)
            .map(|p| self.piece(p, Some(f)))
            .collect();
        inside.sort();
        let mut out = vec![FACE, inside.len() as u32];
        for form in inside {
            out.push(form.len() as u32);
            out.extend(form);
        }
        self.face_memo.insert((f, parent), out.clone());
        out
    }

    fn piece(&mut self, p: Piece, parent: Option<FaceId>) -> Vec<u32> {
        if let Some(done) = self.piece_memo.get(&(p, parent)) {
            return done.clone();
        }
        let mut best: Option<Vec<u32>> = None;
        let mut labels = vec![NONE; self.corners.lives.len()];
        let mut order = Vec::with_capacity(self.members[p].len());
        let mut code = Vec::with_capacity(3 * self.members[p].len());
        let mut lowest: HashMap<FaceId, u32> = HashMap::new();
        for root in self.members[p].clone() {
            self.code(root, &mut labels, &mut order, &mut code);
            lowest.clear();
            for &c in &order {
                let slot = lowest.entry(self.corners.face[c]).or_insert(NONE);
                *slot = (*slot).min(labels[c]);
            }
            let mut orbits: Vec<(u32, FaceId)> = lowest.iter().map(|(&f, &l)| (l, f)).collect();
            orbits.sort_unstable();
            let mut form = vec![PIECE, code.len() as u32];
            form.extend_from_slice(&code);
            for (lowest, f) in orbits {
                form.push(lowest);
                if Some(f) == parent {
                    form.push(0);
                } else {
                    let inner = self.face(f, Some(p));
                    form.push(inner.len() as u32);
                    form.extend(inner);
                }
            }
            if best.as_ref().is_none_or(|b| form < *b) {
                best = Some(form);
            }
        }
        let out = best.expect("a piece has a corner");
        self.piece_memo.insert((p, parent), out.clone());
        out
    }

    /// Centres of each tree of pieces and faces, by stripping leaves,
    /// one list per tree.
    fn centres(&self) -> Vec<Vec<(bool, usize)>> {
        let pieces = self.members.len();
        let faces = self.pieces_of_face.len();
        let total = pieces + faces;
        let mut degree = vec![0usize; total];
        for (p, list) in self.faces_of_piece.iter().enumerate() {
            degree[p] = list.len();
        }
        for (f, list) in self.pieces_of_face.iter().enumerate() {
            degree[pieces + f] = list.len();
        }
        let neighbours = |i: usize| -> Vec<usize> {
            if i < pieces {
                self.faces_of_piece[i].iter().map(|&f| pieces + f).collect()
            } else {
                self.pieces_of_face[i - pieces].clone()
            }
        };
        let mut tree = vec![NONE as usize; total];
        let mut trees = 0;
        for start in (0..total).filter(|&i| degree[i] > 0) {
            if tree[start] != NONE as usize {
                continue;
            }
            let mut stack = vec![start];
            tree[start] = trees;
            while let Some(i) = stack.pop() {
                for n in neighbours(i) {
                    if tree[n] == NONE as usize {
                        tree[n] = trees;
                        stack.push(n);
                    }
                }
            }
            trees += 1;
        }
        let mut remaining = vec![0usize; trees];
        for &t in tree.iter().filter(|&&t| t != NONE as usize) {
            remaining[t] += 1;
        }
        let mut alive: Vec<bool> = tree.iter().map(|&t| t != NONE as usize).collect();
        let mut leaves: Vec<usize> = (0..total).filter(|&i| degree[i] == 1).collect();
        while !leaves.is_empty() {
            let mut next = Vec::new();
            let stripping: Vec<bool> = remaining.iter().map(|&r| r > 2).collect();
            for &leaf in &leaves {
                if !stripping[tree[leaf]] {
                    continue;
                }
                alive[leaf] = false;
                remaining[tree[leaf]] -= 1;
                for n in neighbours(leaf) {
                    if alive[n] {
                        degree[n] -= 1;
                        if degree[n] == 1 {
                            next.push(n);
                        }
                    }
                }
            }
            leaves = next;
        }
        let mut out = vec![Vec::new(); trees];
        for i in (0..total).filter(|&i| alive[i]) {
            out[tree[i]].push(if i < pieces {
                (true, i)
            } else {
                (false, i - pieces)
            });
        }
        out
    }

    /// One form per tree, each the minimum over its centres.
    fn forms(&mut self) -> Vec<Vec<u32>> {
        self.centres()
            .into_iter()
            .map(|centres| {
                centres
                    .into_iter()
                    .map(|(is_piece, id)| {
                        if is_piece {
                            self.piece(id, None)
                        } else {
                            self.face(id, None)
                        }
                    })
                    .min()
                    .expect("a tree has a centre")
            })
            .collect()
    }
}

/// The form of the game on the faces of `m` from `frozen` on: the sorted
/// forms of its independent parts, each in the orientation that reads
/// lower. Empty when no node there has a life left.
pub(super) fn live_form(m: &FaceModel, frozen: usize) -> Vec<u32> {
    let corners = Corners::of(m, frozen);
    let forward = Tree::new(&corners, true).forms();
    let backward = Tree::new(&corners, false).forms();
    let mut parts: Vec<Vec<u32>> = forward
        .into_iter()
        .zip(backward)
        .map(|(f, b)| f.min(b))
        .collect();
    parts.sort();
    let mut out = Vec::new();
    for part in parts {
        out.push(part.len() as u32);
        out.extend(part);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::face::Move;
    use proptest::prelude::*;

    fn apply_all(n: usize, moves: &[Move]) -> FaceModel {
        let mut m = FaceModel::new(n);
        for mv in moves {
            m.apply(mv).unwrap();
        }
        m
    }

    fn chord(a: usize, b: usize) -> Move {
        Move {
            a,
            at_a: None,
            b,
            at_b: None,
            enclosed: vec![],
        }
    }

    fn lone_loop(a: usize, enclosed: Vec<usize>) -> Move {
        Move {
            a,
            at_a: None,
            b: a,
            at_b: None,
            enclosed,
        }
    }

    fn form(m: &FaceModel) -> Vec<u32> {
        live_form(m, 0)
    }

    #[test]
    fn fresh_boards_differ_by_size_only() {
        assert_eq!(form(&FaceModel::new(3)), form(&FaceModel::new(3)));
        assert_ne!(form(&FaceModel::new(3)), form(&FaceModel::new(4)));
        assert!(form(&FaceModel::new(1)).len() > 2);
    }

    #[test]
    fn renamed_nodes_and_reordered_moves_give_one_form() {
        let a = apply_all(4, &[lone_loop(0, vec![]), chord(1, 2)]);
        let b = apply_all(4, &[lone_loop(3, vec![]), chord(0, 2)]);
        let c = apply_all(4, &[chord(1, 3), lone_loop(2, vec![])]);
        assert_eq!(form(&a), form(&b));
        assert_eq!(form(&a), form(&c));
        assert_ne!(a, c);
    }

    #[test]
    fn what_a_loop_encloses_tells_positions_apart() {
        let open = apply_all(3, &[lone_loop(0, vec![])]);
        let walled = apply_all(3, &[lone_loop(0, vec![2])]);
        assert_ne!(form(&open), form(&walled));
    }

    fn find(m: &FaceModel, a: usize, b: usize) -> Move {
        m.moves()
            .into_iter()
            .find(|mv| mv.a == a && mv.b == b)
            .unwrap()
    }

    /// One node played out leaves two dead nodes on a loop and the
    /// chord's node with a life and no partner, so two corners remain. A
    /// chord and a second chord onto its node kill that node and leave
    /// five corners, four of them on nodes with one life.
    #[test]
    fn dead_nodes_leave_the_form() {
        let mut one = FaceModel::new(1);
        while let Some(mv) = one.moves().first().cloned() {
            one.apply(&mv).unwrap();
        }
        assert_eq!(one.node_count(), 3);
        let corners = Corners::of(&one, 0);
        assert_eq!(corners.lives, vec![1, 1]);
        assert_eq!(corners.partner, vec![Some(1), Some(0)]);
        assert_ne!(corners.face[0], corners.face[1]);

        let mut two = FaceModel::new(2);
        two.apply(&find(&two, 0, 1)).unwrap();
        two.apply(&find(&two, 0, 2)).unwrap();
        let outer = Corners::of(&two, 0);
        assert_eq!(outer.lives.len(), 5);
        assert_eq!(outer.lives.iter().filter(|&&l| l == 1).count(), 4);
    }

    #[test]
    fn a_mirror_image_has_the_same_form() {
        let mut m = FaceModel::new(3);
        m.apply(&find(&m, 0, 1)).unwrap();
        m.apply(&find(&m, 0, 2)).unwrap();
        let chiral = m
            .moves()
            .into_iter()
            .filter(|mv| mv.a == 0 || mv.b == 0)
            .map(|mv| {
                let mut probe = m.clone();
                probe.apply(&mv).unwrap();
                probe
            })
            .find(|probe| {
                crate::face::canonical_form(probe) != crate::face::canonical_form(&probe.mirror())
            })
            .expect("a chiral completion of node 0");
        assert_eq!(form(&chiral), form(&chiral.mirror()));
    }

    /// Every position reachable from `n` nodes, grouped by live form:
    /// one outcome per form. Prints the first disagreement.
    fn forms_agree_with_outcomes(n: usize) {
        use crate::face::canonical_form;
        use std::collections::HashMap;
        let mut outcomes: HashMap<Vec<u32>, bool> = HashMap::new();
        fn wins(m: &mut FaceModel, memo: &mut HashMap<Vec<u32>, bool>) -> bool {
            let key = canonical_form(m);
            if let Some(&w) = memo.get(&key) {
                return w;
            }
            let mut result = false;
            for mv in m.search_moves() {
                m.apply(&mv).unwrap();
                let opponent = wins(m, memo);
                m.undo();
                if !opponent {
                    result = true;
                    break;
                }
            }
            memo.insert(key, result);
            result
        }
        let mut by_form: HashMap<Vec<u32>, (bool, FaceModel)> = HashMap::new();
        let mut frontier = vec![FaceModel::new(n)];
        let mut seen = std::collections::HashSet::new();
        while let Some(m) = frontier.pop() {
            if !seen.insert(canonical_form(&m)) {
                continue;
            }
            let w = wins(&mut m.clone(), &mut outcomes);
            let f = form(&m);
            match by_form.get(&f) {
                Some((known, other)) => {
                    assert_eq!(*known, w, "{n} nodes: {other:?} and {m:?} share {f:?}");
                }
                None => {
                    by_form.insert(f, (w, m.clone()));
                }
            }
            for mv in m.moves() {
                let mut child = m.clone();
                child.apply(&mv).unwrap();
                frontier.push(child);
            }
        }
        println!(
            "{n} nodes: {} positions, {} live forms",
            seen.len(),
            by_form.len()
        );
    }

    /// Two games side by side read as the sorted pair of their forms.
    #[test]
    fn a_position_of_two_regions_has_both_parts_in_its_form() {
        let mut m = FaceModel::new(4);
        let loop_round = m
            .moves()
            .into_iter()
            .find(|mv| mv.a == 0 && mv.b == 0 && mv.enclosed == vec![2, 3])
            .unwrap();
        m.apply(&loop_round).unwrap();
        let inside = m.corner_face(1, None).unwrap();
        let chord = m
            .moves()
            .into_iter()
            .find(|mv| mv.a == 0 && mv.b == 4 && m.corner_face(0, mv.at_a) == Ok(inside))
            .unwrap();
        m.apply(&chord).unwrap();
        assert_eq!(m.regions().len(), 2);
        let mut parts: Vec<Vec<u32>> = m.regions().iter().map(|r| r.form().to_vec()).collect();
        parts.sort();
        assert_ne!(parts[0], parts[1]);
        assert_eq!(form(&m), parts.concat());
    }

    #[test]
    fn every_position_from_two_nodes_has_one_outcome_per_form() {
        forms_agree_with_outcomes(1);
        forms_agree_with_outcomes(2);
    }

    #[test]
    #[ignore = "about 4249 positions, run in release"]
    fn every_position_from_three_nodes_has_one_outcome_per_form() {
        forms_agree_with_outcomes(3);
    }

    /// Whether the player to move wins, over the reduced move list.
    fn first_player_wins(m: &mut FaceModel) -> bool {
        for mv in m.search_moves() {
            m.apply(&mv).unwrap();
            let opponent_wins = first_player_wins(m);
            m.undo();
            if !opponent_wins {
                return true;
            }
        }
        false
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(96))]

        /// Two random games that reach the same form reach the same
        /// outcome; a position renamed at random keeps its form.
        #[test]
        fn equal_live_forms_give_equal_outcomes(
            n in 1usize..=3,
            picks in prop::collection::vec(any::<u32>(), 9),
            other in prop::collection::vec(any::<u32>(), 9),
        ) {
            let mut seen: Vec<(Vec<u32>, bool)> = Vec::new();
            for picks in [&picks, &other] {
                let mut m = FaceModel::new(n);
                for &pick in picks {
                    let moves = m.moves();
                    if moves.is_empty() {
                        break;
                    }
                    m.apply(&moves[pick as usize % moves.len()]).unwrap();
                    let f = form(&m);
                    let wins = first_player_wins(&mut m.clone());
                    if let Some((_, known)) = seen.iter().find(|(g, _)| *g == f) {
                        prop_assert_eq!(*known, wins, "{:?}", m);
                    } else {
                        seen.push((f, wins));
                    }
                }
            }
        }
    }
}
