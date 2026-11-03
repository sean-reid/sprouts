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

use super::{Component, FaceId, FaceModel};

const NONE: u32 = u32::MAX;
const FACE: u32 = 0;
const PIECE: u32 = 1;

type Corner = usize;
type Piece = usize;
/// Start and length of a form in the scratch's store.
type Span = (usize, usize);

#[derive(Default)]
struct Corners {
    next: Vec<Corner>,
    prev: Vec<Corner>,
    partner: Vec<Option<Corner>>,
    lives: Vec<u32>,
    face: Vec<FaceId>,
}

impl Corners {
    #[cfg(test)]
    fn of(m: &FaceModel, frozen: usize) -> Self {
        let mut out = Self::default();
        out.fill(m, frozen, &mut Vec::new());
        out
    }

    fn clear(&mut self) {
        self.next.clear();
        self.prev.clear();
        self.partner.clear();
        self.lives.clear();
        self.face.clear();
    }

    /// The live corners of the faces from `frozen` on: one per half-edge
    /// out of a node with lives left, one per isolated node, boundary by
    /// boundary.
    fn fill(&mut self, m: &FaceModel, frozen: usize, of_dart: &mut Vec<usize>) {
        self.clear();
        of_dart.clear();
        of_dart.resize(m.half_edges.len(), NONE as usize);
        for f in frozen..m.faces.len() {
            for comp in &m.faces[f].components {
                let first = self.lives.len();
                match *comp {
                    Component::Node(_) => self.push(3, f),
                    Component::Cycle(start) => {
                        for h in m.cycle(start) {
                            let degree = m.nodes[m.origin(h)].degree;
                            if degree < 3 {
                                of_dart[h] = self.lives.len();
                                self.push(3 - u32::from(degree), f);
                            }
                        }
                    }
                }
                let last = self.lives.len();
                if last > first {
                    self.next[last - 1] = first;
                    self.prev[first] = last - 1;
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
                self.partner[c] = Some(other).filter(|&o| o != NONE as usize);
            }
        }
    }

    fn push(&mut self, lives: u32, face: FaceId) {
        let c = self.lives.len();
        self.next.push(c + 1);
        self.prev.push(c.wrapping_sub(1));
        self.partner.push(None);
        self.lives.push(lives);
        self.face.push(face);
    }
}

fn find(parent: &mut [usize], mut v: usize) -> usize {
    while parent[v] != v {
        parent[v] = parent[parent[v]];
        v = parent[v];
    }
    v
}

fn reset<T>(lists: &mut Vec<Vec<T>>, n: usize) {
    lists.truncate(n);
    lists.iter_mut().for_each(Vec::clear);
    lists.resize_with(n, Vec::new);
}

/// Buffers for building live forms, kept between calls so a search pays
/// for allocation once.
#[derive(Default)]
pub struct LiveScratch {
    corners: Corners,
    of_dart: Vec<usize>,
    parent: Vec<usize>,
    number: Vec<usize>,
    piece_of: Vec<Piece>,
    forward: bool,
    /// Corners of each piece, ascending.
    members: Vec<Vec<Corner>>,
    /// Faces round each piece, ascending.
    faces_of_piece: Vec<Vec<FaceId>>,
    /// Pieces touching each face, ascending; indexed by face id.
    pieces_of_face: Vec<Vec<Piece>>,
    store: Vec<u32>,
    face_memo: Vec<Span>,
    piece_memo: Vec<Span>,
    spans: Vec<Span>,
    inner: Vec<Span>,
    labels: Vec<u32>,
    order: Vec<Corner>,
    code: Vec<u32>,
    lowest: Vec<u32>,
    orbits: Vec<(u32, FaceId, usize)>,
    kept: Vec<u32>,
    kept_spans: Vec<Span>,
    out: Vec<u32>,
}

impl LiveScratch {
    /// `live_form` into the scratch's own buffer.
    pub fn form(&mut self, m: &FaceModel, frozen: usize) -> &[u32] {
        let mut corners = std::mem::take(&mut self.corners);
        corners.fill(m, frozen, &mut self.of_dart);
        self.corners = corners;
        self.out.clear();
        if self.corners.lives.is_empty() {
            return &self.out;
        }
        self.build_pieces();
        let centres = self.centres();

        self.kept.clear();
        self.kept_spans.clear();
        self.forward = true;
        self.start_orientation();
        for tree in &centres {
            let span = self.tree_form(tree);
            let at = self.kept.len();
            self.kept
                .extend_from_slice(&self.store[span.0..span.0 + span.1]);
            self.kept_spans.push((at, span.1));
        }
        self.forward = false;
        self.start_orientation();
        for (i, tree) in centres.iter().enumerate() {
            let back = self.tree_form(tree);
            let (at, len) = self.kept_spans[i];
            if self.store[back.0..back.0 + back.1] < self.kept[at..at + len] {
                let at = self.kept.len();
                self.kept
                    .extend_from_slice(&self.store[back.0..back.0 + back.1]);
                self.kept_spans[i] = (at, back.1);
            }
        }
        let kept = &self.kept;
        self.kept_spans
            .sort_unstable_by(|a, b| kept[a.0..a.0 + a.1].cmp(&kept[b.0..b.0 + b.1]));
        for &(at, len) in &self.kept_spans {
            self.out.push(len as u32);
            self.out.extend_from_slice(&self.kept[at..at + len]);
        }
        &self.out
    }

    fn start_orientation(&mut self) {
        self.store.clear();
        let faces = self.pieces_of_face.len();
        let pieces = self.members.len();
        self.face_memo.clear();
        self.face_memo.resize(faces * (pieces + 1), (usize::MAX, 0));
        self.piece_memo.clear();
        self.piece_memo
            .resize(pieces * (faces + 1), (usize::MAX, 0));
        self.labels.clear();
        self.labels.resize(self.corners.lives.len(), NONE);
        self.order.clear();
        self.lowest.clear();
        self.lowest.resize(faces, NONE);
    }

    /// Corners joined by `next` and `partner` are one piece, numbered
    /// from zero; fills the piece and face incidence lists.
    fn build_pieces(&mut self) {
        let n = self.corners.lives.len();
        self.parent.clear();
        self.parent.extend(0..n);
        for c in 0..n {
            let (next, partner) = (self.corners.next[c], self.corners.partner[c]);
            for other in [Some(next), partner].into_iter().flatten() {
                let (a, b) = (find(&mut self.parent, c), find(&mut self.parent, other));
                self.parent[a] = b;
            }
        }
        self.number.clear();
        self.number.resize(n, NONE as usize);
        self.piece_of.clear();
        let mut count = 0;
        for c in 0..n {
            let root = find(&mut self.parent, c);
            if self.number[root] == NONE as usize {
                self.number[root] = count;
                count += 1;
            }
            self.piece_of.push(self.number[root]);
        }
        let faces = self.corners.face.iter().copied().max().map_or(0, |f| f + 1);
        reset(&mut self.members, count);
        reset(&mut self.faces_of_piece, count);
        reset(&mut self.pieces_of_face, faces);
        for (c, &p) in self.piece_of.iter().enumerate() {
            let f = self.corners.face[c];
            self.members[p].push(c);
            self.faces_of_piece[p].push(f);
            self.pieces_of_face[f].push(p);
        }
        for list in self
            .faces_of_piece
            .iter_mut()
            .chain(self.pieces_of_face.iter_mut())
        {
            list.sort_unstable();
            list.dedup();
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
    fn code_from(&mut self, root: Corner) {
        for &c in &self.order {
            self.labels[c] = NONE;
        }
        self.order.clear();
        self.code.clear();
        self.order.push(root);
        self.labels[root] = 0;
        let mut i = 0;
        while i < self.order.len() {
            let c = self.order[i];
            for e in [Some(self.step(c)), self.corners.partner[c]] {
                let Some(e) = e else {
                    self.code.push(NONE);
                    continue;
                };
                if self.labels[e] == NONE {
                    self.labels[e] = self.order.len() as u32;
                    self.order.push(e);
                }
                self.code.push(self.labels[e]);
            }
            self.code.push(self.corners.lives[c]);
            i += 1;
        }
    }

    fn face(&mut self, f: FaceId, parent: Option<Piece>) -> Span {
        let slot = f * (self.members.len() + 1) + parent.unwrap_or(self.members.len());
        if self.face_memo[slot].0 != usize::MAX {
            return self.face_memo[slot];
        }
        let base = self.spans.len();
        for i in 0..self.pieces_of_face[f].len() {
            let p = self.pieces_of_face[f][i];
            if Some(p) != parent {
                let span = self.piece(p, Some(f));
                self.spans.push(span);
            }
        }
        let store = &self.store;
        self.spans[base..]
            .sort_unstable_by(|a, b| store[a.0..a.0 + a.1].cmp(&store[b.0..b.0 + b.1]));
        let start = self.store.len();
        self.store.push(FACE);
        self.store.push((self.spans.len() - base) as u32);
        for i in base..self.spans.len() {
            let (at, len) = self.spans[i];
            self.store.push(len as u32);
            self.store.extend_from_within(at..at + len);
        }
        self.spans.truncate(base);
        let out = (start, self.store.len() - start);
        self.face_memo[slot] = out;
        out
    }

    fn piece(&mut self, p: Piece, parent: Option<FaceId>) -> Span {
        let slot =
            p * (self.pieces_of_face.len() + 1) + parent.unwrap_or(self.pieces_of_face.len());
        if self.piece_memo[slot].0 != usize::MAX {
            return self.piece_memo[slot];
        }
        let base = self.inner.len();
        for i in 0..self.faces_of_piece[p].len() {
            let f = self.faces_of_piece[p][i];
            let span = if Some(f) == parent {
                (0, 0)
            } else {
                self.face(f, Some(p))
            };
            self.inner.push(span);
        }
        let mut best: Span = (usize::MAX, 0);
        for r in 0..self.members[p].len() {
            let root = self.members[p][r];
            self.code_from(root);
            for &c in &self.order {
                let f = self.corners.face[c];
                self.lowest[f] = self.lowest[f].min(self.labels[c]);
            }
            self.orbits.clear();
            for (i, &f) in self.faces_of_piece[p].iter().enumerate() {
                self.orbits.push((self.lowest[f], f, i));
                self.lowest[f] = NONE;
            }
            self.orbits.sort_unstable();
            let start = self.store.len();
            self.store.push(PIECE);
            self.store.push(self.code.len() as u32);
            self.store.extend_from_slice(&self.code);
            for &(low, f, i) in &self.orbits {
                self.store.push(low);
                if Some(f) == parent {
                    self.store.push(0);
                } else {
                    let (at, len) = self.inner[base + i];
                    self.store.push(len as u32);
                    self.store.extend_from_within(at..at + len);
                }
            }
            let len = self.store.len() - start;
            if best.0 == usize::MAX || self.store[start..] < self.store[best.0..best.0 + best.1] {
                best = (start, len);
            } else {
                self.store.truncate(start);
            }
        }
        self.inner.truncate(base);
        self.piece_memo[slot] = best;
        best
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

    /// The form of one tree: the minimum over its centres.
    fn tree_form(&mut self, centres: &[(bool, usize)]) -> Span {
        let mut best: Option<Span> = None;
        for &(is_piece, id) in centres {
            let span = if is_piece {
                self.piece(id, None)
            } else {
                self.face(id, None)
            };
            let better = best
                .is_none_or(|b| self.store[span.0..span.0 + span.1] < self.store[b.0..b.0 + b.1]);
            if better {
                best = Some(span);
            }
        }
        best.expect("a tree has a centre")
    }
}

/// The form of the game on the faces of `m` from `frozen` on: the sorted
/// forms of its independent parts, each in the orientation that reads
/// lower and led by its length, so `parts` splits them out again. Empty
/// when no node there has a life left.
pub fn live_form(m: &FaceModel, frozen: usize) -> Vec<u32> {
    LiveScratch::default().form(m, frozen).to_vec()
}

/// The forms of a position's regions, as `Region::form` gives them, read
/// off the position's own form.
pub fn parts(form: &[u32]) -> Vec<&[u32]> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < form.len() {
        let end = at + 1 + form[at] as usize;
        out.push(&form[at..end]);
        at = end;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::face::Move;
    use proptest::prelude::*;
    use std::collections::HashMap;

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

    /// The nimber of the whole position by the mex recursion over the
    /// rotation-system form, which keeps every position apart.
    fn plain_nimber(m: &mut FaceModel, memo: &mut HashMap<Vec<u32>, u32>) -> u32 {
        let key = crate::face::canonical_form(m);
        if let Some(&v) = memo.get(&key) {
            return v;
        }
        let mut seen: u64 = 0;
        for mv in m.search_moves() {
            m.apply(&mv).unwrap();
            seen |= 1 << plain_nimber(m, memo);
            m.undo();
        }
        let value = seen.trailing_ones();
        memo.insert(key, value);
        value
    }

    /// Every position reachable from `n` nodes, grouped by live form:
    /// one nimber per form, and the count of each.
    fn forms_agree_with_nimbers(n: usize) -> (usize, usize) {
        use crate::face::canonical_form;
        let mut nimbers: HashMap<Vec<u32>, u32> = HashMap::new();
        let mut by_form: HashMap<Vec<u32>, (u32, FaceModel)> = HashMap::new();
        let mut frontier = vec![FaceModel::new(n)];
        let mut seen = std::collections::HashSet::new();
        while let Some(m) = frontier.pop() {
            if !seen.insert(canonical_form(&m)) {
                continue;
            }
            let value = plain_nimber(&mut m.clone(), &mut nimbers);
            let f = form(&m);
            match by_form.get(&f) {
                Some((known, other)) => {
                    assert_eq!(*known, value, "{n} nodes: {other:?} and {m:?} share {f:?}");
                }
                None => {
                    by_form.insert(f, (value, m.clone()));
                }
            }
            for mv in m.moves() {
                let mut child = m.clone();
                child.apply(&mv).unwrap();
                frontier.push(child);
            }
        }
        (seen.len(), by_form.len())
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
        let mut forms: Vec<Vec<u32>> = m.regions().iter().map(|r| r.form().to_vec()).collect();
        forms.sort();
        assert_ne!(forms[0], forms[1]);
        assert_eq!(form(&m), forms.concat());
        let split: Vec<&[u32]> = forms.iter().map(Vec::as_slice).collect();
        assert_eq!(parts(&form(&m)), split);
        assert_eq!(parts(&form(&FaceModel::new(3))).len(), 1);
        assert!(parts(&[]).is_empty());
    }

    #[test]
    fn every_position_from_two_nodes_has_one_nimber_per_form() {
        assert_eq!(forms_agree_with_nimbers(1), (3, 3));
        assert_eq!(forms_agree_with_nimbers(2), (57, 25));
    }

    /// About a second in release; a debug build takes twenty times that.
    #[cfg(not(debug_assertions))]
    #[test]
    fn every_position_from_three_nodes_has_one_nimber_per_form() {
        assert_eq!(forms_agree_with_nimbers(3), (4249, 248));
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
