//! Canonical form of a position: equal for two positions exactly when one
//! is the other with its nodes and half-edges renamed.
//!
//! The graph's connected pieces and the faces form a tree, since a
//! connected piece splits the sphere and no other piece can touch two of
//! the parts. Each piece is written as the breadth-first code of its
//! rotation system from a root half-edge, the faces round it following in
//! the order the code gives them, each face holding the sorted forms of
//! the pieces inside it. The minimum over root half-edges, and over the
//! tree's one or two centres, is the form.

use std::collections::HashMap;

use super::{Component, FaceId, FaceModel, HalfEdgeId};

const FACE: u32 = 0;
const PIECE: u32 = 1;
const ISOLATED: u32 = 2;

type Piece = usize;

/// Which piece each node belongs to, pieces numbered from zero.
fn pieces(m: &FaceModel) -> (Vec<Piece>, usize) {
    let n = m.nodes.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }
    for h in (0..m.half_edges.len()).step_by(2) {
        let a = find(&mut parent, m.origin(h));
        let b = find(&mut parent, m.head(h));
        parent[a] = b;
    }
    let mut number = vec![usize::MAX; n];
    let mut count = 0;
    let mut of_node = vec![0; n];
    for (v, slot) in of_node.iter_mut().enumerate() {
        let root = find(&mut parent, v);
        if number[root] == usize::MAX {
            number[root] = count;
            count += 1;
        }
        *slot = number[root];
    }
    (of_node, count)
}

struct Tree<'a> {
    m: &'a FaceModel,
    /// Half-edges of each piece, ascending.
    darts: Vec<Vec<HalfEdgeId>>,
    /// Faces round each piece, ascending.
    faces_of_piece: Vec<Vec<FaceId>>,
    /// Pieces touching each face, ascending.
    pieces_of_face: Vec<Vec<Piece>>,
    face_memo: HashMap<(FaceId, Option<Piece>), Vec<u32>>,
    piece_memo: HashMap<(Piece, Option<FaceId>), Vec<u32>>,
}

impl<'a> Tree<'a> {
    fn new(m: &'a FaceModel) -> Self {
        let (piece_of_node, count) = pieces(m);
        let mut darts = vec![Vec::new(); count];
        for (h, e) in m.half_edges.iter().enumerate() {
            darts[piece_of_node[e.origin]].push(h);
        }
        let mut faces_of_piece = vec![Vec::new(); count];
        let mut pieces_of_face = vec![Vec::new(); m.faces.len()];
        for (f, face) in m.faces.iter().enumerate() {
            for comp in &face.components {
                let piece = match *comp {
                    Component::Node(v) => piece_of_node[v],
                    Component::Cycle(h) => piece_of_node[m.origin(h)],
                };
                faces_of_piece[piece].push(f);
                pieces_of_face[f].push(piece);
            }
        }
        for list in &mut faces_of_piece {
            list.sort_unstable();
            list.dedup();
        }
        for list in &mut pieces_of_face {
            list.sort_unstable();
            list.dedup();
        }
        Self {
            m,
            darts,
            faces_of_piece,
            pieces_of_face,
            face_memo: HashMap::new(),
            piece_memo: HashMap::new(),
        }
    }

    /// Next half-edge counterclockwise round the origin of `h`.
    fn sigma(&self, h: HalfEdgeId) -> HalfEdgeId {
        let node = &self.m.nodes[self.m.origin(h)];
        let rotation = node.rotation();
        let at = node.position(h);
        rotation[(at + 1) % rotation.len()]
    }

    /// Breadth-first code of the piece holding `root`. `labels` is indexed
    /// by half-edge and holds `u32::MAX` for half-edges not yet reached;
    /// `order` lists the piece's half-edges in label order.
    fn code(
        &self,
        root: HalfEdgeId,
        labels: &mut [u32],
        order: &mut Vec<HalfEdgeId>,
        code: &mut Vec<u32>,
    ) {
        for &d in order.iter() {
            labels[d] = u32::MAX;
        }
        order.clear();
        code.clear();
        order.push(root);
        labels[root] = 0;
        let mut i = 0;
        while i < order.len() {
            let d = order[i];
            for e in [d ^ 1, self.sigma(d)] {
                if labels[e] == u32::MAX {
                    labels[e] = order.len() as u32;
                    order.push(e);
                }
                code.push(labels[e]);
            }
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
        let out = if self.darts[p].is_empty() {
            vec![ISOLATED]
        } else {
            let mut best: Option<Vec<u32>> = None;
            let mut labels = vec![u32::MAX; self.m.half_edges.len()];
            let mut order = Vec::with_capacity(self.darts[p].len());
            let mut code = Vec::with_capacity(2 * self.darts[p].len());
            let mut lowest = vec![u32::MAX; self.m.faces.len()];
            for root in self.darts[p].clone() {
                self.code(root, &mut labels, &mut order, &mut code);
                for &f in &self.faces_of_piece[p] {
                    lowest[f] = u32::MAX;
                }
                for &d in &order {
                    let f = self.m.face_of(d);
                    lowest[f] = lowest[f].min(labels[d]);
                }
                let mut orbits: Vec<(u32, FaceId)> = self.faces_of_piece[p]
                    .iter()
                    .map(|&f| (lowest[f], f))
                    .collect();
                orbits.sort_unstable();
                let mut form = vec![PIECE, code.len() as u32];
                form.extend_from_slice(&code);
                for (lowest, f) in orbits {
                    if Some(f) == parent {
                        form.push(lowest);
                        form.push(0);
                    } else {
                        let inner = self.face(f, Some(p));
                        form.push(lowest);
                        form.push(inner.len() as u32);
                        form.extend(inner);
                    }
                }
                if best.as_ref().is_none_or(|b| form < *b) {
                    best = Some(form);
                }
            }
            best.expect("a piece with half-edges has a root")
        };
        self.piece_memo.insert((p, parent), out.clone());
        out
    }

    /// Centres of the piece and face tree: pieces first, then faces, by
    /// repeatedly stripping leaves.
    fn centres(&self) -> Vec<(bool, usize)> {
        let pieces = self.darts.len();
        let faces = self.m.faces.len();
        let total = pieces + faces;
        let mut degree = vec![0usize; total];
        for (p, list) in self.faces_of_piece.iter().enumerate() {
            degree[p] = list.len();
        }
        for (f, list) in self.pieces_of_face.iter().enumerate() {
            degree[pieces + f] = list.len();
        }
        let mut alive = vec![true; total];
        let mut remaining = total;
        let mut leaves: Vec<usize> = (0..total).filter(|&i| degree[i] <= 1).collect();
        while remaining > 2 {
            let mut next = Vec::new();
            for &leaf in &leaves {
                alive[leaf] = false;
                remaining -= 1;
                let neighbours: Vec<usize> = if leaf < pieces {
                    self.faces_of_piece[leaf]
                        .iter()
                        .map(|&f| pieces + f)
                        .collect()
                } else {
                    self.pieces_of_face[leaf - pieces].clone()
                };
                for n in neighbours {
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
        (0..total)
            .filter(|&i| alive[i])
            .map(|i| {
                if i < pieces {
                    (true, i)
                } else {
                    (false, i - pieces)
                }
            })
            .collect()
    }
}

/// The position with every name removed, as a word that two positions
/// share exactly when one is the other renamed. Mirror images differ.
pub fn canonical_form(m: &FaceModel) -> Vec<u32> {
    let mut tree = Tree::new(m);
    tree.centres()
        .into_iter()
        .map(|(is_piece, id)| {
            if is_piece {
                tree.piece(id, None)
            } else {
                tree.face(id, None)
            }
        })
        .min()
        .expect("a position has a centre")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::face::Move;

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

    #[test]
    fn fresh_boards_of_a_size_share_a_form_and_sizes_differ() {
        assert_eq!(
            canonical_form(&FaceModel::new(4)),
            canonical_form(&FaceModel::new(4))
        );
        assert_ne!(
            canonical_form(&FaceModel::new(4)),
            canonical_form(&FaceModel::new(5))
        );
    }

    #[test]
    fn the_same_moves_in_another_order_give_one_form_and_different_half_edge_ids() {
        let first = apply_all(4, &[chord(0, 1), chord(2, 3)]);
        let second = apply_all(4, &[chord(2, 3), chord(0, 1)]);
        assert_ne!(first, second);
        assert_eq!(canonical_form(&first), canonical_form(&second));
    }

    #[test]
    fn renamed_nodes_give_one_form() {
        let a = apply_all(4, &[lone_loop(0, vec![]), chord(1, 2)]);
        let b = apply_all(4, &[lone_loop(3, vec![]), chord(0, 2)]);
        let c = apply_all(4, &[chord(1, 3), lone_loop(2, vec![])]);
        assert_eq!(canonical_form(&a), canonical_form(&b));
        assert_eq!(canonical_form(&a), canonical_form(&c));
    }

    #[test]
    fn what_a_loop_encloses_tells_positions_apart() {
        let open = apply_all(3, &[lone_loop(0, vec![])]);
        let walled = apply_all(3, &[lone_loop(0, vec![2])]);
        let walled_other = apply_all(3, &[lone_loop(0, vec![1])]);
        assert_ne!(canonical_form(&open), canonical_form(&walled));
        assert_eq!(canonical_form(&walled), canonical_form(&walled_other));
    }

    /// The model lives on the sphere, so a loop with a pendant chord on one
    /// side and a lone node on the other is one position whichever side
    /// the loop was drawn round.
    #[test]
    fn the_two_sides_of_a_loop_are_the_same_on_the_sphere() {
        let mut pendant_inside = apply_all(3, &[lone_loop(0, vec![1])]);
        let mut pendant_outside = pendant_inside.clone();
        let inside = pendant_inside
            .moves()
            .into_iter()
            .find(|mv| mv.a == 1 && mv.b == 3)
            .unwrap();
        pendant_inside.apply(&inside).unwrap();
        let outside = pendant_outside
            .moves()
            .into_iter()
            .find(|mv| mv.a == 2 && mv.b == 3)
            .unwrap();
        pendant_outside.apply(&outside).unwrap();
        assert_ne!(pendant_inside, pendant_outside);
        assert_eq!(
            canonical_form(&pendant_inside),
            canonical_form(&pendant_outside)
        );
    }

    #[test]
    fn which_side_of_a_loop_holds_the_chord_and_the_node_tells_positions_apart() {
        let apart = apply_all(4, &[lone_loop(0, vec![1]), chord(2, 3)]);
        let together = apply_all(4, &[lone_loop(0, vec![]), chord(1, 2)]);
        assert_eq!(apart.edge_count(), together.edge_count());
        assert_eq!(apart.face_count(), together.face_count());
        assert_ne!(canonical_form(&apart), canonical_form(&together));
    }

    #[test]
    fn undo_and_redo_keep_the_form() {
        let mut m = apply_all(5, &[chord(0, 1), lone_loop(2, vec![3])]);
        let before = canonical_form(&m);
        let mv = m.moves()[0].clone();
        m.apply(&mv).unwrap();
        assert_ne!(canonical_form(&m), before);
        m.undo();
        assert_eq!(canonical_form(&m), before);
    }
}
