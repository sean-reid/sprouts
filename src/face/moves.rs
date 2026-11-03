//! Move enumeration over the rotation system.

use super::{FaceModel, HalfEdgeId, Move, NodeId};

impl FaceModel {
    /// Every legal move, each face-side choice listed separately, in a
    /// fixed order. Walks every face, O(E), and allocates 2^k moves for a
    /// split with k other pieces, so call it once per position.
    pub fn moves(&self) -> Vec<Move> {
        let bounds = self.boundaries();
        let mut out = Vec::new();
        for a in 0..self.nodes.len() {
            let da = self.nodes[a].degree;
            if da == 3 {
                continue;
            }
            if da <= 1 {
                let at = self.nodes[a].rotation().first().copied();
                let face = self.corner_face(a, at).expect("own corner");
                let own = bounds.component(a, at);
                let mut others = other_reps(&bounds.reps[face], own, own);
                if da == 0 && !others.is_empty() {
                    others.remove(0);
                }
                for enclosed in subsets(&others) {
                    out.push(Move {
                        a,
                        at_a: at,
                        b: a,
                        at_b: at,
                        enclosed,
                    });
                }
            }
            for b in a + 1..self.nodes.len() {
                if self.nodes[b].degree == 3 {
                    continue;
                }
                for at_a in self.corners(a) {
                    let face = self.corner_face(a, at_a).expect("own corner");
                    for at_b in self.corners(b) {
                        if self.corner_face(b, at_b) != Ok(face) {
                            continue;
                        }
                        let ca = bounds.component(a, at_a);
                        let cb = bounds.component(b, at_b);
                        let choices = if ca == cb {
                            subsets(&other_reps(&bounds.reps[face], ca, cb))
                        } else {
                            vec![Vec::new()]
                        };
                        for enclosed in choices {
                            out.push(Move {
                                a,
                                at_a,
                                b,
                                at_b,
                                enclosed,
                            });
                        }
                    }
                }
            }
        }
        out
    }

    fn corners(&self, v: NodeId) -> Vec<Option<HalfEdgeId>> {
        let node = &self.nodes[v];
        if node.degree == 0 {
            vec![None]
        } else {
            node.rotation().iter().map(|&h| Some(h)).collect()
        }
    }
}

fn other_reps(reps: &[NodeId], ca: usize, cb: usize) -> Vec<NodeId> {
    reps.iter()
        .enumerate()
        .filter(|&(i, _)| i != ca && i != cb)
        .map(|(_, &r)| r)
        .collect()
}

fn subsets(items: &[NodeId]) -> Vec<Vec<NodeId>> {
    (0..1u64 << items.len())
        .map(|mask| {
            items
                .iter()
                .enumerate()
                .filter(|&(i, _)| mask >> i & 1 == 1)
                .map(|(_, &r)| r)
                .collect()
        })
        .collect()
}
