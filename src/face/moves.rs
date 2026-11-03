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

    /// One move per class of `moves` that reach the same position up to
    /// relabelling the isolated nodes of a face, in a fixed order. Those
    /// nodes are interchangeable, so a loop or chord is listed from the
    /// lowest of them and an enclosure takes the lowest few rather than
    /// every subset; the count is polynomial in the isolated nodes.
    pub fn search_moves(&self) -> Vec<Move> {
        let bounds = self.boundaries();
        let mut isolated: Vec<Vec<NodeId>> = vec![Vec::new(); self.faces.len()];
        for (v, node) in self.nodes.iter().enumerate() {
            if node.degree == 0 {
                isolated[node.home].push(v);
            }
        }
        let mut out = Vec::new();
        for a in 0..self.nodes.len() {
            let da = self.nodes[a].degree;
            if da == 3 || (da == 0 && isolated[self.nodes[a].home][0] != a) {
                continue;
            }
            if da <= 1 {
                let at = self.nodes[a].rotation().first().copied();
                let face = self.corner_face(a, at).expect("own corner");
                let own = bounds.component(a, at);
                let (iso, mut cycles) = self.sort_reps(other_reps(&bounds.reps[face], own, own));
                // Both sides of a loop on an isolated node are the same
                // position, so one other piece is held outside.
                let counts = if da == 0 && !cycles.is_empty() {
                    cycles.remove(0);
                    iso.len()
                } else if da == 0 {
                    iso.len() / 2
                } else {
                    iso.len()
                };
                for enclosed in enclosures(&iso, counts, &cycles) {
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
                let db = self.nodes[b].degree;
                if db == 3 {
                    continue;
                }
                if db == 0 {
                    let peers = &isolated[self.nodes[b].home];
                    let stands = if da == 0 {
                        peers.get(1) == Some(&b)
                    } else {
                        peers[0] == b
                    };
                    if !stands {
                        continue;
                    }
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
                            let (iso, cycles) =
                                self.sort_reps(other_reps(&bounds.reps[face], ca, cb));
                            enclosures(&iso, iso.len(), &cycles)
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

    /// Splits piece representatives into isolated nodes and cycles.
    fn sort_reps(&self, reps: Vec<NodeId>) -> (Vec<NodeId>, Vec<NodeId>) {
        reps.into_iter().partition(|&r| self.nodes[r].degree == 0)
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

/// Every subset of `cycles` joined with the lowest `0..=counts` of `iso`.
fn enclosures(iso: &[NodeId], counts: usize, cycles: &[NodeId]) -> Vec<Vec<NodeId>> {
    subsets(cycles)
        .into_iter()
        .flat_map(|subset| {
            (0..=counts).map(move |c| {
                let mut enclosed = subset.clone();
                enclosed.extend_from_slice(&iso[..c]);
                enclosed
            })
        })
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
