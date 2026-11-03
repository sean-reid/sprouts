//! Regions: the independent games a position is the sum of.
//!
//! A move joins two live corners of one face and changes only that face,
//! but a node with two edges can have a corner in each of two faces, and
//! a move at either corner spends its last life for both. So the games
//! that cannot touch each other are the sets of faces linked by shared
//! live nodes. Such a set is cut out as a model of its own holding its
//! faces and every piece on their boundaries, with each other face round
//! one of those pieces kept as a frozen face: it admits no move, and the
//! form marks it, since the same pieces can bound two different games
//! from their two sides.

use super::canonical::pieces;
use super::live::live_form;
use super::{Component, Face, FaceId, FaceModel, HalfEdge, Move, Node};

/// One independent game: a model whose first `frozen` faces are borrowed
/// from the rest of the position, and the live form of the game on the
/// other faces.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Region {
    model: FaceModel,
    frozen: usize,
    form: Vec<u32>,
}

impl Region {
    pub fn model(&self) -> &FaceModel {
        &self.model
    }

    /// Equal for two regions with the same game once dead nodes are
    /// forgotten, mirror images included.
    pub fn form(&self) -> &[u32] {
        &self.form
    }

    pub fn node_count(&self) -> usize {
        self.model.node_count()
    }

    /// Faces the game is played in, after the frozen ones.
    pub fn face_count(&self) -> usize {
        self.model.face_count() - self.frozen
    }

    /// Lives left on the nodes with a corner in the region's own faces,
    /// the most moves its game can last.
    pub fn lives(&self) -> usize {
        (0..self.model.node_count())
            .filter(|&v| self.plays(v))
            .map(|v| 3 - usize::from(self.model.degree(v)))
            .sum()
    }

    fn plays(&self, v: usize) -> bool {
        match self.model.degree(v) {
            0 => self.model.corner_face(v, None).expect("home face") >= self.frozen,
            3 => false,
            _ => self
                .model
                .rotation(v)
                .iter()
                .any(|&h| self.model.face_of(h) >= self.frozen),
        }
    }

    /// The region's moves, one per class as `FaceModel::search_moves`,
    /// and a split never encloses a piece with no live node in the
    /// region's own faces, since the side such a piece lands on makes no
    /// difference to the game.
    pub fn moves(&self) -> Vec<Move> {
        let (piece_of_node, count) = pieces(&self.model);
        let mut dead = vec![true; count];
        for v in 0..self.model.node_count() {
            if self.plays(v) {
                dead[piece_of_node[v]] = false;
            }
        }
        let keep = |mv: &Move| {
            self.model.corner_face(mv.a, mv.at_a).expect("own corner") >= self.frozen
                && !mv.enclosed.iter().any(|&r| dead[piece_of_node[r]])
        };
        self.model.search_moves().into_iter().filter(keep).collect()
    }

    /// The regions `mv` leaves, taken on a copy of the model.
    pub fn after(&self, mv: &Move) -> Vec<Region> {
        let mut model = self.model.clone();
        model.apply(mv).expect("a listed move applies");
        model.regions_from(self.frozen)
    }
}

impl FaceModel {
    /// The position's independent games, each face with a live node in
    /// exactly one of them, ordered by lowest face.
    pub fn regions(&self) -> Vec<Region> {
        self.regions_from(0)
    }

    /// `regions` over the faces from `frozen` on.
    fn regions_from(&self, frozen: usize) -> Vec<Region> {
        let mut parent: Vec<FaceId> = (0..self.faces.len()).collect();
        fn find(parent: &mut [FaceId], mut f: FaceId) -> FaceId {
            while parent[f] != f {
                parent[f] = parent[parent[f]];
                f = parent[f];
            }
            f
        }
        let mut live = vec![false; self.faces.len()];
        for node in &self.nodes {
            if node.degree == 0 && node.home >= frozen {
                live[node.home] = true;
            }
            if node.degree == 3 {
                continue;
            }
            let mut corners = node
                .rotation()
                .iter()
                .map(|&h| self.face_of(h))
                .filter(|&f| f >= frozen);
            if let Some(first) = corners.next() {
                live[first] = true;
                for other in corners {
                    live[other] = true;
                    let (a, b) = (find(&mut parent, first), find(&mut parent, other));
                    parent[a] = b;
                }
            }
        }
        let (piece_of_node, piece_count) = pieces(self);
        let mut groups: Vec<Vec<FaceId>> = Vec::new();
        let mut group_of_root = vec![usize::MAX; self.faces.len()];
        for f in (0..self.faces.len()).filter(|&f| live[f]) {
            let root = find(&mut parent, f);
            if group_of_root[root] == usize::MAX {
                group_of_root[root] = groups.len();
                groups.push(Vec::new());
            }
            groups[group_of_root[root]].push(f);
        }
        groups
            .iter()
            .map(|faces| self.extract(faces, &piece_of_node, piece_count))
            .collect()
    }

    /// The model cut down to `faces` and the pieces on their boundaries.
    /// Each other face round one of those pieces comes first, frozen,
    /// with that one boundary, so the result is a position on the sphere.
    fn extract(&self, faces: &[FaceId], piece_of_node: &[usize], piece_count: usize) -> Region {
        let mut keep = vec![false; piece_count];
        for &f in faces {
            for comp in &self.faces[f].components {
                keep[piece_of_node[self.lowest_node(*comp)]] = true;
            }
        }
        let kept = |comp: &Component| keep[piece_of_node[self.lowest_node(*comp)]];

        let mut new_node = vec![usize::MAX; self.nodes.len()];
        let mut node_count = 0;
        for (v, slot) in new_node.iter_mut().enumerate() {
            if keep[piece_of_node[v]] {
                *slot = node_count;
                node_count += 1;
            }
        }
        let mut new_dart = vec![usize::MAX; self.half_edges.len()];
        let mut dart_count = 0;
        for (h, slot) in new_dart.iter_mut().enumerate() {
            if keep[piece_of_node[self.half_edges[h].origin]] {
                *slot = dart_count;
                dart_count += 1;
            }
        }
        let mut new_face = vec![usize::MAX; self.faces.len()];
        let mut own = vec![false; self.faces.len()];
        for &f in faces {
            own[f] = true;
        }
        let mut face_count = 0;
        for (f, slot) in new_face.iter_mut().enumerate() {
            if !own[f] && self.faces[f].components.iter().any(kept) {
                *slot = face_count;
                face_count += 1;
            }
        }
        let frozen = face_count;
        for &f in faces {
            new_face[f] = face_count;
            face_count += 1;
        }

        let nodes = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(v, _)| new_node[*v] != usize::MAX)
            .map(|(_, node)| {
                let mut spokes = [0; 3];
                for (slot, &h) in spokes.iter_mut().zip(node.rotation()) {
                    *slot = new_dart[h];
                }
                Node {
                    spokes,
                    degree: node.degree,
                    home: if node.degree == 0 {
                        new_face[node.home]
                    } else {
                        0
                    },
                }
            })
            .collect();
        let half_edges = self
            .half_edges
            .iter()
            .enumerate()
            .filter(|(h, _)| new_dart[*h] != usize::MAX)
            .map(|(_, e)| HalfEdge {
                origin: new_node[e.origin],
                face: new_face[e.face],
            })
            .collect();
        let mut faces_out: Vec<Face> = vec![
            Face {
                components: Vec::new()
            };
            face_count
        ];
        for (f, face) in self.faces.iter().enumerate() {
            if new_face[f] == usize::MAX {
                continue;
            }
            faces_out[new_face[f]].components = face
                .components
                .iter()
                .filter(|comp| kept(comp))
                .map(|comp| match *comp {
                    Component::Node(v) => Component::Node(new_node[v]),
                    Component::Cycle(h) => Component::Cycle(new_dart[h]),
                })
                .collect();
        }
        let model = FaceModel {
            nodes,
            half_edges,
            faces: faces_out,
            journal: Vec::new(),
        };
        let form = live_form(&model, frozen);
        Region {
            model,
            frozen,
            form,
        }
    }

    /// The mirror image: every rotation reversed, so each half-edge's
    /// left face is what lay on its right.
    pub fn mirror(&self) -> FaceModel {
        let nodes = self
            .nodes
            .iter()
            .map(|node| {
                let mut spokes = node.spokes;
                spokes[..node.degree as usize].reverse();
                Node { spokes, ..*node }
            })
            .collect();
        let half_edges = self
            .half_edges
            .iter()
            .enumerate()
            .map(|(h, e)| HalfEdge {
                origin: e.origin,
                face: self.half_edges[h ^ 1].face,
            })
            .collect();
        let faces = self
            .faces
            .iter()
            .map(|face| Face {
                components: face
                    .components
                    .iter()
                    .map(|comp| match *comp {
                        Component::Cycle(h) => Component::Cycle(h ^ 1),
                        node => node,
                    })
                    .collect(),
            })
            .collect();
        FaceModel {
            nodes,
            half_edges,
            faces,
            journal: Vec::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::check;
    use super::*;
    use crate::face::canonical_form;
    use crate::face::live::live_form;

    /// The listed move from `a` to `b` enclosing `enclosed`, whose corner
    /// at `a` lies on face `face` when one is named.
    fn find(m: &FaceModel, a: usize, b: usize, enclosed: &[usize], face: Option<FaceId>) -> Move {
        m.moves()
            .into_iter()
            .find(|mv| {
                mv.a == a
                    && mv.b == b
                    && mv.enclosed == enclosed
                    && face.is_none_or(|f| m.corner_face(a, mv.at_a) == Ok(f))
            })
            .unwrap_or_else(|| panic!("move {a} {b} enclosing {enclosed:?} on {face:?}"))
    }

    /// Five nodes: a loop at 0 with node 1 on one side and 2, 3, 4 on the
    /// other, then the chord from 0 to its loop node on node 1's side,
    /// which kills both. Node 1 and the chord's node play on one side; 2,
    /// 3 and 4 play on the other.
    fn two_games() -> FaceModel {
        let mut m = FaceModel::new(5);
        m.apply(&find(&m, 0, 0, &[2, 3, 4], None)).unwrap();
        let inside = m.corner_face(1, None).unwrap();
        m.apply(&find(&m, 0, 5, &[], Some(inside))).unwrap();
        assert_eq!(m.degree(0), 3);
        assert_eq!(m.degree(5), 3);
        m
    }

    fn forms(m: &FaceModel) -> Vec<Vec<u32>> {
        let mut out: Vec<Vec<u32>> = m.regions().iter().map(|r| r.form().to_vec()).collect();
        out.sort();
        out
    }

    fn moves_by_region(m: &FaceModel) -> usize {
        m.regions().iter().map(|r| r.moves().len()).sum()
    }

    #[test]
    fn a_fresh_board_is_one_region_with_the_board_form() {
        let m = FaceModel::new(4);
        let regions = m.regions();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].form(), live_form(&m, 0));
        assert_eq!(regions[0].node_count(), 4);
        assert_eq!(regions[0].lives(), 12);
        assert_eq!(regions[0].moves(), m.search_moves());
    }

    #[test]
    fn faces_sharing_a_live_node_are_one_region() {
        let mut m = FaceModel::new(3);
        m.apply(&find(&m, 0, 0, &[], None)).unwrap();
        assert_eq!(m.face_count(), 2);
        let regions = m.regions();
        assert_eq!(regions.len(), 1);
        assert_eq!(regions[0].face_count(), 2);
        assert_eq!(regions[0].form(), live_form(&m, 0));
    }

    #[test]
    fn killing_the_shared_nodes_splits_the_position_in_two() {
        let m = two_games();
        let regions = m.regions();
        assert_eq!(regions.len(), 2);
        let mut sizes: Vec<(usize, usize, usize)> = regions
            .iter()
            .map(|r| (r.node_count(), r.lives(), r.face_count()))
            .collect();
        sizes.sort_unstable();
        assert_eq!(sizes, vec![(4, 4, 2), (6, 9, 1)]);
        for region in &regions {
            check(region.model());
            assert_eq!(region.model().face_count(), 3);
        }
        assert_eq!(moves_by_region(&m), m.search_moves().len());
    }

    #[test]
    fn a_region_lists_the_moves_of_its_own_faces() {
        let m = two_games();
        let outside = m.corner_face(2, None).unwrap();
        let outer = m
            .regions()
            .into_iter()
            .find(|r| r.node_count() == 6)
            .unwrap();
        let whole = m
            .search_moves()
            .into_iter()
            .filter(|mv| m.corner_face(mv.a, mv.at_a) == Ok(outside))
            .count();
        assert_eq!(outer.moves().len(), whole);
        assert_eq!(whole, 4);
        for mv in outer.moves() {
            assert_eq!(outer.model().corner_face(mv.a, mv.at_a), Ok(2));
        }
    }

    /// Three nodes after four listed moves (by index in `search_moves`),
    /// where a region borrows a piece whose only live node is in the
    /// other region and could be enclosed from the region's own face.
    #[test]
    fn a_borrowed_piece_is_never_enclosed() {
        let mut m = FaceModel::new(3);
        for pick in [0, 0, 9, 13] {
            let mv = m.search_moves()[pick].clone();
            m.apply(&mv).unwrap();
        }
        let mut seen = 0;
        for region in m.regions() {
            let (piece_of_node, _) = pieces(region.model());
            let borrowed: Vec<usize> = (0..region.model().node_count())
                .filter(|&v| !region.plays(v) && region.model().degree(v) < 3)
                .collect();
            let reaches = |mv: &Move| {
                mv.enclosed.iter().any(|&r| {
                    borrowed
                        .iter()
                        .any(|&b| piece_of_node[b] == piece_of_node[r])
                })
            };
            let own =
                |mv: &Move| region.model().corner_face(mv.a, mv.at_a).unwrap() >= region.frozen;
            seen += region
                .model()
                .search_moves()
                .into_iter()
                .filter(|mv| own(mv) && reaches(mv))
                .count();
            assert!(!region.moves().iter().any(reaches));
        }
        assert!(seen > 0);
    }

    #[test]
    fn a_finished_game_leaves_regions_without_moves() {
        let mut m = FaceModel::new(1);
        while let Some(mv) = m.moves().first().cloned() {
            m.apply(&mv).unwrap();
        }
        assert!(m.is_game_over());
        for region in m.regions() {
            assert!(region.moves().is_empty());
        }
        assert_eq!(moves_by_region(&m), 0);
    }

    #[test]
    fn a_move_in_one_region_leaves_the_other_regions_form_alone() {
        let before = two_games();
        let outside = before.corner_face(2, None).unwrap();
        let mut after = before.clone();
        after
            .apply(&find(&after, 2, 3, &[], Some(outside)))
            .unwrap();
        let inner = before
            .regions()
            .into_iter()
            .find(|r| r.node_count() == 4)
            .unwrap();
        let kept: Vec<Vec<u32>> = forms(&before)
            .into_iter()
            .filter(|f| forms(&after).contains(f))
            .collect();
        assert_eq!(kept, vec![inner.form().to_vec()]);
    }

    #[test]
    fn regions_built_in_either_order_share_their_forms() {
        let base = two_games();
        let outside = base.corner_face(2, None).unwrap();
        let inside = base.corner_face(1, None).unwrap();
        let mut first = base.clone();
        first
            .apply(&find(&first, 2, 3, &[], Some(outside)))
            .unwrap();
        first.apply(&find(&first, 1, 1, &[], Some(inside))).unwrap();
        let mut second = base.clone();
        second
            .apply(&find(&second, 1, 1, &[], Some(inside)))
            .unwrap();
        second
            .apply(&find(&second, 2, 3, &[], Some(outside)))
            .unwrap();
        assert_ne!(first, second);
        assert_eq!(forms(&first), forms(&second));
        assert_eq!(forms(&first).len(), 2);
    }

    #[test]
    fn a_renamed_position_gives_the_same_region_forms() {
        let mut m = FaceModel::new(5);
        m.apply(&find(&m, 4, 4, &[1, 2, 3], None)).unwrap();
        let inside = m.corner_face(0, None).unwrap();
        m.apply(&find(&m, 4, 5, &[], Some(inside))).unwrap();
        assert_eq!(forms(&m), forms(&two_games()));
    }

    #[test]
    fn the_regions_a_move_leaves_are_those_of_the_position_after_it() {
        let m = two_games();
        let outside = m.corner_face(2, None).unwrap();
        let regions = m.regions();
        let outer = regions.iter().find(|r| r.node_count() == 6).unwrap();
        let inner = regions.iter().find(|r| r.node_count() == 4).unwrap();
        let mut by_region: Vec<Vec<Vec<u32>>> = outer
            .moves()
            .iter()
            .map(|mv| {
                let mut left: Vec<Vec<u32>> =
                    outer.after(mv).iter().map(|r| r.form().to_vec()).collect();
                left.push(inner.form().to_vec());
                left.sort();
                left
            })
            .collect();
        let mut by_whole: Vec<Vec<Vec<u32>>> = m
            .search_moves()
            .into_iter()
            .filter(|mv| m.corner_face(mv.a, mv.at_a) == Ok(outside))
            .map(|mv| {
                let mut after = m.clone();
                after.apply(&mv).unwrap();
                forms(&after)
            })
            .collect();
        by_region.sort();
        by_whole.sort();
        assert_eq!(by_region, by_whole);
    }

    /// A four-cycle of dead nodes with a chord on one side and two
    /// pendants on the other is one piece. Its two sides are different
    /// games, so the forms differ even though the models match.
    #[test]
    fn two_sides_of_one_piece_get_different_forms() {
        let mut m = FaceModel::new(4);
        m.apply(&find(&m, 0, 1, &[], None)).unwrap();
        m.apply(&find(&m, 0, 1, &[2, 3], None)).unwrap();
        let empty = (0..m.face_count())
            .find(|&f| m.components(f).len() == 1)
            .unwrap();
        m.apply(&find(&m, 0, 1, &[], Some(empty))).unwrap();
        let busy = m.corner_face(2, None).unwrap();
        m.apply(&find(&m, 2, 4, &[], Some(busy))).unwrap();
        m.apply(&find(&m, 3, 5, &[], Some(busy))).unwrap();
        let regions = m.regions();
        assert_eq!(regions.len(), 2);
        assert_eq!(
            canonical_form(regions[0].model()),
            canonical_form(regions[1].model())
        );
        assert_ne!(regions[0].form(), regions[1].form());
        let mut moves: Vec<usize> = regions.iter().map(|r| r.moves().len()).collect();
        moves.sort_unstable();
        assert_eq!(moves[0], 0);
        assert!(moves[1] > 0);
    }

    #[test]
    fn the_mirror_is_a_position_and_mirrors_back() {
        let mut m = FaceModel::new(4);
        for (a, b) in [(0, 1), (0, 2), (1, 4)] {
            let mv = m
                .moves()
                .into_iter()
                .find(|mv| mv.a == a && mv.b == b)
                .unwrap();
            m.apply(&mv).unwrap();
        }
        let mirror = m.mirror();
        check(&mirror);
        assert_eq!(mirror.node_count(), m.node_count());
        assert_eq!(mirror.face_count(), m.face_count());
        assert_eq!(mirror.moves().len(), m.moves().len());
        assert_eq!(canonical_form(&mirror.mirror()), canonical_form(&m));
    }

    /// A node whose three spokes lead to different things is chiral: its
    /// mirror image is another rotation system with the same game.
    #[test]
    fn a_chiral_region_and_its_mirror_share_a_form() {
        let mut m = FaceModel::new(3);
        m.apply(&find(&m, 0, 1, &[], None)).unwrap();
        m.apply(&find(&m, 0, 2, &[], None)).unwrap();
        let chiral = m
            .moves()
            .into_iter()
            .filter(|mv| mv.a == 0 || mv.b == 0)
            .map(|mv| {
                let mut probe = m.clone();
                probe.apply(&mv).unwrap();
                probe
            })
            .find(|probe| canonical_form(probe) != canonical_form(&probe.mirror()))
            .expect("a chiral completion of node 0");
        assert_eq!(chiral.degree(0), 3);
        assert_eq!(forms(&chiral), forms(&chiral.mirror()));
    }
}
