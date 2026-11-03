//! Exact planar model of a Sprouts position with no geometry in it.
//!
//! The position is a rotation system: every node lists its half-edges
//! counterclockwise, and every face lies on the left of its boundary walk.
//! A Sprouts line is two edges meeting at the new node. Positions live on
//! the sphere, which is the same game as the plane.

pub type NodeId = usize;
pub type HalfEdgeId = usize;
pub type FaceId = usize;

/// One boundary piece of a face: a closed walk, or a node without edges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Component {
    Cycle(HalfEdgeId),
    Node(NodeId),
}

/// A line from `a` to `b` through a new node.
///
/// `at_a` is the half-edge out of `a` that the new edge follows in the
/// rotation, `None` when `a` has no edges. When `a` and `b` sit on the same
/// boundary component the face splits, and `enclosed` lists the lowest node
/// of each other component that ends up on the left of the path `a`, `m`,
/// `b`. A self-loop sets `at_b` equal to `at_a` and `enclosed` is what the
/// loop surrounds.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Move {
    pub a: NodeId,
    pub at_a: Option<HalfEdgeId>,
    pub b: NodeId,
    pub at_b: Option<HalfEdgeId>,
    pub enclosed: Vec<NodeId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IllegalMove {
    Node,
    Degree,
    Corner,
    Face,
    Enclosed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Node {
    spokes: [HalfEdgeId; 3],
    degree: u8,
    /// Face holding the node while it has no edges.
    home: FaceId,
}

impl Node {
    fn new(home: FaceId) -> Self {
        Self {
            spokes: [0; 3],
            degree: 0,
            home,
        }
    }

    fn rotation(&self) -> &[HalfEdgeId] {
        &self.spokes[..self.degree as usize]
    }

    fn position(&self, h: HalfEdgeId) -> usize {
        self.rotation()
            .iter()
            .position(|&s| s == h)
            .expect("half-edge in rotation")
    }

    fn pred(&self, h: HalfEdgeId) -> HalfEdgeId {
        let d = self.degree as usize;
        self.spokes[(self.position(h) + d - 1) % d]
    }

    fn insert_after(&mut self, after: Option<HalfEdgeId>, h: HalfEdgeId) {
        let at = after.map_or(0, |a| self.position(a) + 1);
        let d = self.degree as usize;
        self.spokes.copy_within(at..d, at + 1);
        self.spokes[at] = h;
        self.degree += 1;
    }

    fn remove(&mut self, h: HalfEdgeId) {
        let at = self.position(h);
        let d = self.degree as usize;
        self.spokes.copy_within(at + 1..d, at);
        self.spokes[d - 1] = 0;
        self.degree -= 1;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct HalfEdge {
    origin: NodeId,
    face: FaceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Face {
    components: Vec<Component>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Undo {
    mv: Move,
    face: FaceId,
    components: Vec<Component>,
    split: bool,
}

/// Per-face component index of every half-edge and isolated node, plus the
/// lowest node on each component, in component order.
struct Boundaries {
    of_half_edge: Vec<usize>,
    of_node: Vec<usize>,
    reps: Vec<Vec<NodeId>>,
}

impl Boundaries {
    fn component(&self, v: NodeId, at: Option<HalfEdgeId>) -> usize {
        match at {
            Some(h) => self.of_half_edge[h],
            None => self.of_node[v],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FaceModel {
    nodes: Vec<Node>,
    half_edges: Vec<HalfEdge>,
    faces: Vec<Face>,
    journal: Vec<Undo>,
}

impl FaceModel {
    /// `n` isolated nodes in the outer face.
    pub fn new(n: usize) -> Self {
        Self {
            nodes: vec![Node::new(0); n],
            half_edges: Vec::new(),
            faces: vec![Face {
                components: (0..n).map(Component::Node).collect(),
            }],
            journal: Vec::new(),
        }
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn edge_count(&self) -> usize {
        self.half_edges.len() / 2
    }

    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    pub fn degree(&self, v: NodeId) -> u8 {
        self.nodes[v].degree
    }

    /// Half-edges out of `v`, counterclockwise.
    pub fn rotation(&self, v: NodeId) -> &[HalfEdgeId] {
        self.nodes[v].rotation()
    }

    pub fn origin(&self, h: HalfEdgeId) -> NodeId {
        self.half_edges[h].origin
    }

    pub fn head(&self, h: HalfEdgeId) -> NodeId {
        self.half_edges[h ^ 1].origin
    }

    /// Face on the left of `h`.
    pub fn face_of(&self, h: HalfEdgeId) -> FaceId {
        self.half_edges[h].face
    }

    pub fn components(&self, f: FaceId) -> &[Component] {
        &self.faces[f].components
    }

    /// Face of the corner at `v` that follows `at` counterclockwise.
    pub fn corner_face(&self, v: NodeId, at: Option<HalfEdgeId>) -> Result<FaceId, IllegalMove> {
        let node = self.nodes.get(v).ok_or(IllegalMove::Node)?;
        match at {
            None if node.degree == 0 => Ok(node.home),
            Some(h) if node.rotation().contains(&h) => Ok(self.half_edges[h].face),
            _ => Err(IllegalMove::Corner),
        }
    }

    pub fn is_game_over(&self) -> bool {
        self.moves().is_empty()
    }

    /// Every legal move, each face-side choice listed separately, in a
    /// fixed order.
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

    pub fn apply(&mut self, mv: &Move) -> Result<(), IllegalMove> {
        let (a, b) = (mv.a, mv.b);
        let face = self.corner_face(a, mv.at_a)?;
        if self.corner_face(b, mv.at_b)? != face {
            return Err(IllegalMove::Face);
        }
        let need = if a == b { 2 } else { 1 };
        if self.nodes[a].degree + need > 3 || self.nodes[b].degree + need > 3 {
            return Err(IllegalMove::Degree);
        }
        if a == b && mv.at_a != mv.at_b {
            return Err(IllegalMove::Corner);
        }
        let bounds = self.boundaries();
        let ca = bounds.component(a, mv.at_a);
        let cb = bounds.component(b, mv.at_b);
        let split = ca == cb;
        let reps = &bounds.reps[face];
        if !split && !mv.enclosed.is_empty() {
            return Err(IllegalMove::Enclosed);
        }
        for (i, r) in mv.enclosed.iter().enumerate() {
            let known = reps.iter().position(|x| x == r).is_some_and(|k| k != ca);
            if !known || mv.enclosed[..i].contains(r) {
                return Err(IllegalMove::Enclosed);
            }
        }

        let m = self.nodes.len();
        let h_am = self.half_edges.len();
        let (h_ma, h_mb, h_bm) = (h_am + 1, h_am + 2, h_am + 3);
        for origin in [a, m, m, b] {
            self.half_edges.push(HalfEdge { origin, face });
        }
        let mut mid = Node::new(face);
        mid.spokes[..2].copy_from_slice(&[h_ma, h_mb]);
        mid.degree = 2;
        self.nodes.push(mid);
        self.nodes[a].insert_after(mv.at_a, h_am);
        let at_b = if a == b { Some(h_am) } else { mv.at_b };
        self.nodes[b].insert_after(at_b, h_bm);

        let old = std::mem::take(&mut self.faces[face].components);
        if split {
            let new_face = self.faces.len();
            self.faces.push(Face {
                components: vec![Component::Cycle(h_am)],
            });
            self.move_component(Component::Cycle(h_am), new_face);
            self.faces[face].components.push(Component::Cycle(h_bm));
            for (i, comp) in old.iter().enumerate() {
                if i == ca {
                    continue;
                }
                if mv.enclosed.contains(&reps[i]) {
                    self.move_component(*comp, new_face);
                    self.faces[new_face].components.push(*comp);
                } else {
                    self.faces[face].components.push(*comp);
                }
            }
        } else {
            self.faces[face].components.push(Component::Cycle(h_am));
            for (i, comp) in old.iter().enumerate() {
                if i != ca && i != cb {
                    self.faces[face].components.push(*comp);
                }
            }
        }
        self.journal.push(Undo {
            mv: mv.clone(),
            face,
            components: old,
            split,
        });
        Ok(())
    }

    /// Reverts the last applied move and returns it.
    pub fn undo(&mut self) -> Option<Move> {
        let Undo {
            mv,
            face,
            components,
            split,
        } = self.journal.pop()?;
        if split {
            let gone = self.faces.pop().expect("split face");
            for comp in gone.components {
                self.move_component(comp, face);
            }
        }
        self.faces[face].components = components;
        let h_am = self.half_edges.len() - 4;
        self.nodes[mv.a].remove(h_am);
        self.nodes[mv.b].remove(h_am + 3);
        self.half_edges.truncate(h_am);
        self.nodes.pop();
        Some(mv)
    }

    fn next(&self, h: HalfEdgeId) -> HalfEdgeId {
        self.nodes[self.head(h)].pred(h ^ 1)
    }

    fn cycle(&self, start: HalfEdgeId) -> Vec<HalfEdgeId> {
        let mut out = vec![start];
        let mut h = self.next(start);
        while h != start {
            out.push(h);
            h = self.next(h);
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

    fn move_component(&mut self, comp: Component, to: FaceId) {
        match comp {
            Component::Node(v) => self.nodes[v].home = to,
            Component::Cycle(start) => {
                for h in self.cycle(start) {
                    self.half_edges[h].face = to;
                }
            }
        }
    }

    fn boundaries(&self) -> Boundaries {
        let mut of_half_edge = vec![usize::MAX; self.half_edges.len()];
        let mut of_node = vec![usize::MAX; self.nodes.len()];
        let mut reps = Vec::with_capacity(self.faces.len());
        for face in &self.faces {
            let mut face_reps = Vec::with_capacity(face.components.len());
            for (i, comp) in face.components.iter().enumerate() {
                match *comp {
                    Component::Node(v) => {
                        of_node[v] = i;
                        face_reps.push(v);
                    }
                    Component::Cycle(start) => {
                        let mut lowest = usize::MAX;
                        for h in self.cycle(start) {
                            of_half_edge[h] = i;
                            lowest = lowest.min(self.half_edges[h].origin);
                        }
                        face_reps.push(lowest);
                    }
                }
            }
            reps.push(face_reps);
        }
        Boundaries {
            of_half_edge,
            of_node,
            reps,
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

#[cfg(test)]
mod tests;
