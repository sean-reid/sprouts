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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
/// loop surrounds. A loop on an isolated node has two equivalent sides;
/// `apply` takes either, `moves` lists one.
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct HalfEdge {
    origin: NodeId,
    face: FaceId,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
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

/// Where a checked move goes: its face, the component index of each
/// corner, and whether the two corners share a component.
struct Placement {
    face: FaceId,
    bounds: Boundaries,
    ca: usize,
    cb: usize,
    split: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FaceModel {
    nodes: Vec<Node>,
    half_edges: Vec<HalfEdge>,
    faces: Vec<Face>,
    journal: Vec<Undo>,
}

/// Hashes the position and not the journal, so two models that agree on
/// every node, half-edge and face hash alike however they were reached.
impl std::hash::Hash for FaceModel {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.nodes.hash(state);
        self.half_edges.hash(state);
        self.faces.hash(state);
    }
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

    /// The half-edge running the other way along the same edge.
    pub fn twin(&self, h: HalfEdgeId) -> HalfEdgeId {
        h ^ 1
    }

    /// Face on the left of `h`.
    pub fn face_of(&self, h: HalfEdgeId) -> FaceId {
        self.half_edges[h].face
    }

    pub fn components(&self, f: FaceId) -> &[Component] {
        &self.faces[f].components
    }

    /// The boundary piece holding the corner at `v` that follows `at`.
    pub fn component_of(
        &self,
        v: NodeId,
        at: Option<HalfEdgeId>,
    ) -> Result<Component, IllegalMove> {
        let face = self.corner_face(v, at)?;
        let Some(h) = at else {
            return Ok(Component::Node(v));
        };
        self.faces[face]
            .components
            .iter()
            .copied()
            .find(|comp| matches!(comp, Component::Cycle(start) if self.cycle(*start).contains(&h)))
            .ok_or(IllegalMove::Corner)
    }

    /// Lowest node on a piece, which is how `Move::enclosed` names it.
    pub fn lowest_node(&self, comp: Component) -> NodeId {
        match comp {
            Component::Node(v) => v,
            Component::Cycle(start) => self
                .cycle(start)
                .into_iter()
                .map(|h| self.half_edges[h].origin)
                .min()
                .expect("a cycle has a half-edge"),
        }
    }

    /// Half-edges of the closed boundary walk through `start`, in walk order.
    pub fn cycle(&self, start: HalfEdgeId) -> Vec<HalfEdgeId> {
        let mut out = vec![start];
        let mut h = self.next(start);
        while h != start {
            out.push(h);
            h = self.next(h);
        }
        out
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
        !(0..self.nodes.len()).any(|v| self.can_move(v))
    }

    /// Nodes with a legal move, ascending.
    pub fn active_nodes(&self) -> Vec<NodeId> {
        (0..self.nodes.len())
            .filter(|&v| self.can_move(v))
            .collect()
    }

    /// Whether `v` has a legal move. A node with two lives can always loop
    /// in its own face; one with a single life needs a partner with a life
    /// on the boundary of one of its faces. O(E) at worst.
    pub fn can_move(&self, v: NodeId) -> bool {
        match self.nodes[v].degree {
            0 | 1 => true,
            2 => self.nodes[v]
                .rotation()
                .iter()
                .any(|&h| self.has_partner(self.half_edges[h].face, v)),
            _ => false,
        }
    }

    fn has_partner(&self, face: FaceId, v: NodeId) -> bool {
        self.half_edges
            .iter()
            .any(|e| e.face == face && e.origin != v && self.nodes[e.origin].degree < 3)
            || self
                .nodes
                .iter()
                .enumerate()
                .any(|(u, node)| u != v && node.degree == 0 && node.home == face)
    }

    /// O(E): walks every face to validate `enclosed`.
    fn place(&self, mv: &Move) -> Result<Placement, IllegalMove> {
        let (a, b) = (mv.a, mv.b);
        let face = self.corner_face(a, mv.at_a)?;
        if self.corner_face(b, mv.at_b)? != face {
            return Err(IllegalMove::Face);
        }
        let need = if a == b { 2 } else { 1 };
        if self.nodes[a].degree + need > 3 || self.nodes[b].degree + need > 3 {
            return Err(IllegalMove::Degree);
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
        Ok(Placement {
            face,
            bounds,
            ca,
            cb,
            split,
        })
    }

    /// Whether `apply` would accept `mv`, at the same cost.
    pub fn check(&self, mv: &Move) -> Result<(), IllegalMove> {
        self.place(mv).map(drop)
    }

    /// O(E): walks every face to validate `enclosed` and to split.
    pub fn apply(&mut self, mv: &Move) -> Result<(), IllegalMove> {
        let (a, b) = (mv.a, mv.b);
        let Placement {
            face,
            bounds,
            ca,
            cb,
            split,
        } = self.place(mv)?;
        let reps = &bounds.reps[face];

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

mod canonical;
mod live;
mod moves;
mod region;

pub use canonical::canonical_form;
pub use live::{live_form, parts, LiveScratch};
pub use region::Region;

#[cfg(test)]
mod tests;
