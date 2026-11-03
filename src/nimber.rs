//! Nimbers of regions, so a position is played as the sum of its regions.
//!
//! Under normal play the player to move loses exactly when the nim-sum of
//! the regions' nimbers is zero. A region's nimber is the mex over the
//! nim-sums its moves leave and needs every move looked at, so a region
//! standing alone is only asked whether it is won, which stops at the
//! first winning move, and nimbers are found for the parts of a split.
//! Both depend only on the form and are kept for the life of the worker.

use std::collections::{HashMap, HashSet};

use crate::face::{FaceModel, Region};

/// Child positions one turn may form while solving before it gives up
/// and the time-boxed search takes over; about a quarter second.
pub const NIMBER_BUDGET: usize = 10_000;
/// Most lives in a region the exact choice is tried on. Six fresh nodes
/// have 18 and take 800,000 child positions from cold.
pub const EXACT_LIVES: usize = 16;
/// Regions kept before the table starts over.
pub const TABLE_CAP: usize = 200_000;

#[derive(Default)]
pub struct Nimbers {
    table: HashMap<Vec<u32>, u32>,
    /// Regions known won by the player to move without their nimber; a
    /// lost one has nimber zero and goes in `table`.
    won: HashSet<Vec<u32>>,
    hits: usize,
    misses: usize,
    largest: (usize, usize),
}

impl Nimbers {
    pub fn new() -> Self {
        Self::default()
    }

    /// Regions with a nimber or an outcome stored.
    #[cfg(any(test, feature = "bench"))]
    pub fn len(&self) -> usize {
        self.table.len() + self.won.len()
    }

    #[cfg(test)]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    #[cfg(any(test, feature = "bench"))]
    pub fn hits(&self) -> usize {
        self.hits
    }

    #[cfg(any(test, feature = "bench"))]
    pub fn misses(&self) -> usize {
        self.misses
    }

    /// Lives and node count of the biggest region settled so far.
    #[cfg(any(test, feature = "bench"))]
    pub fn largest(&self) -> (usize, usize) {
        self.largest
    }

    /// The stored nimber of `region`, without solving.
    pub fn get(&mut self, region: &Region) -> Option<u32> {
        let found = self.table.get(region.form()).copied();
        if found.is_some() {
            self.hits += 1;
        } else {
            self.misses += 1;
        }
        found
    }

    /// Whether the player to move loses the sum of the regions with
    /// `forms`, from what is stored: a lone region's outcome, or every
    /// region's nimber.
    pub fn known_lost(&mut self, forms: &[&[u32]]) -> Option<bool> {
        if let [one] = forms {
            if self.won.contains(*one) {
                self.hits += 1;
                return Some(false);
            }
        }
        let mut sum = 0;
        for form in forms {
            match self.table.get(*form) {
                Some(value) => {
                    self.hits += 1;
                    sum ^= value;
                }
                None => {
                    self.misses += 1;
                    return None;
                }
            }
        }
        Some(sum == 0)
    }

    /// Whether the player to move loses the sum `regions`, solving what
    /// is missing within `budget` child positions.
    pub fn lost(&mut self, regions: &[Region], budget: &mut usize) -> Option<bool> {
        match regions {
            [] => Some(true),
            [one] => Some(!self.wins(one, budget)?),
            many => {
                let sum = many
                    .iter()
                    .try_fold(0, |acc, region| Some(acc ^ self.solve(region, budget)?))?;
                Some(sum == 0)
            }
        }
    }

    /// `lost` over the regions of `model`.
    pub fn lost_in(&mut self, model: &FaceModel, budget: &mut usize) -> Option<bool> {
        self.lost(&model.regions(), budget)
    }

    /// Whether the player to move wins `region` on its own: some move
    /// leaves a lost sum. Stops at the first.
    pub fn wins(&mut self, region: &Region, budget: &mut usize) -> Option<bool> {
        if let Some(value) = self.get(region) {
            return Some(value != 0);
        }
        if self.won.contains(region.form()) {
            return Some(true);
        }
        for mv in region.moves() {
            if *budget == 0 {
                return None;
            }
            *budget -= 1;
            if self.lost(&region.after(&mv), budget)? {
                self.grew(region);
                self.won.insert(region.form().to_vec());
                return Some(true);
            }
        }
        self.insert(region, 0);
        Some(false)
    }

    /// The nimber of `region`, solved when not stored: the mex over the
    /// nim-sums of the regions each listed move leaves. Each child
    /// position formed costs one of `budget`; at zero the solve stops
    /// with nothing stored for the regions still open.
    pub fn solve(&mut self, region: &Region, budget: &mut usize) -> Option<u32> {
        if let Some(value) = self.get(region) {
            return Some(value);
        }
        let mut seen: u64 = 0;
        for mv in region.moves() {
            if *budget == 0 {
                return None;
            }
            *budget -= 1;
            let mut total = 0;
            for child in &region.after(&mv) {
                total ^= self.solve(child, budget)?;
            }
            assert!(total < 64, "nimber {total}");
            seen |= 1 << total;
        }
        let value = seen.trailing_ones();
        self.insert(region, value);
        Some(value)
    }

    /// Nim-sum of the regions of `model`, solving each within `budget`.
    #[cfg(test)]
    pub fn solve_all(&mut self, model: &FaceModel, budget: &mut usize) -> Option<u32> {
        model
            .regions()
            .iter()
            .try_fold(0, |acc, region| Some(acc ^ self.solve(region, budget)?))
    }

    fn insert(&mut self, region: &Region, value: u32) {
        self.grew(region);
        self.table.insert(region.form().to_vec(), value);
    }

    /// Notes the region's size and makes room for it.
    fn grew(&mut self, region: &Region) {
        let size = (region.lives(), region.node_count());
        if size > self.largest {
            self.largest = size;
        }
        if self.table.len() + self.won.len() >= TABLE_CAP {
            self.table.clear();
            self.won.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn fresh(n: usize) -> Region {
        FaceModel::new(n).regions().remove(0)
    }

    /// More child positions than any test here forms.
    fn plenty() -> usize {
        1_000_000
    }

    fn solved(n: usize) -> u32 {
        Nimbers::new().solve(&fresh(n), &mut plenty()).unwrap()
    }

    /// Whether the player to move wins, by searching every listed move.
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

    #[test]
    fn one_and_two_lone_nodes_are_lost_for_the_player_to_move() {
        assert_eq!(solved(1), 0);
        assert_eq!(solved(2), 0);
    }

    #[test]
    fn the_three_node_openings_sum_to_the_three_node_nimber() {
        let mut nimbers = Nimbers::new();
        let root = fresh(3);
        let value = nimbers.solve(&root, &mut plenty()).unwrap();
        assert_ne!(value, 0);
        let mut model = root.model().clone();
        let mut children = Vec::new();
        for mv in model.moves() {
            model.apply(&mv).unwrap();
            children.push(nimbers.solve_all(&model, &mut plenty()).unwrap());
            model.undo();
        }
        assert!(children.contains(&0));
        assert!(!children.contains(&value));
        let mex = (0..).find(|v| !children.contains(v)).unwrap();
        assert_eq!(mex, value);
    }

    /// Applegate, Jacobson and Sleator: the first player loses from 1 and
    /// 2 nodes and wins from 3 and 4.
    #[test]
    fn the_known_outcomes_up_to_four_nodes() {
        let mut nimbers = Nimbers::new();
        let outcomes: Vec<bool> = (1..=4)
            .map(|n| nimbers.wins(&fresh(n), &mut 100_000).unwrap())
            .collect();
        assert_eq!(outcomes, [false, false, true, true]);
        let by_nimber: Vec<bool> = (1..=4)
            .map(|n| nimbers.solve(&fresh(n), &mut 100_000).unwrap() != 0)
            .collect();
        assert_eq!(by_nimber, outcomes);
    }

    /// A won region is remembered as won, a lost one as nimber zero.
    #[test]
    fn outcomes_are_stored_and_a_lost_region_has_nimber_zero() {
        let mut nimbers = Nimbers::new();
        assert_eq!(nimbers.wins(&fresh(3), &mut 100_000), Some(true));
        assert_eq!(nimbers.known_lost(&[fresh(3).form()]), Some(false));
        assert_eq!(nimbers.get(&fresh(3)), None);
        assert_eq!(nimbers.wins(&fresh(2), &mut 100_000), Some(false));
        assert_eq!(nimbers.get(&fresh(2)), Some(0));
        assert_eq!(nimbers.known_lost(&[fresh(2).form()]), Some(true));
        assert_eq!(
            nimbers.known_lost(&[fresh(2).form(), fresh(3).form()]),
            None
        );
    }

    /// The first player wins from 5 nodes and loses from 6. Run with
    /// `cargo test --release nimber::tests::the_known_outcomes_at_five
    /// -- --ignored`.
    #[test]
    #[ignore = "solves five and six nodes, run in release"]
    fn the_known_outcomes_at_five_and_six_nodes() {
        let mut nimbers = Nimbers::new();
        let mut budget = 5_000_000;
        assert_eq!(nimbers.wins(&fresh(5), &mut budget), Some(true));
        assert_eq!(nimbers.wins(&fresh(6), &mut budget), Some(false));
    }

    #[test]
    fn a_solved_region_is_found_again_from_another_build_and_its_mirror() {
        let mut nimbers = Nimbers::new();
        let mut a = FaceModel::new(4);
        let chord = a
            .moves()
            .into_iter()
            .find(|mv| mv.a == 0 && mv.b == 1)
            .unwrap();
        a.apply(&chord).unwrap();
        let value = nimbers.solve_all(&a, &mut plenty()).unwrap();
        let stored = nimbers.len();
        let mut b = FaceModel::new(4);
        let other = b
            .moves()
            .into_iter()
            .find(|mv| mv.a == 2 && mv.b == 3)
            .unwrap();
        b.apply(&other).unwrap();
        let hits = nimbers.hits();
        assert_eq!(nimbers.solve_all(&b, &mut plenty()), Some(value));
        assert_eq!(nimbers.hits(), hits + 1);
        assert_eq!(nimbers.solve_all(&b.mirror(), &mut plenty()), Some(value));
        assert_eq!(nimbers.len(), stored);
    }

    #[test]
    fn a_spent_budget_gives_nothing_and_stores_only_finished_regions() {
        let mut nimbers = Nimbers::new();
        let mut budget = 0;
        assert_eq!(nimbers.solve(&fresh(3), &mut budget), None);
        assert!(nimbers.is_empty());
        let mut budget = 40;
        assert_eq!(nimbers.solve(&fresh(4), &mut budget), None);
        assert_eq!(budget, 0);
        assert!(!nimbers.is_empty());
        for (form, value) in &nimbers.table {
            assert!(form.len() > 1);
            assert!(*value < 8);
        }
        assert_eq!(nimbers.solve(&fresh(4), &mut plenty()), Some(solved(4)));
    }

    #[test]
    fn the_largest_region_is_the_root_it_was_asked_for() {
        let mut nimbers = Nimbers::new();
        nimbers.solve(&fresh(4), &mut 100_000).unwrap();
        assert_eq!(nimbers.largest(), (12, 4));
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(64))]

        /// The nim-sum says who wins exactly when the whole-board search
        /// does, on positions with and without several regions.
        #[test]
        fn the_nim_sum_agrees_with_the_whole_board_search(
            n in 1usize..=4,
            picks in prop::collection::vec(any::<u32>(), 10),
        ) {
            let mut nimbers = Nimbers::new();
            let mut m = FaceModel::new(n);
            let skip = if n == 4 { 3 } else { 0 };
            for (k, &pick) in picks.iter().enumerate() {
                let moves = m.moves();
                if moves.is_empty() {
                    break;
                }
                if k >= skip {
                    let total = nimbers.solve_all(&m, &mut plenty()).unwrap();
                    prop_assert_eq!(total != 0, first_player_wins(&mut m.clone()), "{:?}", m);
                }
                m.apply(&moves[pick as usize % moves.len()]).unwrap();
            }
        }
    }

    /// Prints what each node count costs to solve from scratch. Run with
    /// `cargo test --release nimber::tests::solving_cost -- --ignored
    /// --nocapture`.
    #[test]
    #[ignore = "prints timings"]
    fn solving_cost() {
        for n in 1..=6 {
            let cap = if n < 5 { 200_000 } else { 1_000_000 };
            let mut nimbers = Nimbers::new();
            let mut budget = cap;
            let started = std::time::Instant::now();
            let won = nimbers.wins(&fresh(n), &mut budget);
            println!(
                "{n} nodes: won {won:?}, {} children, {} forms, {:.1} ms",
                cap - budget,
                nimbers.len(),
                started.elapsed().as_secs_f64() * 1000.0
            );
            if n > 4 {
                continue;
            }
            let mut nimbers = Nimbers::new();
            let mut budget = cap;
            let started = std::time::Instant::now();
            let value = nimbers.solve(&fresh(n), &mut budget);
            println!(
                "{n} nodes: nimber {value:?}, {} children, {} forms, {:.1} ms",
                cap - budget,
                nimbers.len(),
                started.elapsed().as_secs_f64() * 1000.0
            );
        }
    }

    /// Four nodes, a loop round two of them and the chord across it on
    /// the other side: two regions, whose nim-sum says what the search
    /// over the whole board says.
    #[test]
    fn a_split_position_is_solved_from_its_parts() {
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
        let regions = m.regions();
        assert_eq!(regions.len(), 2);
        let mut nimbers = Nimbers::new();
        let values: Vec<u32> = regions
            .iter()
            .map(|r| nimbers.solve(r, &mut plenty()).unwrap())
            .collect();
        assert_ne!(values[0], values[1]);
        let sum = values[0] ^ values[1];
        assert_eq!(sum != 0, first_player_wins(&mut m.clone()));
        assert_eq!(nimbers.lost(&regions, &mut plenty()), Some(sum == 0));
    }

    /// Four fresh nodes are not reached from three, so settling them on a
    /// full table starts it over and leaves about what that alone stores,
    /// less the few small regions hit before the table went.
    #[test]
    fn the_table_starts_over_past_its_cap() {
        let mut alone = Nimbers::new();
        assert_eq!(alone.wins(&fresh(4), &mut plenty()), Some(true));
        let mut nimbers = Nimbers::new();
        nimbers.wins(&fresh(3), &mut plenty()).unwrap();
        assert_eq!(nimbers.get(&fresh(4)), None);
        for form in nimbers.len()..TABLE_CAP {
            nimbers.table.insert(vec![u32::MAX, form as u32], 0);
        }
        assert_eq!(nimbers.len(), TABLE_CAP);
        assert_eq!(nimbers.wins(&fresh(4), &mut plenty()), Some(true));
        assert!(nimbers.len() <= alone.len());
        assert!(nimbers.len() > alone.len() / 2);
        assert_eq!(nimbers.known_lost(&[fresh(4).form()]), Some(false));
    }
}
