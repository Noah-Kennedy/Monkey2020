use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::hash::Hash;
use std::marker::PhantomData;
use std::option::Option::Some;

use crate::heap::PairingHeap;
use fasthash::RandomState;

pub trait StateSpace<S> {
    fn neighbors(&self, state: &S) -> Vec<(f32, S)>;
    fn heuristic(&self, state: &S, goal: &S) -> f32;
}

struct Task<S> {
    priority: f32,
    state: S,
}

impl<S> PartialEq for Task<S> {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl<S> PartialOrd for Task<S> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.priority.partial_cmp(&other.priority)
    }
}

type Hasher = fasthash::sea::Hash64;

pub struct AStar<G, S> {
    space: G,
    expandable: HashSet<S, RandomState<Hasher>>,
    actions: HashMap<S, S, RandomState<Hasher>>,
    current_cost: HashMap<S, f32, RandomState<Hasher>>,
    _phantom: PhantomData<S>,
}

impl<G, S> AStar<G, S> {
    pub fn new(space: G) -> Self {
        Self {
            space,
            expandable: HashSet::with_hasher(RandomState::<Hasher>::new()),
            actions: HashMap::with_hasher(RandomState::<Hasher>::new()),
            current_cost: HashMap::with_hasher(RandomState::<Hasher>::new()),
            _phantom: PhantomData::default(),
        }
    }
}

impl<G, S> AStar<G, S> where G: StateSpace<S>, S: Hash + Eq + Clone {
    #[inline(never)]
    pub fn find_path(&mut self, start: &S, end: &S) -> Option<Vec<S>> {
        self.actions.clear();
        self.current_cost.clear();
        self.expandable.clear();

        self.current_cost.insert(start.clone(), 0.0);

        let mut queue = PairingHeap::new();

        self.expandable.insert(start.clone());

        let st = Task { priority: self.space.heuristic(start, end), state: start.clone() };

        queue.insert(st);

        while let Some(Task { priority: _, state: current }) = queue.delete_min() {
            self.expandable.remove(&current);

            if &current == end {
                return Some(self.reconstruct_path(end));
            } else {
                for (cost, neighbor) in self.space.neighbors(&current) {
                    let tentative_cost = self.current_cost[&current] + cost;

                    if tentative_cost < *self.current_cost
                        .get(&neighbor)
                        .unwrap_or(&f32::INFINITY)
                    {
                        self.actions.insert(neighbor.clone(), current.clone());
                        self.current_cost.insert(neighbor.clone(), tentative_cost);

                        let tc = tentative_cost + self.space.heuristic(&neighbor, end);

                        if !self.expandable.contains(&neighbor) {
                            self.expandable.insert(neighbor.clone());
                            let t = Task { priority: tc, state: neighbor };
                            queue.insert(t);
                        }
                    }
                }
            }
        }

        None
    }

    #[inline(never)]
    fn reconstruct_path(&self, end: &S) -> Vec<S> {
        let mut total_path = Vec::new();

        let mut current = end.clone();

        while let Some(previous) = self.actions.get(&current) {
            total_path.push(current.clone());
            current = previous.clone();
        }

        total_path.reverse();

        total_path
    }
}