use std::hash::Hash;

pub trait State: Hash + Clone + Eq + Ord {}

pub trait StateSpace<'a, S: 'a> where S: State {
    type StateSpaceIterator: Iterator<Item=&'a S>;
    fn iter_states(&self) -> Self::StateSpaceIterator;
    fn iter_nonterminal_states(&self) -> Self::StateSpaceIterator;
}

pub trait TransitionTable<'a, 's, S: 's, A: 'a> where S: State, A: Action {
    type TransitionsIterator: Iterator<Item=(&'a A, &'s S)>;
    fn list_transitions(&self, state: &S) -> Self::TransitionsIterator;
}

pub trait Action: Hash + Clone + Eq + Ord {}

pub trait ActionSpace<A> where A: Action {}

pub trait TerminalTable<S> where S: State {
    fn terminal(&self, state: &S) -> bool;
}

pub trait RewardTable<S, A> where S: State, A: Action {
    fn reward(&self, start: &S, end: &S, action: &A) -> f32;
}