use std::sync::Arc;

pub trait StateSpace<S, A> {
    fn nonterminal_states(&self) -> Arc<Vec<S>>;
    fn terminal(&self, state: &S) -> bool;
    fn actions(&self, state: &S) -> Vec<A>;
    fn q_states(&self, state: &S, action: &A) -> Vec<(f32, S)>;
}

pub trait RewardTable<S> {
    fn reward(&self, state: &S) -> f32;
}