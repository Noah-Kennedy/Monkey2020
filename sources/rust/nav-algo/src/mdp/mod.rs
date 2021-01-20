use crate::StateSpace;
use std::sync::Arc;

pub mod value_iteration;
#[cfg(test)]
mod tests;

pub trait MDPStateSpace<S, A>: StateSpace<S, A> {
    fn terminal(&self, state: &S) -> bool;
    fn nonterminal_states(&self) -> Arc<Vec<S>>;
    fn q_states(&self, state: &S, action: &A) -> Vec<(f32, S)>;
}