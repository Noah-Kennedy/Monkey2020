pub trait StateSpace<S, A> {
    fn actions(&self, state: &S) -> Vec<A>;
}

pub trait RewardTable<S> {
    fn reward(&self, state: &S) -> f32;
}