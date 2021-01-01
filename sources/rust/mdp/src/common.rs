pub trait State: Clone + PartialEq + PartialOrd {}

pub trait StateSpace<S> where S: State {
    fn nonterminal_states(&self) -> Vec<S>;
}

pub trait TransitionTable<S, A> where S: State, A: Action {
    fn list_transitions(&self, state: &S) -> Vec<(A, S)>;
}

pub trait Action: Clone + PartialEq + PartialOrd {}

pub trait TerminalTable<S> where S: State {
    fn terminal(&self, state: &S) -> bool;
}

pub trait RewardTable<S, A> where S: State, A: Action {
    fn reward(&self, start: &S, end: &S, action: &A) -> f32;
}