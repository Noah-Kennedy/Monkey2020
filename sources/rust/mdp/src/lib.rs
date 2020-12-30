pub mod value_iteration;

pub trait State {}

pub trait StateSpace {}

pub trait Action {}

pub trait ActionSpace {}

pub trait ProbabilityTable<S, A> where S: State, A: Action {
    fn probability(&self, start: &S, end: &S, action: &A) -> f32;
}

pub trait RewardTable<S, A> where S: State, A: Action {
    fn reward(&self, start: &S, end: &S, action: &A) -> f32;
}

pub trait ForecastTableReader<S, A> where S: State, A: Action {
    fn read_forecast(&self, start: &S, end: &S, action: &A) -> f32;
}

pub trait ForecastTableWriter<S, A> where S: State, A: Action {
    fn write_forecast(&self, start: &S, end: &S, action: &A, value: f32) -> f32;
}

//
// pub trait State {
//     /// The intrinsic, immutable value of the state.
//     fn intrinsic_value(&self) -> f32;
//     fn is_terminal(&self) -> bool;
// }
//
// pub trait StateSpace<S> {
//
// }
//
// pub trait StateSpaceReadView<'a, S> {
//     fn neighbors(&self, output: &mut Vec<S>);
// }