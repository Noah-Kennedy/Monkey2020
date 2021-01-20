use crate::model::{MonkeyModel, DiscreteState};
use crate::a_star::StateSpace;

pub trait RewardTable<S> {
    fn reward(&self, state: &S) -> f32;
}

pub struct MonkeyStateSpace<R> {
    pub cost: R,
    pub model: MonkeyModel,
}

impl<R> StateSpace<DiscreteState> for MonkeyStateSpace<R>
    where R: RewardTable<DiscreteState>
{
    #[inline(never)]
    fn neighbors(&self, state: &DiscreteState) -> Vec<(f32, DiscreteState)> {
        self.model.actions(state)
            .iter()
            .map(|a| {
                let s2 = self.model.apply_state(state, a);
                let g = self.cost.reward(&s2).abs();
                (g + 0.005 * (a.velocity.x.pow(2) as f32 + a.velocity.y.pow(2) as f32).sqrt(), s2)
            })
            .collect()
    }

    fn heuristic(&self, state: &DiscreteState, goal: &DiscreteState) -> f32 {
        ((state.position.x as f32 - goal.position.x as f32).powi(2)
            + (state.position.y as f32 - goal.position.y as f32).powi(2)
        ).sqrt()
            * 0.005
    }
}