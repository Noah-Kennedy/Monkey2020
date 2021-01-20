use nav_algo::{RewardTable, StateSpace};
use nav_algo::a_star::AStarStateSpace;

use crate::{DiscreteState, RobotStateSpace};

pub struct RobotAStarStateSpace<R> {
    pub cost: R,
    pub space: RobotStateSpace,
}

impl<R> AStarStateSpace<DiscreteState> for RobotAStarStateSpace<R>
    where R: RewardTable<DiscreteState>
{
    fn actions(&self, state: &DiscreteState) -> Vec<(f32, DiscreteState)> {
        self.space.actions(state)
            .iter()
            .map(|a| {
                let s2 = self.space.apply_state(state, a);
                let g = self.cost.reward(&s2).abs();
                (g + 0.005 * (a.velocity.x.pow(2) as f32 + a.velocity.y.pow(2) as f32).sqrt(), s2)
            })
            .collect()
    }

    fn heuristic(&self, state: &DiscreteState, goal: &DiscreteState) -> f32 {
        ((state.position.x as f32 - goal.position.x as f32).powi(2)
            + (state.position.y as f32 - goal.position.y as f32).powi(2)
        ).sqrt()
            * 0.005 as f32
    }
}