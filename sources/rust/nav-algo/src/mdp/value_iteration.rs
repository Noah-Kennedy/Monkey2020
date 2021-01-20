use std::marker::PhantomData;

use crate::{RewardTable};
use crate::mdp::MDPStateSpace;

pub trait ForecastTable<'a, S> {
    type ReadView: ForecastTableReadView<S>;
    type WriteView: ForecastTableWriteView<S>;

    fn read(&'a self) -> Self::ReadView;
    fn write(&'a self) -> Self::WriteView;
}

pub trait ForecastTableReadView<S> {
    fn read_forecast(&self, state: &S) -> f32;
}

pub trait ForecastTableWriteView<S> {
    fn write_forecast(&mut self, state: &S, value: f32);
}

pub struct ValueIterationParameters {
    pub epochs: u32,
    pub living_reward: f32,
    pub discount: f32,
}

pub struct ValueIterationMDPSystem<S, A, Ss, Re> {
    pub reward_table: Re,
    pub state_space: Ss,
    pub _phantom: PhantomData<(S, A)>,
}

impl<S, A, Ss, Re>
ValueIterationMDPSystem<S, A, Ss, Re>
    where Re: RewardTable<S>,
          Ss: MDPStateSpace<S, A>,
{
    pub fn solve_forecasts<'a, Fo>(
        &self,
        params: &ValueIterationParameters,
        first_table: &'a Fo,
        second_table: &'a Fo,
    )
        -> u8
        where Fo: ForecastTable<'a, S>,
    {
        let states = self.state_space.nonterminal_states();

        for i in 0..(params.epochs) {
            println!("Epoch {}", i);
            let (input, mut output) =
                if i % 2 == 0 {
                    (
                        first_table.read(),
                        second_table.write()
                    )
                } else {
                    (
                        second_table.read(),
                        first_table.write()
                    )
                };

            self.run_iteration(params, &states, &input, &mut output);
        }

        (params.epochs % 2) as u8
    }

    fn run_iteration<'a, I, O>(
        &self,
        params: &ValueIterationParameters,
        states: &Vec<S>,
        input: &'a I,
        output: &'a mut O,
    )
        where I: ForecastTableReadView<S>,
              O: ForecastTableWriteView<S>
    {
        for s in states.iter() {
            let mut value = params.living_reward;

            let actions = self.state_space.actions(s);

            for a in actions.iter() {
                let q_states = self.state_space.q_states(s, a);

                let v = q_states.iter()
                    .map(|(p, s1)| {
                        if self.state_space.terminal(s1) {
                            p * (self.reward_table.reward(s1))
                        } else {
                            p * (self.reward_table.reward(s1) + input.read_forecast(s1))
                        }
                    })
                    .sum();

                value = value.max(v);
            }

            value *= params.discount;

            output.write_forecast(s, value);
        }
    }
}