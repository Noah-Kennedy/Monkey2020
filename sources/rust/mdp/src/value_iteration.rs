use std::marker::PhantomData;

use crate::{Action, ActionSpace, RewardTable, State, StateSpace, TerminalTable, TransitionTable};

pub trait ForecastTable<S> where S: State {
    type ReadView: ForecastTableReadView<S>;
    type WriteView: ForecastTableWriteView<S>;

    fn read(&self) -> Self::ReadView;
    fn write(&self) -> Self::WriteView;
}

pub trait ForecastTableReadView<S> where S: State {
    fn read_forecast(&self, state: &S) -> f32;
}

pub trait ForecastTableWriteView<S> where S: State {
    fn write_forecast(&self, state: &S, value: f32) -> f32;
}

pub struct ValueIterationParameters {
    pub epochs: u32,
    pub living_reward: f32,
    pub discount: f32,
    pub noise: f32,
}

pub struct ValueIterationMDPSystem<'a, 's, S, A, Ss, As, Te, Tr, Re>
    where S: State + 's,
          A: Action + 'a,
          Te: TerminalTable<S>,
          Tr: TransitionTable<'a, 's, S, A>,
          Re: RewardTable<S, A>,
          Ss: StateSpace<'s, S>,
          As: ActionSpace<A>,
{
    pub terminal_table: Te,
    pub reward_table: Re,
    pub transition_table: Tr,
    pub state_space: Ss,
    pub action_space: As,
    pub _phantom: PhantomData<(&'s S, &'a A)>,
}

impl<'a, 's, S, A, Ss, As, Te, Tr, Re>
ValueIterationMDPSystem<'a, 's, S, A, Ss, As, Te, Tr, Re>
    where S: State + 's,
          A: Action + 'a,
          Te: TerminalTable<S>,
          Tr: TransitionTable<'a, 's, S, A>,
          Re: RewardTable<S, A>,
          Ss: StateSpace<'s, S>,
          As: ActionSpace<A>,
{
    pub fn solve_forecasts<Fo>(
        &self,
        params: &ValueIterationParameters,
        first_table: &Fo,
        second_table: &Fo,
    )
        -> u8
        where Fo: ForecastTable<S>,
    {
        assert!(params.noise >= 0.0 && params.noise <= 1.0);

        for i in 0..(params.epochs) {
            let (input, output) = if i % 2 == 0 {
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

            self.run_iteration(params, &input, &output);
        }

        (params.epochs % 2) as u8
    }

    fn run_iteration<I, O>(
        &self,
        params: &ValueIterationParameters,
        input: &I,
        output: &O,
    )
        where I: ForecastTableReadView<S>,
              O: ForecastTableWriteView<S>
    {
        let mut choices = Vec::new();

        for state in self.state_space.iter_nonterminal_states() {
            let mut value = params.living_reward;

            for (action, new_state) in self.transition_table.list_transitions(state) {
                let utility = self.reward_table.reward(state, new_state, action)
                    + input.read_forecast(new_state);

                choices.push((utility, action, new_state));
            }

            let (max_idx, (utility, _, _)) = choices.iter()
                .enumerate()
                .max_by(|&(_, (k1, _, _)), &(_, (k2, _, _))| k1.partial_cmp(k2).unwrap())
                .unwrap();

            value += *utility * (1.0 - params.noise);

            choices.remove(max_idx);

            let noise_prop = params.noise / choices.len() as f32;

            for (utility, _, _) in choices.iter() {
                value += noise_prop * utility
            }

            value *= params.discount;

            output.write_forecast(state, value);
        }
    }
}