use std::marker::PhantomData;

use crate::{Action, RewardTable, State, StateSpace, TerminalTable, TransitionTable};

pub trait ForecastTable<'a, S> where S: State {
    type ReadView: ForecastTableReadView<S>;
    type WriteView: ForecastTableWriteView<S>;

    fn read(&'a self) -> Self::ReadView;
    fn write(&'a self) -> Self::WriteView;
}

pub trait ForecastTableReadView<S> where S: State {
    fn read_forecast(&self, state: &S) -> f32;
}

pub trait ForecastTableWriteView<S> where S: State {
    fn write_forecast(&mut self, state: &S, value: f32) -> f32;
}

pub struct ValueIterationParameters {
    pub epochs: u32,
    pub living_reward: f32,
    pub discount: f32,
    pub noise: f32,
}

pub struct ValueIterationMDPSystem<S, A, Ss, Te, Tr, Re>
    where S: State,
          A: Action,
          Te: TerminalTable<S>,
          Tr: TransitionTable<S, A>,
          Re: RewardTable<S, A>,
          Ss: StateSpace<S>,
{
    pub terminal_table: Te,
    pub reward_table: Re,
    pub transition_table: Tr,
    pub state_space: Ss,
    pub _phantom: PhantomData<(S, A)>,
}

impl<S, A, Ss, Te, Tr, Re>
ValueIterationMDPSystem<S, A, Ss, Te, Tr, Re>
    where S: State,
          A: Action,
          Te: TerminalTable<S>,
          Tr: TransitionTable<S, A>,
          Re: RewardTable<S, A>,
          Ss: StateSpace<S>,
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
        assert!(params.noise >= 0.0 && params.noise <= 1.0);

        let states = self.state_space.nonterminal_states();

        for i in 0..(params.epochs) {
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
        let mut choices = Vec::new();

        for state in states.iter() {
            let mut value = params.living_reward;

            let transitions = self.transition_table.list_transitions(state);

            for (action, new_state) in transitions.iter() {
                let mut utility = self.reward_table.reward(&state, &new_state, &action);

                if !self.terminal_table.terminal(&new_state) {
                    utility += input.read_forecast(&new_state);
                }

                choices.push((utility, action.to_owned(), new_state.to_owned()));
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

            choices.clear();

            value *= params.discount;

            output.write_forecast(&state, value);
        }
    }
}