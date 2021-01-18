use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use crate::{RewardTable, StateSpace};
use crate::mdp::value_iteration::{ForecastTable, ForecastTableReadView, ForecastTableWriteView, ValueIterationMDPSystem, ValueIterationParameters};

type GridState = (usize, usize);
type GridAction = (isize, isize);

const REWARDS: [[f32; 4]; 3] = [
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, -1.0],
    [0.0, 0.0, 0.0, 0.0]
];

const TERMINAL: [[bool; 4]; 3] = [
    [false, false, false, true],
    [false, false, false, true],
    [false, false, false, false]
];

const EXISTS: [[bool; 4]; 3] = [
    [true, true, true, true],
    [true, false, true, true],
    [true, true, true, true]
];

const ACTIONS: [GridAction; 4] = [
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1)
];

struct GridStateSpace;

struct GridRewardTable;

#[derive(Default)]
struct GridForecastTable {
    inner: Arc<RwLock<[[f32; 4]; 3]>>
}

struct GridForecastTableReadView<'a> {
    inner: RwLockReadGuard<'a, [[f32; 4]; 3]>
}

struct GridForecastTableWriteView<'a> {
    inner: RwLockWriteGuard<'a, [[f32; 4]; 3]>
}

impl<'a> ForecastTableReadView<GridState> for GridForecastTableReadView<'a> {
    fn read_forecast(&self, state: &GridState) -> f32 {
        self.inner[state.0][state.1]
    }
}

impl<'a> ForecastTableWriteView<GridState> for GridForecastTableWriteView<'a> {
    fn write_forecast(&mut self, state: &GridState, value: f32) {
        self.inner[state.0][state.1] = value;
    }
}

impl<'a> ForecastTable<'a, GridState> for GridForecastTable {
    type ReadView = GridForecastTableReadView<'a>;
    type WriteView = GridForecastTableWriteView<'a>;

    fn read(&'a self) -> Self::ReadView {
        GridForecastTableReadView {
            inner: self.inner.read().unwrap()
        }
    }

    fn write(&'a self) -> Self::WriteView {
        GridForecastTableWriteView {
            inner: self.inner.write().unwrap()
        }
    }
}

impl StateSpace<GridState, GridAction> for GridStateSpace {
    fn nonterminal_states(&self) -> Arc<Vec<(usize, usize)>> {
        let mut out = Vec::new();

        for r in 0..3 {
            for c in 0..4 {
                if EXISTS[r][c] && !TERMINAL[r][c] {
                    out.push((r, c));
                }
            }
        }

        Arc::new(out)
    }
    fn terminal(&self, state: &(usize, usize)) -> bool {
        TERMINAL[state.0][state.1]
    }

    fn actions(&self, state: &GridState) -> Vec<GridAction> {
        ACTIONS.iter()
            .map(|(r, c)| ((*r, *c), (*r + state.0 as isize, *c + state.1 as isize)))
            .filter(|(_, (r, c))|
                *r >= 0
                    && *c >= 0
                    && *r < 3
                    && *c < 4
                    && EXISTS[*r as usize][*c as usize]
            )
            .map(|(a, _)| a)
            .collect()
    }

    fn q_states(&self, state: &GridState, action: &GridAction) -> Vec<(f32, GridState)> {
        let mut q_states = Vec::new();

        q_states.push((
            0.8,
            (
                (state.0 as isize + action.0) as usize,
                (state.1 as isize + action.1) as usize
            ))
        );

        let actions = self.actions(state);

        let action_states: Vec<GridState> = actions.iter()
            .map(|(x, y)| ((state.0 as isize + *x) as usize,
                           (state.1 as isize + *y) as usize))
            .filter(|s| s != &q_states[0].1)
            .collect();

        let n = 0.2 / action_states.len() as f32;

        for s in action_states {
            q_states.push((n, s));
        }

        q_states
    }
}

impl RewardTable<GridState> for GridRewardTable {
    fn reward(&self, state: &GridState) -> f32 {
        REWARDS[state.0][state.1]
    }
}

#[test]
fn test_epoch_1() {
    let params = ValueIterationParameters {
        epochs: 1,
        discount: 0.9,
        living_reward: 0.0,
    };

    const EXTRINSIC: [[f32; 4]; 3] =
        [
            [0.00, 0.00, 0.72, 0.00],
            [0.00, 0.00, -0.09, 0.00],
            [0.00, 0.00, 0.00, -0.18]
        ];

    run_test(&params, EXTRINSIC)
}

fn run_test(params: &ValueIterationParameters, extrinsic: [[f32; 4]; 3]) {
    let sys = ValueIterationMDPSystem {
        reward_table: GridRewardTable,
        state_space: GridStateSpace,
        _phantom: Default::default(),
    };

    let primary = GridForecastTable::default();
    let secondary = GridForecastTable::default();

    let idx = sys.solve_forecasts(params, &primary, &secondary);

    let arr = [primary, secondary];

    let keep = &arr[idx as usize];

    let inner = keep.read().inner;

    for r in 0..2 {
        for c in 0..3 {
            assert!(inner[r][c] - f32::EPSILON <= extrinsic[r][r]
                || inner[r][c] + f32::EPSILON >= extrinsic[r][r]);
        }
    }
}