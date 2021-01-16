use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use crate::{DiscreteState, RobotStateSpace};
use mdp::value_iteration::{ForecastTableReadView, ForecastTableWriteView, ForecastTable};

pub struct RobotForecastTable {
    inner: Arc<RwLock<Vec<f32>>>,
    space: RobotStateSpace,
}

impl RobotForecastTable {
    pub fn new(space: RobotStateSpace) -> Self {
        let inner = Arc::new(RwLock::new(
            vec![0.0; space.length as usize * space.max_pos_r as usize * space.width as usize]
        ));

        Self {
            inner,
            space
        }
    }
}

pub struct RobotForecastTableReadView<'a> {
    inner: RwLockReadGuard<'a, Vec<f32>>,
    space: RobotStateSpace,
}

pub struct RobotForecastTableWriteView<'a> {
    inner: RwLockWriteGuard<'a, Vec<f32>>,
    space: RobotStateSpace,
}

impl<'a> ForecastTable<'a, DiscreteState> for RobotForecastTable {
    type ReadView = RobotForecastTableReadView<'a>;
    type WriteView = RobotForecastTableWriteView<'a>;

    fn read(&'a self) -> Self::ReadView {
        RobotForecastTableReadView {
            inner: self.inner.read().unwrap(),
            space: self.space.clone()
        }
    }

    fn write(&'a self) -> Self::WriteView {
        RobotForecastTableWriteView {
            inner: self.inner.write().unwrap(),
            space: self.space.clone()
        }
    }
}


impl<'a> ForecastTableReadView<DiscreteState> for RobotForecastTableReadView<'a> {
    fn read_forecast(&self, state: &DiscreteState) -> f32 {
        self.inner[index(&self.space, state)]
    }
}

impl<'a> ForecastTableWriteView<DiscreteState> for RobotForecastTableWriteView<'a> {
    fn write_forecast(&mut self, state: &DiscreteState, value: f32) {
        self.inner[index(&self.space, state)] = value;
    }
}

fn index(space: &RobotStateSpace, state: &DiscreteState) -> usize {
    let theta_off = state.position.r as usize;
    let y_off = state.position.y as usize * space.max_pos_r as usize;
    let x_off = state.position.x as usize * space.length as usize * space.max_pos_r as usize;
    theta_off + y_off + x_off
}