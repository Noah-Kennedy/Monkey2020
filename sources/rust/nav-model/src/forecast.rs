use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use plotters::drawing::IntoDrawingArea;
use plotters::prelude::BitMapBackend;
use plotters::style::{HSLColor, WHITE};

use nav_algo::mdp::value_iteration::{ForecastTable, ForecastTableReadView, ForecastTableWriteView};

use crate::{DiscreteState, RobotStateSpace, RobotVector};

pub struct RobotForecastTable {
    inner: Arc<RwLock<Vec<f32>>>,
    space: RobotStateSpace,
}

impl RobotForecastTable {
    pub fn new(space: RobotStateSpace) -> Self {
        let inner = Arc::new(RwLock::new(
            vec![0.0; space.length as usize * space.ang_res as usize * space.width as usize]
        ));

        Self {
            inner,
            space,
        }
    }
}

#[derive(Debug)]
pub struct RobotForecastTableReadView<'a> {
    inner: RwLockReadGuard<'a, Vec<f32>>,
    space: RobotStateSpace,
}

#[derive(Debug)]
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
            space: self.space.clone(),
        }
    }

    fn write(&'a self) -> Self::WriteView {
        RobotForecastTableWriteView {
            inner: self.inner.write().unwrap(),
            space: self.space.clone(),
        }
    }
}


impl<'a> ForecastTableReadView<DiscreteState> for RobotForecastTableReadView<'a> {
    fn read_forecast(&self, state: &DiscreteState) -> f32 {
        self.inner[index(&self.space, state)]
    }
}

impl<'a> RobotForecastTableReadView<'a> {
    pub fn plot_forecast(&self) {
        for theta in 0..self.space.ang_res {
            let name = format!("images/forecast/{}-I{}.png", theta, self.space.ang_res);

            let root = BitMapBackend::new(&name, (self.space.width as u32, self.space.length as
                u32)).into_drawing_area();

            root.fill(&WHITE).unwrap();

            for x in 0..self.space.width {
                for y in 0..self.space.length {
                    let s = DiscreteState {
                        position: RobotVector {
                            x: x as i32,
                            y: y as i32,
                            r: theta as i32,
                        }
                    };

                    let c = HSLColor(self.read_forecast(&s) as f64, 1.0, 0.5);

                    root.draw_pixel((x as i32, y as i32), &c).unwrap();
                }
            }
        }
    }
}

impl<'a> ForecastTableWriteView<DiscreteState> for RobotForecastTableWriteView<'a> {
    fn write_forecast(&mut self, state: &DiscreteState, value: f32) {
        self.inner[index(&self.space, state)] = value;
    }
}

pub fn index(space: &RobotStateSpace, state: &DiscreteState) -> usize {
    let theta_off = state.position.r as usize;
    let y_off = state.position.y as usize * space.ang_res as usize;
    let x_off = state.position.x as usize * space.length as usize * space.ang_res as usize;
    theta_off + y_off + x_off
}