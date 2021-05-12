use std::time::{Duration, Instant};

use log::error;
use tokio::sync::watch;
use crossbeam::channel;

use libmonkey_sys::monkey_vision::ZedImuData;
use monkey_api::{Location, MotorSpeeds};
use monkey_api::requests::AutonomousParams;
use monkey_pathfinding::a_star::AStar;
use monkey_pathfinding::model::{DiscreteState, MonkeyModel, RobotVector};
use monkey_pathfinding::state_space::MonkeyStateSpace;
use monkey_vision::core::MonkeyVision;

use crate::aimbot::{Path, Vehicle};
use crate::math::Vec2D;
use crate::mesh_to_grid::Grid;

pub mod math;
pub mod mesh_to_grid;
mod aimbot;

const MESH_FILE: &str = "mesh.ply";

/// Commands for interacting with the navigation system.
#[derive(Debug, PartialOrd, PartialEq, Clone)]
pub enum Command {
    StartNav(AutonomousParams),
    SetSpeed(MotorSpeeds),
    /// The theta component should be in degrees.
    SetTarget(Option<Location>),
    EndNav,
}

pub struct NavState {
    speed: MotorSpeeds,
    target: Option<Location>,
    keep_going: bool,
    imu_data: ZedImuData,
    path: Option<Path>,
    grid: Grid,
    time_since_last_spatial_map_update: Duration,
    last_time: Instant,
}

impl NavState {
    fn init(params: &AutonomousParams) -> NavState {
        NavState {
            speed: Default::default(),
            target: None,
            keep_going: true,
            imu_data: Default::default(),
            path: None,
            grid: Grid::new(params.min_x, params.max_x, params.min_z, params.max_z, params.res_x, params.res_z),
            time_since_last_spatial_map_update: Duration::from_secs(0),
            last_time: Instant::now(),
        }
    }
}

/// Channels for communicating with the autonomous mode controller.
pub struct ZhuLi {
    pub command_rec: channel::Receiver<Command>,
    pub speed_send: watch::Sender<MotorSpeeds>,
    pub params: AutonomousParams,
    pub state: Option<NavState>
}

impl ZhuLi {
    pub fn poll_commands(&mut self) {
        while !self.command_rec.is_empty() {
            match self.command_rec.try_recv().unwrap() {
                Command::StartNav(params) => {
                    self.state = Some(NavState::init(&params));
                    self.params = params;
                },
                Command::EndNav => self.state = None,
                Command::SetSpeed(speed) => {
                    if let Some(state) = &mut self.state {
                        state.speed = speed;
                    }
                },
                Command::SetTarget(target) => {
                    if let Some(state) = &mut self.state {
                        state.target = target;
                    }
                }
            }
        }
    }

    /// Starts the autonomous control loop. This blocks until `Command::EndAutonomous` is sent.
    pub fn do_the_thing(&mut self, vision: &mut MonkeyVision, mesh_file: &str) {
        if let Some(state) = self.state.as_mut() {
            let now = Instant::now();
            let dt = now.duration_since(state.last_time);
            state.last_time = now;

            state.time_since_last_spatial_map_update += dt;
            if state.time_since_last_spatial_map_update >= self.params.min_mesh_to_grid_period {
                match mesh_to_grid::mesh_to_grid(&mut state.grid, mesh_file, self.params.interaction_radius, self.params.vertical_cutoff) {
                    Ok(_) => {
                        // TODO: If path is some, check it against the new grid to ensure that it is still safe. If not safe, clear path.
                        //  Currently just clearing the path regardless
                        state.path = None;
                    }
                    Err(err) => error!("{:?}", err)
                }
                state.time_since_last_spatial_map_update = Duration::from_secs(0);
            }

            if let Err(err) = vision.imu_data(&mut state.imu_data) {
                error!("{:?}", err);
            }

            let new_speeds = ZhuLi::the_thing(state, &self.params, dt);
            if let Err(err) = self.speed_send.send(new_speeds) {
                error!("{:?}", err);
            }
        }
    }

    fn the_thing(state: &mut NavState, params: &AutonomousParams, dt: Duration) -> MotorSpeeds {
        let left_lin_speed = state.speed.left_speed * params.wheel_radius;
        let right_lin_speed = state.speed.right_speed * params.wheel_radius;

        let vehicle = Vehicle {
            pos: Vec2D { x: state.imu_data.x_pos, y: state.imu_data.z_pos },
            speed: (left_lin_speed + right_lin_speed) / 2.0,
            orientation: state.imu_data.z_rot.to_radians(),
            max_speed: params.max_speed,
            max_force: params.max_force,
            mass: params.mass,
            moment_of_inertia: params.moment_of_inertia,
            allow_backwards: params.allow_backwards,
        };

        let steering = match state.target.as_ref() {
            Some(target) => {
                if state.path.is_none() {
                    // TODO: Have path computation on a separate thread?
                    let s = MonkeyStateSpace {
                        cost: state.grid.clone(),
                        model: MonkeyModel {
                            length: state.grid.res_x(),
                            width: state.grid.res_z(),
                            ang_res: 64,
                            min_turn_rad: (params.min_turn_radius / state.grid.x_scale()).ceil() as u16, // TODO: x_scale vs z_scale?
                            max_speed: (params.max_speed / state.grid.x_scale()).floor() as u16,
                            rev_max_speed: (params.max_speed / state.grid.x_scale()).floor() as u16,
                        },
                    };

                    let mut astar = AStar::new(s);

                    let discrete_path = astar.find_path(&DiscreteState {
                        position: RobotVector {
                            x: (state.imu_data.x_pos / state.grid.x_scale()).floor() as i16,
                            y: (state.imu_data.z_pos / state.grid.z_scale()).floor() as i16,
                            r: (state.imu_data.z_rot * 64.0 / 360.0).floor() as i16,
                        }
                    }, &DiscreteState {
                        position: RobotVector {
                            x: (target.x / state.grid.x_scale()).floor() as i16,
                            y: (target.y / state.grid.z_scale()).floor() as i16,
                            r: (target.theta * 64.0 / 360.0).floor() as i16,
                        }
                    });

                    state.path = discrete_path.map(|path| -> Path {
                        let waypoints = path.iter().map(|d| -> Vec2D {
                            Vec2D {
                                x: d.position.x as f32 * state.grid.x_scale(),
                                y: d.position.y as f32 * state.grid.z_scale(),
                            }
                        }).collect();

                        Path::new(waypoints).unwrap()
                    });
                }

                match state.path.as_ref() {
                    Some(path) => vehicle.follow(path, params.stopping_dist, target.theta.to_radians(), dt),
                    None => vehicle.stop()
                }
            }
            None => vehicle.stop()
        };

        let accel = steering.thrust / params.mass;
        let rot_accel = steering.torque / params.moment_of_inertia;
        let left_accel = accel + rot_accel * params.drive_width / 2.0;
        let right_accel = accel - rot_accel * params.drive_width / 2.0;
        let left_speed = (left_lin_speed + left_accel * dt.as_secs_f32()) / params.wheel_radius;
        let right_speed = (right_lin_speed + right_accel * dt.as_secs_f32()) / params.wheel_radius;

        MotorSpeeds { right_speed, left_speed }
    }
}
