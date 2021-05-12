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
#[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
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

    fn apply_command(&mut self, command: Command) {
        match command {
            Command::StartNav(params) => self.params = params,
            Command::SetSpeed(speed) => self.speed = speed,
            Command::SetTarget(target) => {
                self.target = target;
                self.path = None;
            },
            Command::EndNav => {
                self.path = None;
                self.target = None;
                self.keep_going = false;
            }
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
        if let Some(state) = &mut self.state {
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

            let new_speeds = self.the_thing(dt);
            if let Err(err) = self.speed_send.send(new_speeds) {
                error!("{:?}", err);
            }
        }
    }

    fn the_thing(&mut self, dt: Duration) -> MotorSpeeds {
        let left_lin_speed = self.state.speed.left_speed * self.params.wheel_radius;
        let right_lin_speed = self.state.speed.right_speed * self.params.wheel_radius;

        let vehicle = Vehicle {
            pos: Vec2D { x: self.state.imu_data.x_pos, y: self.state.imu_data.z_pos },
            speed: (left_lin_speed + right_lin_speed) / 2.0,
            orientation: self.state.imu_data.z_rot.to_radians(),
            max_speed: self.params.max_speed,
            max_force: self.params.max_force,
            mass: self.params.mass,
            moment_of_inertia: self.params.moment_of_inertia,
            allow_backwards: self.params.allow_backwards,
        };

        let steering = match self.state.target.as_ref() {
            Some(target) => {
                if self.state.path.is_none() {
                    // TODO: Have path computation on a separate thread?
                    let s = MonkeyStateSpace {
                        cost: self.state.grid.clone(),
                        model: MonkeyModel {
                            length: self.state.grid.res_x(),
                            width: self.state.grid.res_z(),
                            ang_res: 64,
                            min_turn_rad: (self.params.min_turn_radius / self.state.grid.x_scale()).ceil() as u16, // TODO: x_scale vs z_scale?
                            max_speed: (self.params.max_speed / self.state.grid.x_scale()).floor() as u16,
                            rev_max_speed: (self.params.max_speed / self.state.grid.x_scale()).floor() as u16,
                        },
                    };

                    let mut astar = AStar::new(s);

                    let discrete_path = astar.find_path(&DiscreteState {
                        position: RobotVector {
                            x: (self.state.imu_data.x_pos / self.state.grid.x_scale()).floor() as i16,
                            y: (self.state.imu_data.z_pos / self.state.grid.z_scale()).floor() as i16,
                            r: (self.state.imu_data.z_rot * 64.0 / 360.0).floor() as i16,
                        }
                    }, &DiscreteState {
                        position: RobotVector {
                            x: (target.x / self.state.grid.x_scale()).floor() as i16,
                            y: (target.y / self.state.grid.z_scale()).floor() as i16,
                            r: (target.theta * 64.0 / 360.0).floor() as i16,
                        }
                    });

                    self.state.path = discrete_path.map(|path| -> Path {
                        let waypoints = path.iter().map(|d| -> Vec2D {
                            Vec2D {
                                x: d.position.x as f32 * self.state.grid.x_scale(),
                                y: d.position.y as f32 * self.state.grid.z_scale(),
                            }
                        }).collect();

                        Path::new(waypoints).unwrap()
                    });
                }

                match self.state.path.as_ref() {
                    Some(path) => vehicle.follow(path, self.params.stopping_dist, target.theta.to_radians(), dt),
                    None => vehicle.stop()
                }
            }
            None => vehicle.stop()
        };

        let accel = steering.thrust / self.params.mass;
        let rot_accel = steering.torque / self.params.moment_of_inertia;
        let left_accel = accel + rot_accel * self.params.drive_width / 2.0;
        let right_accel = accel - rot_accel * self.params.drive_width / 2.0;
        let left_speed = (left_lin_speed + left_accel * dt.as_secs_f32()) / self.params.wheel_radius;
        let right_speed = (right_lin_speed + right_accel * dt.as_secs_f32()) / self.params.wheel_radius;

        MotorSpeeds { right_speed, left_speed }
    }
}
