use std::time::Duration;

use log::error;

use libmonkey_sys::monkey_vision::ZedImuData;
use monkey_api::objects::{Location, MotorSpeeds};
use monkey_api::objects::requests::AutonomousParams;
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

const MESH_FILE: &str = "../run-data/mesh.ply";

pub struct AutonomousState {
    pub speed: MotorSpeeds,
    /// The theta component should be in degrees.
    pub target_location: Option<Location>,
    pub time_since_last_spatial_map_update: Duration,
    pub path: Option<Path>,
    pub grid: Grid,
    pub zed_imu_data: ZedImuData,
}

pub fn zhu_li_do_the_thing(vision: &mut MonkeyVision, params: &AutonomousParams, state: &mut AutonomousState, dt: Duration) -> MotorSpeeds {
    if let Err(err) = vision.imu_data(&mut state.zed_imu_data) {
        error!("{:?}", err);
    }

    let left_lin_speed = state.speed.left_speed * params.wheel_radius;
    let right_lin_speed = state.speed.right_speed * params.wheel_radius;

    let vehicle = Vehicle {
        pos: Vec2D { x: state.zed_imu_data.x_pos, y: state.zed_imu_data.z_pos },
        speed: (left_lin_speed + right_lin_speed) / 2.0,
        orientation: state.zed_imu_data.z_rot.to_radians(),
        max_speed: params.max_speed,
        max_force: params.max_force,
        mass: params.mass,
        moment_of_inertia: params.moment_of_inertia,
        allow_backwards: params.allow_backwards,
    };

    let steering = match state.target_location {
        Some(target_location) => {
            if state.time_since_last_spatial_map_update.as_millis() as usize
                >= params.min_spatial_map_update_period
            {
                vision.request_map_update();
                state.time_since_last_spatial_map_update = Duration::from_secs(0);
                match mesh_to_grid::mesh_to_grid(&mut state.grid, MESH_FILE, params.interaction_radius, params.vertical_cutoff) {
                    Ok(_) => {
                        // TODO: If path is some, check it against the new grid to ensure that it is still safe. If not safe, clear path.
                        //  Currently just clearing the path regardless
                        state.path = None;
                    }
                    Err(err) => error!("{:?}", err)
                }
                // Note that there is no guarantee that the requested map update will happen before
                // mesh_to_grid is called. It's just that both the request and mesh_to_grid should
                // only be ran periodically as they are computationally expensive.
            }

            if state.path.is_none() {
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

                let path = astar.find_path(&DiscreteState {
                    position: RobotVector {
                        x: (state.zed_imu_data.x_pos / state.grid.x_scale()).floor() as i16,
                        y: (state.zed_imu_data.z_pos / state.grid.z_scale()).floor() as i16,
                        r: (state.zed_imu_data.z_rot * 64.0 / 360.0).floor() as i16,
                    }
                }, &DiscreteState {
                    position: RobotVector {
                        x: (target_location.x / state.grid.x_scale()).floor() as i16,
                        y: (target_location.y / state.grid.z_scale()).floor() as i16,
                        r: (target_location.theta * 64.0 / 360.0).floor() as i16,
                    }
                });

                state.path = path.map(|path| -> Path {
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
                Some(path) => vehicle.follow(path, params.stopping_dist, target_location.theta.to_radians(), dt),
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
