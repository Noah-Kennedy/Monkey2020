use monkey_vision::core::MonkeyVision;
use crate::aimbot::{Path, Vehicle, SteeringCommand};
use crate::math::Vec2D;
use std::time::{Duration, Instant};
use crate::mesh_to_grid::Grid;
use monkey_api::objects::requests::AutonomousParams;
use monkey_api::objects::{Location, MotorSpeeds};
use libmonkey_sys::monkey_vision::{ZedCameraResolution, ZedDepthQuality, ZedMappingResolution, ZedMappingRange, ZedMeshFilter, ZedImuData};
use monkey_pathfinding::state_space::MonkeyStateSpace;
use monkey_pathfinding::model::{MonkeyModel, DiscreteState, RobotVector};
use monkey_pathfinding::a_star::AStar;
use std::ops::Div;

mod math;
mod mesh_to_grid;
mod aimbot;

const MESH_FILE: &str = "../run-data/mesh.ply";

// Loop variables
static mut KEEP_GOING: bool = true;
/// The theta component should be in degrees.
static mut TARGET_LOCATION: Option<Location> = None;
static mut PATH: Option<Path> = None;
static mut SPEED: MotorSpeeds = MotorSpeeds::default();

pub unsafe fn set_speed(speed: MotorSpeeds) {
    SPEED = speed;
}

pub unsafe fn set_target(target: Option<Location>) {
    TARGET_LOCATION = target;
}

pub unsafe fn end_autonomous_mode() {
    TARGET_LOCATION = None;
    PATH = None;
    KEEP_GOING = false;
}

pub fn zhu_li_do_the_thing(vision: &mut MonkeyVision, autonomous_params: &AutonomousParams) {
    let mut zed_imu_data = ZedImuData::default();
    let mut time_since_last_spatial_map_update = Duration::from_secs(0);
    let mut dt = Duration::from_secs(0);
    let mut last_time = Instant::now();
    let mut grid = Grid::new(autonomous_params.min_x, autonomous_params.max_x, autonomous_params.min_z, autonomous_params.max_z, autonomous_params.res_x, autonomous_params.res_z);
    let astar_model = MonkeyModel {
        length: grid.res_x(),
        width: grid.res_z(),
        ang_res: 64,
        min_turn_rad: (autonomous_params.min_turn_radius / grid.x_scale()).ceil() as u16, // TODO: x_scale vs z_scale?
        max_speed: (autonomous_params.max_speed / grid.x_scale()).floor() as u16,
        rev_max_speed: (autonomous_params.max_speed / grid.x_scale()).floor() as u16
    };

    while unsafe { KEEP_GOING } {
        let now = Instant::now();
        let elapsed = now.duration_since(last_time);
        dt = (dt + elapsed).div(2);
        last_time = now;

        if let Some(target_location) = unsafe { TARGET_LOCATION } {
            if time_since_last_spatial_map_update >= autonomous_params.min_spatial_map_update_period {
                vision.request_map_update();
                time_since_last_spatial_map_update = Duration::from_secs(0);
                match mesh_to_grid::mesh_to_grid(&mut grid, MESH_FILE, autonomous_params.interaction_radius, autonomous_params.vertical_cutoff) {
                    Ok(_) => {
                        // TODO: If path is some, check it against the new grid to ensure that it is still safe. If not safe, clear path.
                        //  Currently just clearing the path regardless
                        unsafe { PATH = None; }
                    },
                    Err(_) => {
                        // TODO: log error?
                    }
                }
                // Note that there is no guarantee that the requested map update will happen before
                // mesh_to_grid is called. It's just that both the request and mesh_to_grid should
                // only be ran periodically as they are computationally expensive.
            }

            vision.imu_data(&mut zed_imu_data);

            if unsafe { PATH.is_none() } {
                let s = MonkeyStateSpace {
                    cost: grid.clone(),
                    model: astar_model.clone()
                };

                let mut astar = AStar::new(s);

                let path = astar.find_path(&DiscreteState {
                    position: RobotVector {
                        x: (zed_imu_data.x_pos / grid.x_scale()).floor() as i16,
                        y: (zed_imu_data.z_pos / grid.z_scale()).floor() as i16,
                        r: (zed_imu_data.z_rot * 64.0 / 360.0).floor() as i16
                    }
                }, &DiscreteState {
                    position: RobotVector {
                        x: (target_location.x / grid.x_scale()).floor() as i16,
                        y: (target_location.y / grid.z_scale()).floor() as i16,
                        r: (target_location.theta * 64.0 / 360.0).floor() as i16
                    }
                });

                unsafe {
                    PATH = path.map(|path| -> Path {
                        let waypoints = path.iter().map(|d| -> Vec2D {
                            Vec2D {
                                x: d.position.x as f32 * grid.x_scale(),
                                y: d.position.y as f32 * grid.z_scale()
                            }
                        }).collect();

                        Path::new(waypoints).unwrap()
                    });
                }
            }

            let (left_lin_speed, right_lin_speed) = unsafe { (SPEED.left_speed * autonomous_params.wheel_radius, SPEED.right_speed * autonomous_params.wheel_radius) };

            let vehicle = Vehicle {
                pos: Vec2D { x: zed_imu_data.x_pos, y: zed_imu_data.z_pos },
                speed: (left_lin_speed, right_lin_speed) / 2.0,
                orientation: zed_imu_data.z_rot.to_radians(),
                max_speed: autonomous_params.max_speed,
                max_force: autonomous_params.max_force,
                mass: autonomous_params.mass,
                moment_of_inertia: autonomous_params.moment_of_inertia,
                allow_backwards: autonomous_params.allow_backwards
            };

            let steering = match unsafe { PATH.as_ref() } {
                Some(path) => vehicle.follow(path, autonomous_params.stopping_dist, target_location.theta.map(|t| -> f32 { t.to_radians() })),
                None => vehicle.stop()
            };

            let (accel, rot_accel) = (steering.thrust / autonomous_params.mass, steering.torque / autonomous_params.moment_of_inertia);
            let (left_accel, right_accel) = (accel + rot_accel * autonomous_params.drive_width / 2.0, accel - rot_accel * autonomous_params.drive_width / 2.0);
            let (left_speed, right_speed) = ((left_lin_speed + left_accel * dt.as_secs_f32()) / autonomous_params.wheel_radius, (right_lin_speed + right_accel * dt.as_secs_f32()) / autonomous_params.wheel_radius);
            // TODO: Send new motor speeds over network
        }
    }
}
