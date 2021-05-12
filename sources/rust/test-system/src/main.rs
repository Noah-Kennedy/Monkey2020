use std::thread;
use std::time::Duration;

use monkey_api::{MotorSpeeds, Location};
use monkey_api::requests::AutonomousParams;
use monkey_vision::prelude::{ZedCameraResolution, ZedDepthQuality, ZedMappingRange, ZedMappingResolution, ZedMeshFilter};
use space_monkeys::{Command, ZhuLi};
use monkey_unified_command::command::NavManager;
use monkey_vision::core::MonkeyVision;
use tokio::sync::watch;
use crossbeam::channel;

const AUTO_PARAMS: AutonomousParams = AutonomousParams {
    max_speed: 100.0,
    max_force: 100.0,
    mass: 1.0,
    moment_of_inertia: 1.0,
    allow_backwards: false,
    stopping_dist: 100.0,
    interaction_radius: 0.2,
    min_x: -5.0,
    max_x: 5.0,
    min_z: -5.0,
    max_z: 5.0,
    res_x: 100,
    res_z: 100,
    min_mesh_to_grid_period: Duration::from_millis(1000),
    vertical_cutoff: 1.0,
    min_turn_radius: 5.0,
    drive_width: 0.8,
    wheel_radius: 0.1,
};

fn main() {
    let mesh_file = "mesh.ply";
    let mut vision = monkey_vision::core::create(
        mesh_file,
        ZedCameraResolution::Res720HD60,
        ZedDepthQuality::DepthPerformance,
        ZedMappingResolution::MapMediumRes,
        ZedMappingRange::MapMedium,
        ZedMeshFilter::FilterMedium,
    ).unwrap().0;

    let mut speed = MotorSpeeds::default();

    let (command_send, command_rec) = channel::unbounded();
    let (speed_send, speed_rec) = watch::channel(MotorSpeeds::default());

    let nav_manager = NavManager { command_send, speed_rec };
    let mut zhu_li = ZhuLi { command_rec, speed_send };
    let join_handle = thread::spawn(move || zhu_li.do_the_thing(&mut vision, mesh_file, &AUTO_PARAMS));

    nav_manager.command_send.send(Command::SetTarget(Some(Location {
        x: 2.0,
        y: 3.0,
        theta: 90.0
    })));

    for i in 0..1000 {
        println!("{:?}: {:?}", i, speed);
        nav_manager.command_send.send(Command::SetSpeed(speed)).unwrap();
        if let Ok(s) = nav_manager.speed_rec.try_recv() {
            speed = s;
        }
        thread::sleep(Duration::from_millis(10));
    }

    nav_manager.command_send.send(Command::EndAutonomous).unwrap();
    join_handle.join().unwrap();
}
