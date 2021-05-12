mod util;
pub mod camera;
pub mod command;

#[cfg(test)]
mod tests {
    use std::thread;
    use std::time::Duration;

    use cameralot::prelude::*;
    use monkey_api::MotorSpeeds;
    use monkey_api::requests::AutonomousParams;
    use monkey_vision::prelude::{ZedCameraResolution, ZedDepthQuality, ZedMappingRange, ZedMappingResolution, ZedMeshFilter};
    use space_monkeys::{Command, ZhuLi};

    use crate::command::CommandManager;

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
        camera_res: ZedCameraResolution::Res720HD60,
        depth_quality: ZedDepthQuality::DepthPerformance,
        map_res: ZedMappingResolution::MapMediumRes,
        range: ZedMappingRange::MapMedium,
        mesh_filter: ZedMeshFilter::FilterMedium,
    };

    #[test]
    fn sys_nav() {
        let mut speed = MotorSpeeds::default();

        let mut instagram_hoe = OpenCVCameraFeed::new();
        instagram_hoe.open(0);

        let (command_send, command_rec) = crossbeam::channel::unbounded();
        let (speed_send, speed_rec) = crossbeam::channel::unbounded();

        let command_manager = CommandManager { command_send, speed_rec };
        let mut zhu_li = ZhuLi { command_rec, speed_send };
        let join_handle = thread::spawn(move || zhu_li.do_the_thing(&AUTO_PARAMS));

        for i in 0..1000 {
            println!("{:?}: {:?}", i, speed);
            command_manager.command_send.send(Command::SetSpeed(speed)).unwrap();
            if let Ok(s) = command_manager.speed_rec.try_recv() {
                speed = s;
            }
        }

        command_manager.command_send.send(Command::EndAutonomous).unwrap();
        join_handle.join().unwrap();
    }
}