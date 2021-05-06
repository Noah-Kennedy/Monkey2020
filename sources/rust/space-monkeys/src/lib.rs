pub mod math;
pub mod mesh_to_grid;
mod aimbot;

fn control_loop() {
    // Some pseudocode

    // Upon entering autonomous mode
    // begin_logging();
    //
    // Any prework that can be done independently of other subsystems happens here.
    // comms.init();
    // visual_processing.init();
    // navigation.init();
    //
    // let keep_going = true;
    // let target_location = None;
    // let path = None;
    // let abort = || => {
    //     comms.stop_motion();
    //     target_location = None;
    //     path = None;
    //     keep_going = false;
    // };
    //
    // while keep_going {
    //     if target_location.is_some() {
    //         // Get latest spatial map.
    //         let triangle_mesh = visual_processing.get_triangle_mesh();
    //
    //         // Update grid representation of terrain.
    //         navigation.mesh_to_grid(triangle_mesh);
    //
    //         if path.is_none() || !navigation.path_is_safe(path) {
    //             path = navigation.generate_path();
    //         }
    //
    //         if path.is_none() {
    //             abort();
    //         } else {
    //             let current_location = visual_processing.get_current_location();
    //
    //             // Instruct robot to follow the path provided.
    //             follow_path(path, current_location);
    //         }
    //     }
    //
    //     match comms.receive_command() {
    //         Abort => abort(),
    //         SetTarget(target) => target_location = target,
    //         // etc.
    //     }
    // }
}
