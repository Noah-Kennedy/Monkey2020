use std::time::Instant;

use plotters::drawing::IntoDrawingArea;
use plotters::prelude::BitMapBackend;
use plotters::style::{BLACK, HSLColor};

use space_monkeys::mesh_to_grid;
use space_monkeys::math;

fn main() {
    let min_x = -5.0;
    let max_x = 1.0;
    let min_z = -7.0;
    let max_z = 0.0;
    let res_x = 128;
    let res_z = 128;

    let start = Instant::now();
    let grid = mesh_to_grid::mesh_to_grid("test-data/test_mesh.ply", min_x, max_x, min_z, max_z, res_x, res_z);
    let elapsed = start.elapsed();

    match grid {
        Ok(grid) => {
            let terrain_height = BitMapBackend::new("../images/terrain_height.png", (res_x as u32, res_z as u32)).into_drawing_area();
            let terrain_grad_mag = BitMapBackend::new("../images/terrain_grad_mag.png", (res_x as u32, res_z as u32)).into_drawing_area();
            terrain_height.fill(&BLACK).unwrap();
            terrain_grad_mag.fill(&BLACK).unwrap();

            for grid_z in 0..res_z {
                for grid_x in 0..res_x {
                    let height = grid.get_height_normed(grid_x, grid_z);
                    let grad_mag = grid.get_grad_mag_normed(grid_x, grid_z);

                    let colorizer = |value: f32| -> HSLColor {
                        let mut hue = math::lerp_f32(240.0 / 360.0, -60.0 / 360.0, value) as f64;
                        while hue < 0.0 { hue += 1.0; }
                        let sat = (value * value - value) as f64 + 1.0;
                        let lit = (2.0 * value as f64 - 1.0).powf(3.0) / 2.0 + 0.5;
                        return HSLColor(hue, sat, lit);
                    };

                    terrain_height.draw_pixel((grid_x as i32, grid_z as i32), &colorizer(height)).unwrap();
                    terrain_grad_mag.draw_pixel((grid_x as i32, grid_z as i32), &colorizer(grad_mag)).unwrap();
                }
            }
        }
        Err(err) => panic!("{:?}", err)
    }
    println!("Elapsed: {:?}", elapsed);
}