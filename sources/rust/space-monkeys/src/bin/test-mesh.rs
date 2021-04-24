use std::time::Instant;

use plotters::drawing::IntoDrawingArea;
use plotters::prelude::BitMapBackend;
use plotters::style::{BLACK, HSLColor};

use space_monkeys::{lerp_f32, mesh_to_grid};

fn main() {
    let min_x = -5.0;
    let max_x = 1.0;
    let min_z = -7.0;
    let max_z = 0.0;
    let res_x = 128;
    let res_z = 128;

    let start = Instant::now();
    let grid = mesh_to_grid("test-data/test_mesh.ply", min_x, max_x, min_z, max_z, res_x, res_z);
    let elapsed = start.elapsed();

    match grid {
        Ok(grid) => {
            let root = BitMapBackend::new("../images/terrain.png", (res_x as u32, res_z as u32)).into_drawing_area();
            root.fill(&BLACK).unwrap();

            for grid_z in 0..res_z {
                for grid_x in 0..res_x {
                    let height = grid.get_relative(grid_x, grid_z);
                    let mut hue = lerp_f32(240.0 / 360.0, -60.0 / 360.0, height) as f64;
                    while hue < 0.0 { hue += 1.0; }
                    let sat = (height * height - height) as f64 + 1.0;
                    let lit = (2.0 * height as f64 - 1.0).powf(3.0) / 2.0 + 0.5;
                    let c = HSLColor(hue, sat, lit);
                    root.draw_pixel((grid_x as i32, grid_z as i32), &c).unwrap();
                }
            }
        }
        Err(err) => panic!("{:?}", err)
    }
    println!("Elapsed: {:?}", elapsed);
}