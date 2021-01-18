use std::f32::consts::TAU;

use plotters::drawing::IntoDrawingArea;
use plotters::element::Circle;
use plotters::prelude::BitMapBackend;
use plotters::style::{BLACK, HSLColor, ShapeStyle, WHITE};

use nav_algo::{RewardTable, StateSpace};

use crate::{DiscreteState, index, RobotStateSpace, RobotVector, GOAL};

const PERLIN: usize = 8;

#[derive(Default, Debug, Clone, PartialOrd, PartialEq)]
pub struct PerlinTable {
    table: Box<[[[f32; 2]; PERLIN + 1]; PERLIN + 1]>,
    state_map: Vec<f32>,
    space: RobotStateSpace,
}

impl PerlinTable {
    pub fn new(space: RobotStateSpace) -> Self {
        let mut table = [[[0.0; 2]; PERLIN + 1]; PERLIN + 1];

        for row in table.iter_mut() {
            for cell in row {
                let theta = rand::random::<f32>() * TAU;

                cell[0] = theta.cos();
                cell[1] = theta.sin();


            }
        }

        let table = Box::new(table);

        let mut r = Self { table, state_map: vec![GOAL; space.length as usize * space.ang_res as
            usize * space.width as usize], space };
        r.generate_map();

        r
    }

    fn generate_map(&mut self) {
        for state in self.space.nonterminal_states().iter() {
            let perlin_coords = [
                state.position.x as f32 * PERLIN as f32 / self.space.width as f32,
                state.position.y as f32 * PERLIN as f32 / self.space.length as f32,
            ];

            let cell_corners = [
                [perlin_coords[0].floor(), perlin_coords[1].floor()],
                [perlin_coords[0].floor(), perlin_coords[1].floor() + 1.0],
                [perlin_coords[0].floor() + 1.0, perlin_coords[1].floor()],
                [perlin_coords[0].floor() + 1.0, perlin_coords[1].floor() + 1.0]
            ];

            let offsets = [
                [perlin_coords[0] - cell_corners[0][0], perlin_coords[1] - cell_corners[0][1]],
                [perlin_coords[0] - cell_corners[1][0], perlin_coords[1] - cell_corners[1][1]],
                [perlin_coords[0] - cell_corners[2][0], perlin_coords[1] - cell_corners[2][1]],
                [perlin_coords[0] - cell_corners[3][0], perlin_coords[1] - cell_corners[3][1]],
            ];

            let noises = [
                self.table[cell_corners[0][0] as usize][cell_corners[0][1] as usize],
                self.table[cell_corners[1][0] as usize][cell_corners[1][1] as usize],
                self.table[cell_corners[2][0] as usize][cell_corners[2][1] as usize],
                self.table[cell_corners[3][0] as usize][cell_corners[3][1] as usize],
            ];

            let dots = [
                ((offsets[0][0] * noises[0][0]) + (offsets[0][1] * noises[0][1])),
                ((offsets[1][0] * noises[1][0]) + (offsets[1][1] * noises[1][1])),
                ((offsets[2][0] * noises[2][0]) + (offsets[2][1] * noises[2][1])),
                ((offsets[3][0] * noises[3][0]) + (offsets[3][1] * noises[3][1])),
            ];

            let interp = [
                smoothstep(perlin_coords[0] - perlin_coords[0].floor(), dots[0], dots[2]),
                smoothstep(perlin_coords[0] - perlin_coords[0].floor(), dots[1], dots[3]),
            ];

            let v = smoothstep(perlin_coords[1] - perlin_coords[1].floor(), interp[0], interp[1]);

            self.state_map[index(&self.space, state)] = (v - 1.0) / 2.0
        }
    }

    pub fn plot(&self) {
        let root = BitMapBackend::new("images/perlin.png", (self.space.width as u32, self.space
            .length as u32)).into_drawing_area();

        root.fill(&WHITE).unwrap();

        for x in 0..self.space.width {
            for y in 0..self.space.length {
                let s = DiscreteState { position: RobotVector { x: x as i32, y: y as i32, r: 0 } };
                let r = self.reward(&s);
                let c = HSLColor(r.abs() as f64, 1.0, 0.5);

                root.draw_pixel((x as i32, y as i32), &c).unwrap();
            }
        }
    }

    pub fn plot_path(&self, path: Vec<DiscreteState>) {
        let root = BitMapBackend::new("images/path.png", (self.space.width as u32, self.space
            .length as
            u32)).into_drawing_area();

        root.fill(&WHITE).unwrap();

        for x in 0..self.space.width {
            for y in 0..self.space.length {
                let s = DiscreteState { position: RobotVector { x: x as i32, y: y as i32, r: 0 } };
                let r = self.reward(&s);
                let c = HSLColor(r.abs() as f64, 1.0, 0.5);

                root.draw_pixel((x as i32, y as i32), &c).unwrap();
            }
        }

        for s in path {
            let x = s.position.x;
            let y = s.position.y;

            root.draw(&Circle::new(
                (x, y),
                1,
                Into::<ShapeStyle>::into(&BLACK).filled(),
            )).unwrap();
        }
    }
}

impl RewardTable<DiscreteState> for PerlinTable {
    fn reward(&self, state: &DiscreteState) -> f32 {
        self.state_map[index(&self.space, state)]
    }
}

fn smoothstep(x: f32, low: f32, high: f32) -> f32 {
    low + (x * x * x * (x * (x * 6.0 - 15.0) + 10.0)) * (high - low)
}