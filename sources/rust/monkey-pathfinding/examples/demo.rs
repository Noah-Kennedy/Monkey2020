use std::time::Instant;

use monkey_pathfinding::a_star::AStar;
use monkey_pathfinding::demo_info::{GOAL, MODEL, START};
use monkey_pathfinding::perlin::PerlinTable;
use monkey_pathfinding::state_space::MonkeyStateSpace;

fn main() {
    let timer = Instant::now();

    let table = PerlinTable::new(MODEL.clone());

    let s = MonkeyStateSpace {
        cost: table.clone(),
        model: MODEL,
    };

    let mut astar = AStar::new(s);

    let path = astar.find_path(
        &START,
        &GOAL,
    )
        .unwrap();

    table.plot_path(path);

    let elapsed = timer.elapsed().as_secs_f32();

    println!("Finished in {} seconds", elapsed);
}