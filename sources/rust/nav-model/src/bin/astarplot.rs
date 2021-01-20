use nav_algo::a_star::AStar;
use nav_model::{DiscreteState, PerlinTable, RobotAStarStateSpace, SPACE, START};
use std::time::Instant;

fn main() {
    let table = PerlinTable::new(SPACE.clone());

    let s = RobotAStarStateSpace {
        cost: table.clone(),
        space: SPACE,
    };

    let mut astar = AStar::new(s);

    let timer = Instant::now();

    let path = astar.find_path(
        &START,
        &DiscreteState { position: SPACE.goal },
    )
        .unwrap();

    let elapsed = timer.elapsed().as_secs_f32();

    println!("Finished in {} seconds", elapsed);

    table.plot_path(path);
}