use nav_mdp::{RobotStateSpace, PerlinTable};

fn main() {
    let space = RobotStateSpace {
        length: 2000,
        width: 1000,
        max_pos_r: 60,
        turn_rate: 0,
        max_vel_r: 5,
        max_vel_p: 50,
        noise: 0.2,
        goal: Default::default()
    };

    let table = PerlinTable::new(space.clone());

    table.plot();
}