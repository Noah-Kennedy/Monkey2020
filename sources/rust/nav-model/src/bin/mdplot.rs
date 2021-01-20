use nav_algo::mdp::value_iteration::ValueIterationMDPSystem;
use nav_model::{DiscreteState, get_path, PerlinTable, RobotForecastTable, RobotVector, SPACE, PARAMS};

fn main() {
    let table = PerlinTable::new(SPACE.clone());

    let primary = RobotForecastTable::new(SPACE.clone());
    let secondary = RobotForecastTable::new(SPACE.clone());

    let sys = ValueIterationMDPSystem {
        reward_table: table.clone(),
        state_space: SPACE.clone(),
        _phantom: Default::default(),
    };

    let idx = sys.solve_forecasts(&PARAMS, &primary, &secondary);

    let arr = [primary, secondary];

    let keep = &arr[idx as usize];

    let path = get_path(&SPACE, &DiscreteState {
        position: RobotVector {
            x: 0,
            y: 0,
            r: 0,
        },
    }, &table, keep);

    table.plot_path(path);
}