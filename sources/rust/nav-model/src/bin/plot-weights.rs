use nav_algo::mdp::value_iteration::{ForecastTable, ValueIterationMDPSystem};
use nav_model::{PerlinTable, RobotForecastTable, SPACE, PARAMS};

fn main() {
    let table = PerlinTable::new(SPACE.clone());

    let primary = RobotForecastTable::new(SPACE.clone());
    let secondary = RobotForecastTable::new(SPACE.clone());

    let sys = ValueIterationMDPSystem {
        reward_table: table,
        state_space: SPACE.clone(),
        _phantom: Default::default(),
    };

    let idx = sys.solve_forecasts(&PARAMS, &primary, &secondary);

    let arr = [primary, secondary];

    let keep = &arr[idx as usize];

    let guard = keep.read();

    guard.plot_forecast()
}