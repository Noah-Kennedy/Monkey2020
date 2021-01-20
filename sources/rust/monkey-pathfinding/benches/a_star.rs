#[macro_use]
extern crate criterion;

use criterion::{AxisScale, BatchSize, BenchmarkId, black_box, Criterion, PlotConfiguration, Throughput};

use monkey_pathfinding::a_star::AStar;
use monkey_pathfinding::model::{DiscreteState, MonkeyModel, RobotVector};
use monkey_pathfinding::perlin::PerlinTable;
use monkey_pathfinding::state_space::MonkeyStateSpace;

pub const MODEL: MonkeyModel = MonkeyModel {
    length: 160,
    width: 80,
    ang_res: 60,
    max_omega: 5,
    min_speed: 3,
    max_speed: 5,
    rev_min_speed: 0,
    rev_max_speed: 0,
};

fn by_cartesian_size(c: &mut Criterion) {
    let mut group = c.benchmark_group("Size");

    group.plot_config(PlotConfiguration::default()
        .summary_scale(AxisScale::Logarithmic));

    group.sample_size(10);

    for size in (10..=100).step_by(10) {
        let model = MonkeyModel {
            length: size,
            width: size,
            ..MODEL
        };

        group.throughput(Throughput::Elements(size as u64 * size as u64));

        group.bench_with_input(
            BenchmarkId::from_parameter(size * size),
            &size,
            |b, _| {
                b.iter_batched(
                    || {
                        let table = PerlinTable::new(model.clone());
                        let space = MonkeyStateSpace {
                            cost: table,
                            model: model.clone(),
                        };

                        let start = DiscreteState::default();

                        let end = DiscreteState {
                            position: RobotVector {
                                x: space.model.width as i32 - 1,
                                y: space.model.length as i32 - 1,
                                ..Default::default()
                            }
                        };

                        (AStar::new(space), start, end)
                    },
                    |(mut a_star, start, end)| {
                        let path = a_star.find_path(&start, &end);
                        black_box(path);
                    },
                    BatchSize::SmallInput,
                );
            },
        );
    }

    group.finish();
}

fn by_velocity(c: &mut Criterion) {
    let mut group = c.benchmark_group("Velocity");

    group.plot_config(PlotConfiguration::default()
        .summary_scale(AxisScale::Linear));

    group.sample_size(10);

    for speed in 1..=10 {
        let model = MonkeyModel {
            max_speed: speed,
            min_speed: 1,
            ..MODEL
        };

        group.throughput(Throughput::Elements(speed as u64));

        group.bench_with_input(
            BenchmarkId::from_parameter(speed),
            &speed,
            |b, _| {
                b.iter_batched(
                    || {
                        let table = PerlinTable::new(model.clone());
                        let space = MonkeyStateSpace {
                            cost: table,
                            model: model.clone(),
                        };

                        let start = DiscreteState::default();

                        let end = DiscreteState {
                            position: RobotVector {
                                x: space.model.width as i32 - 1,
                                y: space.model.length as i32 - 1,
                                ..Default::default()
                            }
                        };

                        (AStar::new(space), start, end)
                    },
                    |(mut a_star, start, end)| {
                        let path = a_star.find_path(&start, &end);
                        black_box(path);
                    },
                    BatchSize::SmallInput,
                );
            },
        );
    }

    group.finish();
}

criterion_group!(benches, by_cartesian_size, by_velocity);
criterion_main!(benches);