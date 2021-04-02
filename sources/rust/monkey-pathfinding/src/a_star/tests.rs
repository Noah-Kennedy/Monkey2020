use std::iter::FromIterator;

use super::*;

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash, Debug, Default)]
struct TestState {
    x: i32,
    y: i32,
}

struct TestSpace {
    terminal: fn(TestState) -> bool
}

fn small_grid(state: TestState) -> bool {
    state.x != 0 || state.y != 0
}

fn medium_grid(state: TestState) -> bool {
    let r = -2..=2;

    !(r.contains(&state.x) && r.contains(&state.y))
}

fn large_grid(state: TestState) -> bool {
    let r = -3..=3;

    !(r.contains(&state.x) && r.contains(&state.y))
}

#[test]
fn test_no_path() {
    check_case(
        small_grid,
        TestState::default(),
        TestState { x: 2, y: 0 },
        None,
        HashSet::from_iter(vec![
            TestState { x: 1, y: 0 },
            TestState { x: 0, y: 1 },
            TestState { x: -1, y: 0 },
            TestState { x: 0, y: -1 },
        ]),
    )
}

#[test]
fn test_small() {
    check_case(
        small_grid,
        TestState::default(),
        TestState { x: 1, y: 0 },
        Some(vec![TestState { x: 1, y: 0 }]),
        HashSet::from_iter(vec![
            TestState { x: -1, y: 0 },
            TestState { x: 0, y: -1 },
            TestState { x: 1, y: 0 },
            TestState { x: 0, y: 1 },
        ]),
    )
}

#[test]
fn test_medium() {
    check_case(
        medium_grid,
        TestState::default(),
        TestState { x: 2, y: 0 },
        Some(vec![TestState { x: 1, y: 0 }, TestState { x: 2, y: 0 }]),
        HashSet::from_iter(vec![
            TestState { x: 1, y: 0 },
            TestState { x: -1, y: 0 },
            TestState { x: 2, y: 0 },
            TestState { x: 0, y: 1 },
            TestState { x: 1, y: 1 },
            TestState { x: 0, y: -1 },
            TestState { x: 1, y: -1 },
        ]),
    )
}

#[test]
fn test_large() {
    check_case(
        large_grid,
        TestState::default(),
        TestState { x: 3, y: 0 },
        Some(vec![TestState { x: 1, y: 0 }, TestState { x: 2, y: 0 }, TestState { x: 3, y: 0 }]),
        HashSet::from_iter(vec![
            TestState { x: 1, y: 1 },
            TestState { x: 0, y: 1 },
            TestState { x: -1, y: 0 },
            TestState { x: 2, y: 0 },
            TestState { x: 0, y: -1 },
            TestState { x: 3, y: 0 },
            TestState { x: 2, y: 1 },
            TestState { x: 1, y: -1 },
            TestState { x: 1, y: 0 },
            TestState { x: 2, y: -1 }
        ]),
    )
}

fn check_case(
    terminal: fn(TestState) -> bool,
    start: TestState,
    end: TestState,
    path: Option<Vec<TestState>>,
    looked_at: HashSet<TestState>,
)
{
    let mut a = AStar::new(TestSpace { terminal });
    let p = a.find_path(&start, &end);

    assert_eq!(path, p);
    assert_eq!(looked_at, a.actions.keys().map(ToOwned::to_owned).collect())
}

impl StateSpace<TestState> for TestSpace {
    fn neighbors(&self, state: TestState) -> Vec<(f32, TestState)> {
        let mut adj = Vec::new();

        if !(self.terminal)(state) {
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if (dx == 0) ^ (dy == 0) {
                        let x_new = dx + state.x as i32;
                        let y_new = dy + state.y as i32;

                        let new_state = TestState {
                            x: x_new,
                            y: y_new,
                        };

                        adj.push((1.0, new_state));
                    }
                }
            }
        }

        adj
    }

    fn heuristic(&self, state: TestState, goal: TestState) -> f32 {
        ((state.x - goal.x).pow(2) as f32 + (state.y - goal.y).pow(2) as f32).sqrt()
    }
}
