use nav_model::{PerlinTable, SPACE};

fn main() {
    let table = PerlinTable::new(SPACE);

    table.plot();
}