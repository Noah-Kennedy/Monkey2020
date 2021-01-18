use super::*;

const TEST_ARRAY: [i32; 8] = [451, -684, -546, 241, 859, 886, 111, 111];
const TEST_ARRAY_SORTED: [i32; 8] = [-684, -546, 111, 111, 241, 451, 859, 886];

#[test]
fn test_pairing() {
    let mut heap = PairingHeap::new();

    for x in TEST_ARRAY.iter() {
        heap.insert(*x);
    }

    for x in TEST_ARRAY_SORTED.iter() {
        let v = heap.delete_min();
        assert_eq!(*x, v.unwrap())
    }
}
