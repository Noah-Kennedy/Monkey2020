use std::mem::swap;
use std::collections::LinkedList;

type PairingHeapNode<T> = Option<PairingTree<T>>;

#[derive(Clone, Ord, PartialOrd, Eq, PartialEq, Hash, Debug)]
struct PairingTree<T> {
    element: T,
    children: LinkedList<PairingHeapNode<T>>,
}

pub struct PairingHeap<T> {
    root: PairingHeapNode<T>,
}

impl<T> Default for PairingHeap<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> PairingHeap<T> {
    pub fn new() -> Self {
        Self {
            root: PairingHeapNode::None
        }
    }
}

impl<T> PairingHeap<T> where T: Ord + PartialOrd + PartialEq {
    pub fn find_min(&self) -> Option<&T> {
        match &self.root {
            None => None,
            Some(child) => Some(&child.element),
        }
    }

    pub fn insert(&mut self, element: T) {
        self.meld(PairingHeap { root: Some(PairingTree { element, children: LinkedList::new() }) })
    }

    pub fn delete_min(&mut self) -> Option<T> {
        if let Some(root) = &mut self.root {
            let mut new_one = Self::merge_pairs(&mut root.children);
            swap(self, &mut new_one);
            new_one.root.map(|x| x.element)
        } else {
            self.root = None;
            None
        }
    }

    fn merge_pairs(list: &mut LinkedList<PairingHeapNode<T>>) -> Self {
        if let Some(l0) = list.pop_back() {
            if let Some(l1) = list.pop_back() {
                let mut h0 = Self { root: l0 };
                let h1 = Self { root: l1 };

                h0.meld(h1);
                let merged = Self::merge_pairs(list);
                h0.meld(merged);

                h0
            } else {
                Self { root: l0 }
            }
        } else {
            Self::new()
        }
    }

    fn meld(&mut self, other: Self) {
        if let Some(root1) = &mut self.root {
            if let Some(root2) = other.root {
                if root1.element < root2.element {
                    root1.children.push_front(Some(root2))
                } else {
                    let mut c1 = LinkedList::new();

                    let mut c2 = root2.children;
                    let mut e2 = root2.element;

                    swap(&mut root1.children, &mut c1);
                    swap(&mut e2, &mut root1.element);

                    let ins = PairingTree {
                        element: e2,
                        children: c1,
                    };

                    c2.push_front(Some(ins));

                    root1.children = c2;
                }
            }
        } else {
            self.root = other.root
        }
    }
}