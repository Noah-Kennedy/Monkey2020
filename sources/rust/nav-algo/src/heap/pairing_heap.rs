type PairingHeapNode<T> = Option<PairingTree<T>>;

struct PairingTree<T> {
    element: T,
    children: Vec<PairingHeapNode<T>>,
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

    pub fn meld(self, other: Self) -> Self {
        let root = meld_nodes(self.root, other.root);
        Self { root }
    }

    pub fn insert(self, element: T) -> Self {
        let root = meld_nodes(Some(PairingTree { element, children: Vec::new() }), self.root);
        Self { root }
    }

    pub fn delete_min(self) -> (Option<T>, Self) {
        if let Some(root) = self.root {
            (Some(root.element), Self::merge_pairs(root.children))
        } else {
            (None, Self { root: None })
        }
    }

    fn merge_pairs(mut list: Vec<PairingHeapNode<T>>) -> Self {
        let root = if let Some(l0) = list.pop() {
            if let Some(l1) = list.pop() {
                meld_nodes(
                    meld_nodes(l0, l1),
                    Self::merge_pairs(list).root,
                )
            } else {
                l0
            }
        } else {
            PairingHeapNode::None
        };

        Self { root }
    }
}

fn meld_nodes<T: Ord>(heap1: PairingHeapNode<T>, heap2: PairingHeapNode<T>) -> PairingHeapNode<T> {
    match (heap1, heap2) {
        (root, None) => {
            root
        }
        (None, root) => {
            root
        }
        (Some(mut child1), Some(mut child2)) => {
            Some(
                if child1.element < child2.element {
                    child1.children.insert(0, Some(child2));
                    child1
                } else {
                    child2.children.insert(0, Some(child1));
                    child2
                }
            )
        }
    }
}