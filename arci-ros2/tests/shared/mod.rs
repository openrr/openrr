use std::sync::atomic::{AtomicUsize, Ordering};

use arci_ros2::*;

pub fn test_node() -> Node {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    let node_name = format!("test_arci_ros2_node_{n}");
    Node::new(&node_name, "arci_ros2_test").unwrap()
}
