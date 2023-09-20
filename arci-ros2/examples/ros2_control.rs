#[cfg(feature = "ros2")]
#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use std::time::Duration;

    use arci::*;
    use arci_ros2::{Node, Ros2ControlClient};

    let node = Node::new("ros2_control_node", "arci_ros2").unwrap();
    // node.run_spin_thread(Duration::from_millis(100));
    // let client = Ros2ControlClient::new(node, "/position_trajectory_controller");
    // dbg!(client.joint_names()); // => ["joint1", "joint2"]
    // dbg!(client.current_joint_positions()).unwrap();
    // client
    //     .send_joint_positions(
    //         vec![1.0; client.joint_names().len()],
    //         Duration::from_secs(5),
    //     )?
    //     .await?;
    // dbg!(client.current_joint_positions()).unwrap();
    // client
    //     .send_joint_positions(
    //         vec![0.5; client.joint_names().len()],
    //         Duration::from_secs(5),
    //     )?
    //     .await?;
    // dbg!(client.current_joint_positions()).unwrap();

    let client = Ros2ForwardPositionController::new(node, "/forward_position_controller/commands");
    let msg = Float64MultiArray {
        layout: Default::default(),
        data: vec![0.5, 0.5],
    };
    client
        .publisher
        .lock()
        .publish(&msg)
        .map_err(|e| arci::Error::Connection {
            message: format!("r2r publish error: {e:?}"),
        })?;
    // ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
    // - 0.5
    // - 0.5"

    Ok(())
}

#[cfg(not(feature = "ros2"))]
fn main() {
    println!("This example requires ros2 feature");
}

use arci::*;
use arci_ros2::Node;
use parking_lot::Mutex;
use r2r::std_msgs::msg::Float64MultiArray;
use serde::{Deserialize, Serialize};

/// Implement arci::MoveBase for ROS2
pub struct Ros2ForwardPositionController {
    publisher: Mutex<r2r::Publisher<Float64MultiArray>>,
    // keep not to be dropped
    _node: Node,
}

impl Ros2ForwardPositionController {
    /// Creates a new `Ros2ForwardPositionController` from ROS2 node and topic name.
    #[track_caller]
    pub fn new(node: Node, topic_name: &str) -> Self {
        let publisher = node
            .r2r()
            .create_publisher(topic_name, r2r::QosProfile::default())
            .unwrap();
        Self {
            publisher: Mutex::new(publisher),
            _node: node,
        }
    }
}
