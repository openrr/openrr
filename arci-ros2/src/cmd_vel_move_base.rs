use std::sync::Mutex;

use arci::*;
use r2r::geometry_msgs::msg::Twist;
use serde::{Deserialize, Serialize};

use crate::Node;

/// `arci::MoveBase` implementation for ROS2.
pub struct Ros2CmdVelMoveBase {
    vel_publisher: Mutex<r2r::Publisher<Twist>>,
    // keep not to be dropped
    _node: Node,
}

impl Ros2CmdVelMoveBase {
    /// Creates a new `Ros2CmdVelMoveBase` from geometry_msgs/Twist topic name.
    #[track_caller]
    pub fn new(node: Node, cmd_topic_name: &str) -> Self {
        let vel_publisher = node
            .r2r()
            .create_publisher(cmd_topic_name, r2r::QosProfile::default())
            .unwrap();
        Self {
            vel_publisher: Mutex::new(vel_publisher),
            _node: node,
        }
    }
}

impl MoveBase for Ros2CmdVelMoveBase {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let mut twist_msg = Twist::default();
        twist_msg.linear.x = velocity.x;
        twist_msg.linear.y = velocity.y;
        twist_msg.angular.z = velocity.theta;
        self.vel_publisher
            .lock()
            .unwrap()
            .publish(&twist_msg)
            .map_err(|e| arci::Error::Connection {
                message: format!("r2r publish error: {e:?}"),
            })
    }

    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        unimplemented!("Read from /odom in the future?");
    }
}

/// Configuration for `Ros2CmdVelMoveBase`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2CmdVelMoveBaseConfig {
    /// Topic name for geometry_msgs/Twist.
    pub topic: String,
}
