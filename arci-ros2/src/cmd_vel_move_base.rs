use std::sync::Mutex;

use arci::*;
use serde::{Deserialize, Serialize};

use crate::{msg::geometry_msgs::Twist, Node};

/// `arci::MoveBase` implementation for ROS2.
pub struct Ros2CmdVelMoveBase {
    vel_publisher: Mutex<ros2_client::Publisher<Twist>>,
    // keep not to be dropped
    _node: Node,
}

impl Ros2CmdVelMoveBase {
    /// Creates a new `Ros2CmdVelMoveBase` from geometry_msgs/Twist topic name.
    #[track_caller]
    pub fn new(node: Node, cmd_vel_topic_name: &str) -> Self {
        let cmd_vel_topic = node.create_topic::<Twist>(cmd_vel_topic_name).unwrap();
        let vel_publisher = node.create_publisher(&cmd_vel_topic).unwrap();
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
            .publish(twist_msg)
            .map_err(|e| arci::Error::Connection {
                message: format!("ros2_client publish error: {e:?}"),
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
