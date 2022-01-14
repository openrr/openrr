use arci::*;
use r2r::geometry_msgs::msg::Twist;
use serde::{Deserialize, Serialize};

/// Implement arci::MoveBase for ROS2
pub struct Ros2CmdVelMoveBase {
    vel_publisher: r2r::Publisher<Twist>,
    // keep not to be dropped
    _node: r2r::Node,
}

impl Ros2CmdVelMoveBase {
    /// Creates a new `Ros2CmdVelMoveBase` from ROS2 context and Twist topic name.
    #[track_caller]
    pub fn new(ctx: r2r::Context, cmd_topic_name: &str) -> Self {
        // TODO: Consider using unique name
        let node = r2r::Node::create(ctx, "cmd_vel_node", "arci_ros2").unwrap();
        Self::from_node(node, cmd_topic_name)
    }

    /// Creates a new `Ros2CmdVelMoveBase` from ROS2 node and Twist topic name.
    #[track_caller]
    pub fn from_node(mut node: r2r::Node, cmd_topic_name: &str) -> Self {
        Self {
            vel_publisher: node.create_publisher(cmd_topic_name).unwrap(),
            _node: node,
        }
    }
}

// TODO:
unsafe impl Sync for Ros2CmdVelMoveBase {}

impl MoveBase for Ros2CmdVelMoveBase {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let mut twist_msg = Twist::default();
        twist_msg.linear.x = velocity.x;
        twist_msg.linear.y = velocity.y;
        twist_msg.angular.z = velocity.theta;
        self.vel_publisher
            .publish(&twist_msg)
            .map_err(|e| arci::Error::Connection {
                message: format!("r2r publish error: {e:?}"),
            })
    }

    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        unimplemented!("Read from /odom in the future?");
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
/// Config for Ros2CmdVelMoveBaseConfig
pub struct Ros2CmdVelMoveBaseConfig {
    /// topic name for Twist
    pub topic: String,
}
