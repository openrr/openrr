use std::{
    sync::{Arc, Mutex, RwLock},
    time::Duration,
};

use arci::*;
use r2r::{geometry_msgs::msg::Twist, nav_msgs::msg::Odometry};
use serde::{Deserialize, Serialize};

use crate::{utils, Node};

/// `arci::MoveBase` implementation for ROS2.
pub struct Ros2CmdVelMoveBase {
    vel_publisher: Mutex<r2r::Publisher<Twist>>,
    odom: Arc<RwLock<Option<Odometry>>>,
    odometry_topic_name: String,
    // keep not to be dropped
    _node: Node,
}

impl Ros2CmdVelMoveBase {
    /// Creates a new `Ros2CmdVelMoveBase` from geometry_msgs/Twist topic name.
    #[track_caller]
    pub fn new(node: Node, cmd_topic_name: &str, odometry_topic_name: &str) -> Self {
        let vel_publisher = node
            .r2r()
            .create_publisher(cmd_topic_name, r2r::QosProfile::default())
            .unwrap();

        let mut odom_subscriber = node
            .r2r()
            .subscribe::<Odometry>(odometry_topic_name, r2r::QosProfile::default())
            .unwrap();
        let odom = utils::subscribe_one(&mut odom_subscriber, Duration::from_secs(1));
        let odom = Arc::new(RwLock::new(odom));
        utils::subscribe_thread(odom_subscriber, odom.clone(), Some);

        Self {
            vel_publisher: Mutex::new(vel_publisher),
            odom,
            odometry_topic_name: odometry_topic_name.to_string(),
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
        let subscribed_odom = self.odom.read().unwrap();
        let current_velocity = match &*subscribed_odom {
            Some(msg) => BaseVelocity {
                x: msg.twist.twist.linear.x,
                y: msg.twist.twist.linear.y,
                theta: msg.twist.twist.angular.z,
            },
            None => {
                return Err(Error::Connection {
                    message: format!("Failed to get odom from {}", self.odometry_topic_name),
                });
            }
        };
        Ok(current_velocity)
    }
}

/// Configuration for `Ros2CmdVelMoveBase`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ros2CmdVelMoveBaseConfig {
    /// Topic name for geometry_msgs/Twist.
    pub cmd_vel_topic: String,
    /// Topic name for nav_msgs/Odometry.
    pub odom_topic: String,
}
