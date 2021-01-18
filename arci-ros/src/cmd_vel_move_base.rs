use crate::msg;
use crate::rosrust_utils::wait_subscriber;
use arci::*;
use serde::{Deserialize, Serialize};

pub struct RosCmdVelMoveBase {
    vel_publisher: rosrust::Publisher<msg::geometry_msgs::Twist>,
}

impl RosCmdVelMoveBase {
    pub fn new(cmd_topic_name: &str) -> Self {
        let vel_publisher = rosrust::publish(cmd_topic_name, 1).unwrap();
        wait_subscriber(&vel_publisher);
        Self { vel_publisher }
    }
}

impl MoveBase for RosCmdVelMoveBase {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let mut twist_msg = msg::geometry_msgs::Twist::default();
        twist_msg.linear.x = velocity.x;
        twist_msg.linear.y = velocity.y;
        twist_msg.angular.z = velocity.theta;
        self.vel_publisher
            .send(twist_msg)
            .map_err(|e| arci::Error::Connection {
                message: format!("rosrust publish error: {:?}", e),
            })
    }
    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        panic!("not implemented yet");
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct RosCmdVelMoveBaseConfig {
    pub topic: String,
}
