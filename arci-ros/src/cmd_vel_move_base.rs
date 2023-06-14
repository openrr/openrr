use arci::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{msg, rosrust_utils::wait_subscriber};

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
        self.vel_publisher
            .send((*velocity).into())
            .map_err(|e| arci::Error::Connection {
                message: format!("rosrust publish error: {e:?}"),
            })
    }

    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        Err(Error::Other(anyhow::Error::msg("not implemented yet")))
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RosCmdVelMoveBaseConfig {
    pub topic: String,
}
