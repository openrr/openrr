use arci::*;
use r2r::geometry_msgs::msg::Twist;

#[derive(Debug)]
pub struct Ros2CmdVelMoveBase {
    vel_publisher: r2r::Publisher<Twist>,
}

impl Ros2CmdVelMoveBase {
    pub fn new(node: &mut r2r::Node, cmd_topic_name: &str) -> Self {
        Self {
            vel_publisher: node.create_publisher(cmd_topic_name).unwrap(),
        }
    }
}

unsafe impl Send for Ros2CmdVelMoveBase {}
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
                message: format!("r2r publish error: {:?}", e),
            })
    }

    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        panic!("not implemented yet");
    }
}

#[derive(Debug, Clone)]
pub struct Ros2CmdVelMoveBaseConfig {
    pub topic: String,
}
