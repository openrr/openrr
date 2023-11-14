use std::sync::{Arc, Mutex};

use arci::MotorDriveVelocity;
use msg::{sensor_msgs::JointState, std_msgs::Float64};

use crate::msg;

#[derive(Clone)]
pub struct JointVelocityController(Arc<JointVelocityControllerInner>);

#[derive(Clone)]
struct JointVelocityControllerInner {
    publisher: rosrust::Publisher<Float64>,
    _subscriber: rosrust::Subscriber,
    motor_velocity_state: Arc<Mutex<f64>>,
    is_reverse_direction: bool,
}

impl JointVelocityController {
    pub fn new(
        velocity_topic: &str,
        joint_state_topic: &str,
        is_reverse_direction: bool,
        joint_index: usize,
    ) -> Self {
        let motor_velocity_state = Arc::new(Mutex::new(0.));
        let callback_motor_velocity_state = motor_velocity_state.clone();

        let _subscriber = rosrust::subscribe(joint_state_topic, 1, move |msg: JointState| {
            let mut vel = callback_motor_velocity_state.lock().unwrap();
            *vel = msg.velocity[joint_index];
        })
        .unwrap();

        JointVelocityController(Arc::new(JointVelocityControllerInner {
            publisher: rosrust::publish(velocity_topic, 1).unwrap(),
            _subscriber,
            motor_velocity_state,
            is_reverse_direction,
        }))
    }
}

impl MotorDriveVelocity for JointVelocityController {
    fn set_motor_velocity(&self, velocity: f64) -> Result<(), arci::Error> {
        let vel = if self.0.is_reverse_direction {
            -velocity
        } else {
            velocity
        };

        self.0
            .publisher
            .send(Float64 { data: vel })
            .map_err(|e| arci::Error::Connection {
                message: format!("Error to publish motor velocity. \"{:?}\"", e),
            })
    }

    fn get_motor_velocity(&self) -> Result<f64, arci::Error> {
        let vel = self.0.motor_velocity_state.lock().unwrap();
        Ok(if self.0.is_reverse_direction {
            -(*vel)
        } else {
            *vel
        })
    }
}
