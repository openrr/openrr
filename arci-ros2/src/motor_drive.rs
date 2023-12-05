use std::sync::Mutex;

use arci::*;
use r2r::WrappedTypesupport;

use crate::Node;

pub struct Ros2MotorDrivePosition<T>
where
    T: WrappedTypesupport + From<f64> + 'static,
{
    position_publisher: Mutex<r2r::Publisher<T>>,
    _node: Node,
}

impl<T> Ros2MotorDrivePosition<T>
where
    T: WrappedTypesupport + From<f64> + 'static,
{
    pub fn new(node: Node, topic_name: &str) -> Self {
        let position_publisher = node
            .r2r()
            .create_publisher(topic_name, r2r::QosProfile::default())
            .unwrap();
        Self {
            position_publisher: Mutex::new(position_publisher),
            _node: node,
        }
    }
}

impl<T> MotorDrivePosition for Ros2MotorDrivePosition<T>
where
    T: WrappedTypesupport + From<f64> + 'static,
{
    fn set_motor_position(&self, position: f64) -> Result<(), Error> {
        self.position_publisher
            .lock()
            .unwrap()
            .publish(&T::from(position))
            .map_err(|e| arci::Error::Connection {
                message: format!("r2r publish error: {e:?}"),
            })
    }

    fn get_motor_position(&self) -> Result<f64, Error> {
        unimplemented!()
    }
}
