use auto_impl::auto_impl;

use crate::error::Error;

#[auto_impl(Box, Arc)]
pub trait MotorDrivePosition: Send + Sync {
    fn set_motor_position(&self, position: f64) -> Result<(), Error>;
    fn get_motor_position(&self) -> Result<f64, Error>;
}

#[auto_impl(Box, Arc)]
pub trait MotorDriveVelocity: Send + Sync {
    fn set_motor_velocity(&self, velocity: f64) -> Result<(), Error>;
    fn get_motor_velocity(&self) -> Result<f64, Error>;
}

#[auto_impl(Box, Arc)]
pub trait MotorDriveEffort: Send + Sync {
    fn set_motor_effort(&self, effort: f64) -> Result<(), Error>;
    fn get_motor_effort(&self) -> Result<f64, Error>;
}
