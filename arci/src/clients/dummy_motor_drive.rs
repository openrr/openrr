use std::sync::Mutex;

use crate::{
    error::Error,
    traits::{MotorDriveEffort, MotorDrivePosition, MotorDriveVelocity},
};

#[derive(Debug)]
pub struct DummyMotorDrivePosition {
    pub current_position: Mutex<f64>,
}

impl DummyMotorDrivePosition {
    pub fn new() -> Self {
        Self {
            current_position: Mutex::new(0f64),
        }
    }
}

impl Default for DummyMotorDrivePosition {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDrivePosition for DummyMotorDrivePosition {
    fn set_motor_position(&self, position: f64) -> Result<(), Error> {
        *self.current_position.lock().unwrap() = position;
        Ok(())
    }

    fn get_motor_position(&self) -> Result<f64, Error> {
        Ok(*self.current_position.lock().unwrap())
    }
}

#[derive(Debug)]
pub struct DummyMotorDriveVelocity {
    pub current_velocity: Mutex<f64>,
}

impl DummyMotorDriveVelocity {
    pub fn new() -> Self {
        Self {
            current_velocity: Mutex::new(0f64),
        }
    }
}

impl Default for DummyMotorDriveVelocity {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDriveVelocity for DummyMotorDriveVelocity {
    fn set_motor_velocity(&self, velocity: f64) -> Result<(), Error> {
        *self.current_velocity.lock().unwrap() = velocity;
        Ok(())
    }

    fn get_motor_velocity(&self) -> Result<f64, Error> {
        Ok(*self.current_velocity.lock().unwrap())
    }
}

#[derive(Debug)]
pub struct DummyMotorDriveEffort {
    pub current_effort: Mutex<f64>,
}

impl DummyMotorDriveEffort {
    pub fn new() -> Self {
        Self {
            current_effort: Mutex::new(0f64),
        }
    }
}

impl Default for DummyMotorDriveEffort {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDriveEffort for DummyMotorDriveEffort {
    fn set_motor_effort(&self, effort: f64) -> Result<(), Error> {
        *self.current_effort.lock().unwrap() = effort;
        Ok(())
    }

    fn get_motor_effort(&self) -> Result<f64, Error> {
        Ok(*self.current_effort.lock().unwrap())
    }
}
