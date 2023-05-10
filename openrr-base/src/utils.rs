use arci::BaseVelocity;
use thiserror::Error;

#[derive(Debug, Clone, Copy)]
pub struct BaseVelocityTimestamped {
    pub base_velocity: BaseVelocity,
    pub timestamp: std::time::Instant,
}

impl BaseVelocityTimestamped {
    pub fn new_with_now(base_velocity: BaseVelocity) -> Self {
        Self {
            base_velocity,
            timestamp: std::time::Instant::now(),
        }
    }
}

impl Default for BaseVelocityTimestamped {
    fn default() -> Self {
        Self {
            base_velocity: BaseVelocity::default(),
            timestamp: std::time::Instant::now(),
        }
    }
}

pub type BaseAcceleration = BaseVelocity;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum Error {
    #[error("openrr-base: Current position is unknown")]
    CurrentPositionUnknown,
}
