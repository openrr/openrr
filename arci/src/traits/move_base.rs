use crate::error::Error;
use auto_impl::auto_impl;

#[derive(Clone, Debug, Default, Copy)]
pub struct BaseVelocity {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl BaseVelocity {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }
}

/// Multiply scalar value for velocity
///
/// # Example
///
/// ```
/// use assert_approx_eq::assert_approx_eq;
/// use arci::BaseVelocity;
///
/// let vel = BaseVelocity::new(0.1, -0.2, 1.0);
/// let twice = vel * 2.0;
/// assert_approx_eq!(twice.x, 0.2);
/// assert_approx_eq!(twice.y, -0.4);
/// assert_approx_eq!(twice.theta, 2.0);
/// ```
impl std::ops::Mul<f64> for BaseVelocity {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            theta: self.theta * rhs,
        }
    }
}

#[auto_impl(Box, Rc, Arc)]
pub trait MoveBase {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error>;
    fn current_velocity(&self) -> Result<BaseVelocity, Error>;
}
