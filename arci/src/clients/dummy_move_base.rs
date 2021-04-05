use crate::error::Result;
use crate::traits::{BaseVelocity, MoveBase};
use std::sync::Mutex;

#[derive(Debug, Default)]
pub struct DummyMoveBase {
    pub current_velocity: Mutex<BaseVelocity>,
}

impl DummyMoveBase {
    pub fn new() -> Self {
        Self {
            current_velocity: Mutex::new(BaseVelocity::default()),
        }
    }
}

impl MoveBase for DummyMoveBase {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<()> {
        *self.current_velocity.lock().unwrap() = *velocity;
        Ok(())
    }
    fn current_velocity(&self) -> Result<BaseVelocity> {
        Ok(self.current_velocity.lock().unwrap().to_owned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    #[test]
    fn test_set_get() {
        let base = DummyMoveBase::new();
        let vel = base.current_velocity().unwrap();
        assert_approx_eq!(vel.x, 0.0);
        assert_approx_eq!(vel.y, 0.0);
        assert_approx_eq!(vel.theta, 0.0);
        base.send_velocity(&BaseVelocity::new(0.1, 0.2, -3.0))
            .unwrap();
        let vel2 = base.current_velocity().unwrap();
        assert_approx_eq!(vel2.x, 0.1);
        assert_approx_eq!(vel2.y, 0.2);
        assert_approx_eq!(vel2.theta, -3.0);
    }
}
