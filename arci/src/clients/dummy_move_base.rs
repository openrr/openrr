use crate::error::Error;
use crate::traits::{BaseVelocity, MoveBase};
use std::cell::RefCell;

#[derive(Clone, Debug, Default)]
pub struct DummyMoveBase {
    pub current_velocity: RefCell<BaseVelocity>,
}

impl DummyMoveBase {
    pub fn new() -> Self {
        Self {
            current_velocity: RefCell::new(BaseVelocity::default()),
        }
    }
}

impl MoveBase for DummyMoveBase {
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        self.current_velocity.replace(velocity.to_owned());
        Ok(())
    }
    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        Ok(self.current_velocity.borrow().to_owned())
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
