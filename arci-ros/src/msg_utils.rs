use arci::BaseVelocity;

use crate::msg::geometry_msgs::{Twist, Vector3};

impl From<BaseVelocity> for Twist {
    fn from(base_velocity: BaseVelocity) -> Self {
        Self {
            linear: Vector3 {
                x: base_velocity.x,
                y: base_velocity.y,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: base_velocity.theta,
            },
        }
    }
}

#[allow(clippy::from_over_into)]
impl Into<BaseVelocity> for Twist {
    fn into(self) -> BaseVelocity {
        BaseVelocity {
            x: self.linear.x,
            y: self.linear.y,
            theta: self.angular.z,
        }
    }
}

#[cfg(test)]
mod tests {
    use arci::BaseVelocity;
    use assert_approx_eq::assert_approx_eq;

    use super::{Twist, Vector3};

    #[test]
    fn test_base_velocity_to_twist() {
        let base_velocity = BaseVelocity {
            x: 1.0,
            y: 2.0,
            theta: 3.0,
        };
        let twist = Twist::from(base_velocity);
        assert_approx_eq!(twist.linear.x, 1.0);
        assert_approx_eq!(twist.linear.y, 2.0);
        assert_approx_eq!(twist.linear.z, 0.0);
        assert_approx_eq!(twist.angular.x, 0.0);
        assert_approx_eq!(twist.angular.y, 0.0);
        assert_approx_eq!(twist.angular.z, 3.0);
    }

    #[test]
    fn test_twist_to_base_velocity() {
        let twist = Twist {
            linear: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            angular: Vector3 {
                x: 4.0,
                y: 5.0,
                z: 6.0,
            },
        };
        let base_velocity: BaseVelocity = twist.into();
        assert_approx_eq!(base_velocity.x, 1.0);
        assert_approx_eq!(base_velocity.y, 2.0);
        assert_approx_eq!(base_velocity.theta, 6.0);
    }
}
