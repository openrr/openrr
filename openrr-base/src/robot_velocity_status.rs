use arci::BaseVelocity;
use parking_lot::Mutex;

use crate::*;

#[derive(Debug, Default)]
pub struct RobotVelocityStatus {
    velocity: Mutex<BaseVelocity>,
    velocity_state: Mutex<BaseVelocity>,
    max_velocity: BaseVelocity,
    min_velocity: BaseVelocity,
    max_acceleration: BaseAcceleration,
    min_acceleration: BaseAcceleration,
    last_sent_log: Mutex<BaseVelocityTimestamped>,
}

impl RobotVelocityStatus {
    pub fn new(max_velocity: BaseVelocity, max_acceleration: BaseAcceleration) -> Self {
        Self::new_as_asymmetric(
            max_velocity,
            max_velocity * (-1.),
            max_acceleration,
            max_acceleration * (-1.),
        )
    }

    pub fn new_as_asymmetric(
        max_velocity: BaseVelocity,
        min_velocity: BaseVelocity,
        max_acceleration: BaseAcceleration,
        min_acceleration: BaseAcceleration,
    ) -> Self {
        Self {
            velocity: Mutex::new(BaseVelocity::default()),
            velocity_state: Mutex::new(BaseVelocity::default()),
            max_velocity,
            min_velocity,
            max_acceleration,
            min_acceleration,
            last_sent_log: Mutex::new(BaseVelocityTimestamped::default()),
        }
    }

    pub fn velocity(&self) -> BaseVelocity {
        self.velocity.lock().to_owned()
    }

    pub fn set_velocity(&self, velocity: &BaseVelocity) {
        let mut vel = self.velocity.lock();
        *vel = *velocity;
    }

    pub fn time_since_last_sent(&self) -> f64 {
        self.last_sent_log
            .lock()
            .to_owned()
            .timestamp
            .elapsed()
            .as_secs_f64()
    }

    pub fn set_log(&self, velocity_log: &BaseVelocity) {
        let mut log = self.last_sent_log.lock();
        *log = BaseVelocityTimestamped {
            base_velocity: *velocity_log,
            timestamp: std::time::Instant::now(),
        }
    }

    pub fn set_velocity_state(&self, velocity: BaseVelocity) {
        let mut velocity_state = self.velocity_state.lock();
        *velocity_state = velocity;
    }

    pub fn get_limited_velocity(&self, velocity: &BaseVelocity) -> BaseVelocity {
        let vel = self.limit_acceleration(velocity);
        self.limit_velocity(&vel)
    }

    fn limit_velocity(&self, velocity: &BaseVelocity) -> BaseVelocity {
        BaseVelocity {
            x: velocity.x.clamp(self.min_velocity.x, self.max_velocity.x),
            y: velocity.y.clamp(self.min_velocity.y, self.max_velocity.y),
            theta: velocity
                .theta
                .clamp(self.min_velocity.theta, self.max_velocity.theta),
        }
    }

    fn limit_acceleration(&self, velocity: &BaseVelocity) -> BaseVelocity {
        let dt = self.time_since_last_sent();
        let vel = self.velocity.lock().to_owned();
        BaseVelocity {
            x: velocity.x.clamp(
                vel.x + self.min_acceleration.x * dt,
                vel.x + self.max_acceleration.x * dt,
            ),
            y: velocity.y.clamp(
                vel.y + self.min_acceleration.y * dt,
                vel.y + self.max_acceleration.y * dt,
            ),
            theta: velocity.theta.clamp(
                vel.theta + self.min_acceleration.theta * dt,
                vel.theta + self.max_acceleration.theta * dt,
            ),
        }
    }
}

#[cfg(test)]
mod test {
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    const LIMIT_VEL_X: f64 = 10.0;
    const LIMIT_VEL_THETA: f64 = 10.0;
    const LIMIT_ACC_X: f64 = 1000.0;
    const LIMIT_ACC_THETA: f64 = 1000.0;
    const DUMMY_VELOCITY_STATE_X: f64 = 1.23;

    #[test]
    fn test_robot_velocity_status() {
        let limit_velocity = BaseVelocity::new(LIMIT_VEL_X, 0.0, LIMIT_VEL_THETA);
        let limit_acceleration = BaseAcceleration::new(LIMIT_ACC_X, 0.0, LIMIT_ACC_THETA);

        let status = RobotVelocityStatus::new(limit_velocity, limit_acceleration);

        status.set_velocity(&BaseVelocity {
            x: LIMIT_VEL_X,
            y: 0.,
            theta: -LIMIT_VEL_THETA,
        });

        let vel = status.velocity();
        assert_approx_eq!(vel.x, LIMIT_VEL_X);

        std::thread::sleep(std::time::Duration::from_millis(100));
        let cmd_vel = BaseVelocity {
            x: -20.,
            y: 0.,
            theta: 20.,
        };
        let limited = status.get_limited_velocity(&cmd_vel);

        assert_approx_eq!(limited.x, -LIMIT_VEL_X);
        assert_approx_eq!(limited.theta, LIMIT_VEL_THETA);

        let velocity_state = BaseVelocity {
            x: DUMMY_VELOCITY_STATE_X,
            y: 0.,
            theta: 0.,
        };

        status.set_velocity_state(velocity_state);

        let velocity_state_x = status.velocity_state.lock().x;
        assert_approx_eq!(velocity_state_x, DUMMY_VELOCITY_STATE_X);
    }
}
