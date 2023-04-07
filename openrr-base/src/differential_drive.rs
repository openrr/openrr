use arci::{BaseVelocity, Error, MotorDriveVelocity, MoveBase};
use parking_lot::Mutex;

use crate::utils::*;

#[derive(Debug, Clone)]
pub struct DifferentialDriveHardwareParameters {
    pub wheel_radius: f64,
    pub tread_width: f64,
}

#[derive(Debug, Clone)]
pub struct DifferentialDriveMotorController<MV: MotorDriveVelocity> {
    pub left: MV,
    pub right: MV,
}

#[derive(Debug)]
pub struct DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    velocity: Mutex<BaseVelocity>,
    feedback_velocity: Mutex<BaseVelocity>,
    limit_velocity: BaseVelocity,
    limit_acceleration: BaseAcceleration,
    controller: DifferentialDriveMotorController<MV>,
    param: DifferentialDriveHardwareParameters,
    last_sent_log: Mutex<BaseVelocityTimestamped>,
}

impl<MV> DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    pub fn new(
        param: DifferentialDriveHardwareParameters,
        controller: DifferentialDriveMotorController<MV>,
        limit_velocity: BaseVelocity,
        limit_acceleration: BaseAcceleration,
    ) -> Self {
        Self {
            velocity: Mutex::new(BaseVelocity::default()),
            feedback_velocity: Mutex::new(BaseVelocity::default()),
            limit_velocity,
            limit_acceleration,
            controller,
            param,
            last_sent_log: Mutex::new(BaseVelocityTimestamped {
                base_velocity: BaseVelocity::default(),
                timestamp: std::time::Instant::now(),
            }),
        }
    }
}

impl<MV> MoveBase for DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let delta = self.last_sent_log.lock().timestamp.elapsed().as_secs_f64();

        let limited = self.acceleration_limiter(velocity, &self.velocity.lock(), delta);

        // Set command velocity
        let mut vel = self.velocity.lock();
        let limited = self.velocity_limiter(&limited);
        *vel = limited;

        let wheels_vel = self.transform_velocity_base_to_wheel(&limited);

        self.controller
            .left
            .set_motor_velocity(wheels_vel[0])
            .unwrap();
        self.controller
            .right
            .set_motor_velocity(wheels_vel[1])
            .unwrap();

        let timestamp = std::time::Instant::now();

        let mut log = self.last_sent_log.lock();
        *log = BaseVelocityTimestamped {
            base_velocity: limited,
            timestamp,
        };

        Ok(())
    }

    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        let left_wheel_rotation = self.controller.left.get_motor_velocity().unwrap();
        let right_wheel_rotation = self.controller.right.get_motor_velocity().unwrap();

        let mut fdb_vel = self.feedback_velocity.lock();
        *fdb_vel =
            self.transform_velocity_wheel_to_base(&[left_wheel_rotation, right_wheel_rotation]);

        Ok(*fdb_vel)
    }
}

impl<MV> Limiter for DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    fn velocity_limiter(&self, velocity: &BaseVelocity) -> BaseVelocity {
        BaseVelocity {
            x: velocity
                .x
                .clamp(-self.limit_velocity.x, self.limit_velocity.x),
            y: velocity
                .y
                .clamp(-self.limit_velocity.y, self.limit_velocity.y),
            theta: velocity
                .theta
                .clamp(-self.limit_velocity.theta, self.limit_velocity.theta),
        }
    }

    fn acceleration_limiter(
        &self,
        current_velocity: &BaseVelocity,
        last_velocity: &BaseVelocity,
        delta: f64,
    ) -> BaseVelocity {
        BaseVelocity {
            x: current_velocity.x.clamp(
                last_velocity.x - self.limit_acceleration.x * delta,
                last_velocity.x + self.limit_acceleration.x * delta,
            ),
            y: current_velocity.y.clamp(
                last_velocity.y - self.limit_acceleration.y * delta,
                last_velocity.y + self.limit_acceleration.y * delta,
            ),
            theta: current_velocity.theta.clamp(
                last_velocity.theta - self.limit_acceleration.theta * delta,
                last_velocity.theta + self.limit_acceleration.theta * delta,
            ),
        }
    }
}

impl<MV> VelocityTransformer for DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    /// Output: left_wheel_velocity, right_wheel_velocity
    fn transform_velocity_base_to_wheel(&self, velocity: &BaseVelocity) -> Vec<f64> {
        let wheels_vel = vec![
            (velocity.x - 0.5 * self.param.tread_width * velocity.theta) / self.param.wheel_radius,
            (velocity.x + 0.5 * self.param.tread_width * velocity.theta) / self.param.wheel_radius,
        ];

        wheels_vel
    }

    /// Input: left_wheel_velocity, right_wheel_velocity
    fn transform_velocity_wheel_to_base(&self, wheels_vel: &[f64]) -> BaseVelocity {
        let translation = 0.5 * self.param.wheel_radius * (wheels_vel[0] + wheels_vel[1]);
        let rotation =
            self.param.wheel_radius * (wheels_vel[1] - wheels_vel[0]) / self.param.tread_width;
        BaseVelocity {
            x: translation,
            y: 0f64,
            theta: rotation,
        }
    }
}

#[cfg(test)]
mod test {
    use arci::DummyMotorDriveVelocity;
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    const LIMIT_VEL_X: f64 = 10.0;
    const LIMIT_VEL_THETA: f64 = 10.0;
    const LIMIT_ACC_X: f64 = 10.0;
    const LIMIT_ACC_THETA: f64 = 10.0;

    #[test]
    fn test_limiter() {
        let dd = diff_drive_builder();

        let x_1 = 1.0;
        let theta_1 = -2.0;
        let x_2 = 11.0;
        let theta_2 = -12.0;

        let v1 = BaseVelocity {
            x: x_1,
            y: 0.0,
            theta: theta_1,
        };
        let v2 = BaseVelocity {
            x: x_2,
            y: 0.0,
            theta: theta_2,
        };

        let vel_limited1 = dd.velocity_limiter(&v1);
        let vel_limited2 = dd.velocity_limiter(&v2);

        assert_approx_eq!(vel_limited1.x, x_1);
        assert_approx_eq!(vel_limited1.theta, theta_1);
        assert_approx_eq!(vel_limited2.x, LIMIT_VEL_X);
        assert_approx_eq!(vel_limited2.theta, -LIMIT_VEL_THETA);

        let delta = 0.1;

        let current_velocity1 = BaseVelocity {
            x: 1.0,
            y: 0.0,
            theta: 1.0,
        };
        let current_velocity2 = BaseVelocity {
            x: 10.0,
            y: 0.0,
            theta: 10.0,
        };
        let last_velocity = BaseVelocity {
            x: 0.1,
            y: 0.0,
            theta: 0.1,
        };

        let acc_limited1 = dd.acceleration_limiter(&current_velocity1, &last_velocity, delta);
        let acc_limited2 = dd.acceleration_limiter(&current_velocity2, &last_velocity, delta);

        assert_approx_eq!(acc_limited1.x, 1.0);
        assert_approx_eq!(acc_limited1.theta, 1.0);
        assert_approx_eq!(acc_limited2.x, 1.1);
        assert_approx_eq!(acc_limited2.theta, 1.1);
    }

    #[test]
    fn test_velocity_transformer() {
        let dd = diff_drive_builder();

        let straight_vel = BaseVelocity {
            x: 1.0,
            y: 0.0,
            theta: 0.0,
        };
        let turning_vel = BaseVelocity {
            x: 0.0,
            y: 0.0,
            theta: 1.0,
        };

        let straight_wheel_vel = dd.transform_velocity_base_to_wheel(&straight_vel);
        let turning_wheel_vel = dd.transform_velocity_base_to_wheel(&turning_vel);

        assert_approx_eq!(straight_wheel_vel[0], 2.0);
        assert_approx_eq!(straight_wheel_vel[1], 2.0);
        assert_approx_eq!(turning_wheel_vel[0], -1.0);
        assert_approx_eq!(turning_wheel_vel[1], 1.0);
    }

    fn diff_drive_builder() -> DifferentialDrive<DummyMotorDriveVelocity> {
        let param = DifferentialDriveHardwareParameters {
            wheel_radius: 0.5,
            tread_width: 1.0,
        };
        let controller = DifferentialDriveMotorController {
            left: DummyMotorDriveVelocity::default(),
            right: DummyMotorDriveVelocity::default(),
        };

        controller.left.set_motor_velocity(1.234).unwrap();
        controller.right.set_motor_velocity(2.345).unwrap();

        let limit_velocity = BaseVelocity::new(LIMIT_VEL_X, 0.0, LIMIT_VEL_THETA);
        let limit_acceleration = BaseAcceleration::new(LIMIT_ACC_X, 0.0, LIMIT_ACC_THETA);

        DifferentialDrive::new(param, controller, limit_velocity, limit_acceleration)
    }
}
