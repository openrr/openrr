use arci::{BaseVelocity, Localization, MotorDriveVelocity, MoveBase};

use crate::*;

pub struct DifferentialDriveHardwareParameters {
    pub wheel_radius: f64,
    pub tread_width: f64,
}

pub struct DifferentialDriveMotorController<MV: MotorDriveVelocity> {
    pub left: MV,
    pub right: MV,
}

pub struct DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    robot_velocity: RobotVelocityStatus,
    controller: DifferentialDriveMotorController<MV>,
    param: DifferentialDriveHardwareParameters,
    odometry: Odometry,
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
            robot_velocity: RobotVelocityStatus::new(limit_velocity, limit_acceleration),
            controller,
            param,
            odometry: Odometry::default(),
        }
    }

    /// Output: left_wheel_velocity, right_wheel_velocity
    pub fn transform_velocity_base_to_wheel(&self, velocity: &BaseVelocity) -> [f64; 2] {
        [
            (velocity.x - 0.5 * self.param.tread_width * velocity.theta) / self.param.wheel_radius,
            (velocity.x + 0.5 * self.param.tread_width * velocity.theta) / self.param.wheel_radius,
        ]
    }

    /// Input: left_wheel_velocity, right_wheel_velocity
    pub fn transform_velocity_wheel_to_base(&self, wheels_vel: &[f64; 2]) -> BaseVelocity {
        let left = wheels_vel[0];
        let right = wheels_vel[1];
        let translation = 0.5 * self.param.wheel_radius * (left + right);
        let rotation = self.param.wheel_radius * (right - left) / self.param.tread_width;
        BaseVelocity {
            x: translation,
            y: 0f64,
            theta: rotation,
        }
    }
}

impl<MV> MoveBase for DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), arci::Error> {
        let limited_vel = self.robot_velocity.get_limited_velocity(velocity);

        let wheels_vel = self.transform_velocity_base_to_wheel(&limited_vel);

        self.controller
            .left
            .set_motor_velocity(wheels_vel[0])
            .unwrap();
        self.controller
            .right
            .set_motor_velocity(wheels_vel[1])
            .unwrap();

        let wheels_feedback = [
            self.controller.left.get_motor_velocity().unwrap(),
            self.controller.right.get_motor_velocity().unwrap(),
        ];
        let feedback_vel = self.transform_velocity_wheel_to_base(&wheels_feedback);

        match self.odometry.translate(&feedback_vel) {
            Ok(_) => {}
            Err(e) => println!("{e}"),
        }

        self.robot_velocity.set_log(&limited_vel);

        Ok(())
    }

    fn current_velocity(&self) -> Result<BaseVelocity, arci::Error> {
        let left_wheel_rotation = self.controller.left.get_motor_velocity().unwrap();
        let right_wheel_rotation = self.controller.right.get_motor_velocity().unwrap();

        let feedback_vel =
            self.transform_velocity_wheel_to_base(&[left_wheel_rotation, right_wheel_rotation]);

        self.robot_velocity.set_feedback_velocity(feedback_vel);

        Ok(feedback_vel)
    }
}

impl<MV> Localization for DifferentialDrive<MV>
where
    MV: MotorDriveVelocity,
{
    fn current_pose(&self, _frame_id: &str) -> Result<arci::Isometry2<f64>, arci::Error> {
        match self.odometry.current_pose() {
            Ok(pose) => Ok(pose),
            Err(e) => Err(arci::Error::Other(e.into())),
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
    fn test_velocity_transformer() {
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

        let dd = DifferentialDrive::new(param, controller, limit_velocity, limit_acceleration);

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
}
