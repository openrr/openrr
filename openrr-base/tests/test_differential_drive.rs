use std::sync::Arc;

use arci::{BaseVelocity, DummyMotorDriveVelocity, MotorDriveVelocity, MoveBase};
use assert_approx_eq::assert_approx_eq;
use openrr_base::{BaseAcceleration, differential_drive::*};

#[test]
fn test_diff_drive() {
    const LIMIT_VEL_X: f64 = 10.0;
    const LIMIT_VEL_THETA: f64 = 10.0;
    const LIMIT_ACC_X: f64 = 1000.0;
    const LIMIT_ACC_THETA: f64 = 1000.0;

    let param = DifferentialDriveHardwareParameters {
        wheel_radius: 0.3,
        tread_width: 0.5,
    };
    let controller = DifferentialDriveMotorController {
        left: Arc::new(DummyMotorDriveVelocity::default()),
        right: Arc::new(DummyMotorDriveVelocity::default()),
    };

    controller.left.set_motor_velocity(1.234).unwrap();
    controller.right.set_motor_velocity(2.345).unwrap();

    let limit_velocity = BaseVelocity::new(LIMIT_VEL_X, 0.0, LIMIT_VEL_THETA);
    let limit_acceleration = BaseAcceleration::new(LIMIT_ACC_X, 0.0, LIMIT_ACC_THETA);

    let dd = DifferentialDrive::new(param, controller, limit_velocity, limit_acceleration);

    // Wait for decrease acceleration.
    std::thread::sleep(std::time::Duration::from_millis(20));

    let v1 = BaseVelocity {
        x: 1.234,
        y: 0.,
        theta: 2.345,
    };
    let v2 = BaseVelocity {
        x: 12.34,
        y: 0.,
        theta: 23.45,
    };

    dd.send_velocity(&v1).unwrap();
    let c = dd.current_velocity().unwrap();

    assert_approx_eq!(c.x, v1.x);
    assert_approx_eq!(c.y, 0.);
    assert_approx_eq!(c.theta, v1.theta);
    // Wait for decrease acceleration.
    std::thread::sleep(std::time::Duration::from_millis(20));

    dd.send_velocity(&v2).unwrap();
    let c = dd.current_velocity().unwrap();
    assert_approx_eq!(c.x, LIMIT_VEL_X);
    assert_approx_eq!(c.y, 0.);
    assert_approx_eq!(c.theta, LIMIT_VEL_THETA);
}
