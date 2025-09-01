#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use arci::{BaseVelocity, Gamepad};
    use arci_gamepad_gilrs::*;
    use arci_ros::JointVelocityController;
    use openrr_base::{BaseAcceleration, differential_drive::*};
    use openrr_teleop::{ControlMode, MoveBaseMode};

    arci_ros::init("diff_drive");

    let config = GilGamepadConfig::default();
    let gilrs = GilGamepad::new_from_config(config);

    let param = DifferentialDriveHardwareParameters {
        tread_width: 0.3441,
        wheel_radius: 0.17 / 2.0,
    };

    let controller = DifferentialDriveMotorController {
        left: JointVelocityController::new(
            "/left_wheel_motor_velocity_controller/command",
            "/joint_states",
            false,
            0,
        ),
        right: JointVelocityController::new(
            "/right_wheel_motor_velocity_controller/command",
            "/joint_states",
            true,
            1,
        ),
    };

    let limit_velocity = BaseVelocity {
        x: 1.0,
        y: 0.0,
        theta: 2.0,
    };
    let limit_acceleration = BaseAcceleration {
        x: 4.0,
        y: 0.0,
        theta: 4.0,
    };

    let dd = DifferentialDrive::new(param, controller, limit_velocity, limit_acceleration);

    let teleop = MoveBaseMode::new("DD control".to_string(), dd);

    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.unwrap();
        std::process::exit(0);
    });

    loop {
        let gamepad = gilrs.next_event().await;
        teleop.handle_event(gamepad);
        teleop.proc().await;

        std::thread::sleep(std::time::Duration::from_millis(5));
    }
}
