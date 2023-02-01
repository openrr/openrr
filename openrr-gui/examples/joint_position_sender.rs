/*
Example of joint_position_sender with dummy JointTrajectoryClient.

```bash
cargo run -p openrr-gui --example joint_position_sender
```
*/

use std::{collections::HashMap, sync::Arc};

use openrr_client::{OpenrrClientsConfig, RobotClient};

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "openrr_gui=debug".to_owned()),
        )
        .init();

    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../openrr-planner/sample.urdf");
    let robot = urdf_rs::read_file(urdf_path)?;
    let joint_names_l = vec![
        "l_shoulder_yaw".to_owned(),
        "l_shoulder_pitch".to_owned(),
        "l_shoulder_roll".to_owned(),
        "l_elbow_pitch".to_owned(),
        "l_wrist_yaw".to_owned(),
        "l_wrist_pitch".to_owned(),
    ];
    let joint_names_r = vec![
        "r_shoulder_yaw".to_owned(),
        "r_shoulder_pitch".to_owned(),
        "r_shoulder_roll".to_owned(),
        "r_elbow_pitch".to_owned(),
        "r_wrist_yaw".to_owned(),
        "r_wrist_pitch".to_owned(),
    ];
    let joint_trajectory_client_l = arci::DummyJointTrajectoryClient::new(joint_names_l);
    let joint_trajectory_client_r = arci::DummyJointTrajectoryClient::new(joint_names_r);
    let mut config = OpenrrClientsConfig::default();
    config.urdf_path = Some(urdf_path.to_owned());
    let robot_client =
        RobotClient::<arci::DummyLocalization, arci::DummyMoveBase, arci::DummyNavigation>::new(
            config,
            vec![
                (
                    "l_arm".to_owned(),
                    Arc::new(joint_trajectory_client_l) as Arc<dyn arci::JointTrajectoryClient>,
                ),
                ("r_arm".to_owned(), Arc::new(joint_trajectory_client_r)),
            ]
            .into_iter()
            .collect(),
            HashMap::new(),
            None,
            None,
            None,
        )?;
    openrr_gui::joint_position_sender(robot_client, robot)?;
    Ok(())
}
