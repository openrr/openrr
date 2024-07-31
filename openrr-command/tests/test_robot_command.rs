use std::{collections::HashMap, sync::Arc};

use arci::*;
use openrr_client::RobotClient;
use openrr_command::{RobotCommand, RobotCommandExecutor};

fn new_joint_client(
    joint_names: Vec<String>,
) -> RobotClient<Box<DummyLocalization>, Box<DummyMoveBase>, Box<DummyNavigation>> {
    let mut root_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root_dir.pop(); // openrr-command

    let mut config: openrr_client::OpenrrClientsConfig = toml::from_str(&format!(
        r#"
    urdf_path = "{}/openrr-planner/sample.urdf"
    self_collision_check_pairs = ["l_shoulder_yaw:l_gripper_linear1"]

    [[collision_check_clients_configs]]
    name = "arm_collision_checked"
    client_name = "arm"

    [[ik_clients_configs]]
    name = "arm_ik"
    client_name = "arm_collision_checked"
    solver_name = "arm_ik_solver"

    [ik_solvers_configs.arm_ik_solver]
    ik_target = "l_tool_fixed"

    [[joints_poses]]
    pose_name = "zero"
    client_name = "arm"
    positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    "#,
        root_dir.display().to_string().replace('\\', "/")
    ))
    .unwrap();
    config
        .resolve_path(config.urdf_path.as_ref().unwrap().clone())
        .unwrap();
    RobotClient::new(
        config,
        {
            let mut map = HashMap::new();
            map.insert(
                "arm".to_string(),
                Arc::new(DummyJointTrajectoryClient::new(joint_names))
                    as Arc<dyn JointTrajectoryClient>,
            );
            map
        },
        {
            let mut map = HashMap::new();
            map.insert(
                "speaker".to_string(),
                Arc::new(DummySpeaker::new()) as Arc<dyn Speaker>,
            );
            map
        },
        Some(Box::new(DummyLocalization::new())),
        Some(Box::new(DummyMoveBase::new())),
        Some(Box::new(DummyNavigation::new())),
    )
    .unwrap()
}

#[tokio::test]
async fn test_joint_positions() {
    let joint_names: Vec<String> = [
        "l_shoulder_yaw",
        "l_shoulder_pitch",
        "l_shoulder_roll",
        "l_elbow_pitch",
        "l_wrist_yaw",
        "l_wrist_pitch",
    ]
    .iter()
    .map(|x| x.to_string())
    .collect();
    let client = new_joint_client(joint_names.clone());

    let ex = RobotCommandExecutor {};
    ex.execute(
        &client,
        &RobotCommand::SendJoints {
            name: "arm".to_string(),
            duration: 0.1,
            use_interpolation: false,
            max_resolution_for_interpolation: 0.1,
            min_number_of_points_for_interpolation: 10,
            joint: vec![(0, 0.1), (2, 0.5)],
        },
    )
    .await
    .unwrap();

    ex.execute(
        &client,
        &RobotCommand::SendJointsPose {
            name: "arm".to_string(),
            pose_name: "zero".to_string(),
            duration: 0.1,
        },
    )
    .await
    .unwrap();
    // TODO: check the values

    ex.execute(
        &client,
        &RobotCommand::GetState {
            name: "arm".to_string(),
        },
    )
    .await
    .unwrap();
    // TODO: check the values

    ex.execute(&client, &RobotCommand::List).await.unwrap();
    // TODO: check the values

    ex.execute(
        &client,
        &RobotCommand::Speak {
            name: "speaker".to_string(),
            message: vec!["test".to_string()],
        },
    )
    .await
    .unwrap();
    // TODO: check the values
}
