use arci::*;
use assert_approx_eq::assert_approx_eq;
use openrr_client::*;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;

struct PanicJointTrajectoryClient;

impl JointTrajectoryClient for PanicJointTrajectoryClient {
    #[track_caller]
    fn joint_names(&self) -> Vec<String> {
        // TODO
        // panic!("PanicJointTrajectoryClient::joint_names")
        vec![
            "l_shoulder_yaw".into(),
            "l_shoulder_pitch".into(),
            "l_shoulder_roll".into(),
            "l_elbow_pitch".into(),
            "l_wrist_yaw".into(),
            "l_wrist_pitch".into(),
        ]
    }

    #[track_caller]
    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        panic!("PanicJointTrajectoryClient::current_joint_positions")
    }

    #[track_caller]
    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, arci::Error> {
        panic!(
            "PanicJointTrajectoryClient::send_joint_positions positions={:?}, duration={:?}",
            positions, duration
        )
    }

    #[track_caller]
    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        panic!(
            "PanicJointTrajectoryClient::send_joint_trajectory trajectory={:?}",
            trajectory
        )
    }
}

struct PanicSpeaker;

impl Speaker for PanicSpeaker {
    #[track_caller]
    fn speak(&self, message: &str) -> Result<WaitFuture, arci::Error> {
        panic!("PanicSpeaker::speak message={:?}", message)
    }
}

struct PanicLocalization;

impl Localization for PanicLocalization {
    #[track_caller]
    fn current_pose(&self, frame_id: &str) -> Result<arci::Isometry2<f64>, arci::Error> {
        panic!("PanicLocalization::current_pose frame_id={:?}", frame_id)
    }
}

struct PanicMoveBase;

impl MoveBase for PanicMoveBase {
    #[track_caller]
    fn current_velocity(&self) -> Result<BaseVelocity, arci::Error> {
        panic!("PanicMoveBase::current_velocity")
    }

    #[track_caller]
    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), arci::Error> {
        panic!("PanicMoveBase::send_velocity velocity={:?}", velocity)
    }
}

struct PanicNavigation;

impl Navigation for PanicNavigation {
    #[track_caller]
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: std::time::Duration,
    ) -> Result<WaitFuture, arci::Error> {
        panic!(
            "PanicNavigation::current_pose goal={:?}, frame_id={:?}, timeout={:?}",
            goal, frame_id, timeout
        )
    }

    #[track_caller]
    fn cancel(&self) -> Result<(), arci::Error> {
        panic!("PanicNavigation::cancel")
    }
}

#[test]
fn lazy() {
    let mut root_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root_dir.pop(); // openrr-config

    let mut config: OpenrrClientsConfig = toml::from_str(&format!(
        r#"
urdf_path = "{}/openrr-planner/sample.urdf"
self_collision_check_pairs = ["l_shoulder_yaw:l_gripper_linear1"]

[[joint_trajectory_clients_container_configs]]
name = "arm"
clients_names = ["arm"]

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
        root_dir.display()
    ))
    .unwrap();
    config
        .resolve_path(&config.urdf_path.as_ref().unwrap().clone())
        .unwrap();
    let _client = BoxRobotClient::new(
        config,
        {
            let mut map = HashMap::new();
            map.insert(
                "arm".to_string(),
                Arc::new(PanicJointTrajectoryClient) as Arc<dyn JointTrajectoryClient>,
            );
            map.insert(
                "torso".to_string(),
                Arc::new(PanicJointTrajectoryClient) as Arc<dyn JointTrajectoryClient>,
            );
            map
        },
        {
            let mut map = HashMap::new();
            map.insert("a".to_string(), Arc::new(PanicSpeaker) as Arc<dyn Speaker>);
            map
        },
        Some(Box::new(PanicLocalization)),
        Some(Box::new(PanicMoveBase)),
        Some(Box::new(PanicNavigation)),
    )
    .unwrap();
}

fn new_joint_client(
    joint_names: Vec<String>,
) -> RobotClient<Box<DummyLocalization>, Box<DummyMoveBase>, Box<DummyNavigation>> {
    let mut root_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    root_dir.pop(); // openrr-config

    let mut config: OpenrrClientsConfig = toml::from_str(&format!(
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
        root_dir.display()
    ))
    .unwrap();
    config
        .resolve_path(&config.urdf_path.as_ref().unwrap().clone())
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
    let joint_names: Vec<String> = vec![
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
    assert_eq!(client.joint_names("arm").unwrap(), joint_names);
    let positions = vec![0.1, -0.1, 1.0, 2.0, -1.0, 0.2];
    client
        .send_joint_positions("arm", &positions, 0.1)
        .unwrap()
        .await
        .unwrap();
    let p1 = client.current_joint_positions("arm").unwrap();
    assert_eq!(p1.len(), positions.len());
    for (l, r) in p1.iter().zip(positions.iter()) {
        assert_approx_eq!(l, r);
    }
    client
        .send_joints_pose("arm", "zero", 1.0)
        .unwrap()
        .await
        .unwrap();
    let zero_positions = client.current_joint_positions("arm").unwrap();
    for pos in zero_positions {
        assert_approx_eq!(pos, 0.0);
    }

    assert!(client.current_end_transform("arm").is_err());
    // TODO: check the value
    let _end = client.current_end_transform("arm_ik").unwrap();

    let client_names = client.raw_joint_trajectory_clients_names();
    assert_eq!(client_names.len(), 1);
    assert_eq!(client_names[0], "arm");

    let client_names = client.joint_trajectory_clients_names();
    assert_eq!(client_names.len(), 3);
}
