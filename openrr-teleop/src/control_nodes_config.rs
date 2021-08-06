use std::{collections::HashMap, path::PathBuf, sync::Arc};

use arci::{JointTrajectoryClient, MoveBase, Speaker};
use openrr_client::{ArcRobotClient, Error, IkSolverWithChain, JointsPose};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::{
    ControlNode, IkNode, IkNodeConfig, JointsPoseSender, JointsPoseSenderConfig,
    JoyJointTeleopNode, JoyJointTeleopNodeConfig, MoveBaseNode, RobotCommandConfig,
    RobotCommandExecutor,
};

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JoyJointTeleopConfig {
    pub client_name: String,
    pub config: JoyJointTeleopNodeConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IkNodeTeleopConfig {
    pub solver_name: String,
    pub joint_trajectory_client_name: String,
    pub config: IkNodeConfig,
}

#[derive(Debug, Serialize, Deserialize, Clone, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ControlNodesConfig {
    pub move_base_mode: Option<String>,
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub joy_joint_teleop_configs: Vec<JoyJointTeleopConfig>,
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub ik_node_teleop_configs: Vec<IkNodeTeleopConfig>,
    pub joints_pose_sender_config: Option<JointsPoseSenderConfig>,
    #[serde(default)]
    // https://github.com/alexcrichton/toml-rs/issues/258
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub command_configs: Vec<RobotCommandConfig>,
}

impl ControlNodesConfig {
    #[allow(clippy::too_many_arguments)]
    pub fn create_control_nodes(
        &self,
        base_path: Option<PathBuf>,
        robot_client: Arc<ArcRobotClient>,
        speaker: Arc<dyn Speaker>,
        joint_trajectory_client_map: &HashMap<String, Arc<dyn JointTrajectoryClient>>,
        ik_solver_with_chain_map: &HashMap<String, Arc<IkSolverWithChain>>,
        move_base: Option<Arc<dyn MoveBase>>,
        joints_poses: Vec<JointsPose>,
    ) -> Result<Vec<Box<dyn ControlNode>>, Error> {
        let mut nodes: Vec<Box<dyn ControlNode>> = vec![];

        for joy_joint_teleop_config in &self.joy_joint_teleop_configs {
            nodes.push(Arc::new(JoyJointTeleopNode::new_from_config(
                joy_joint_teleop_config.config.clone(),
                joint_trajectory_client_map
                    .get(&joy_joint_teleop_config.client_name)
                    .ok_or(Error::NoJointTrajectoryClient(
                        "joint_trajectory_client_map".to_owned(),
                    ))?
                    .clone(),
                speaker.clone(),
            )));
        }

        if let Some(mode) = &self.move_base_mode {
            if let Some(m) = move_base {
                nodes.push(Arc::new(MoveBaseNode::new(mode.clone(), m.clone())));
            }
        }

        for ik_node_teleop_config in &self.ik_node_teleop_configs {
            let ik_node = IkNode::new_from_config(
                ik_node_teleop_config.config.clone(),
                joint_trajectory_client_map
                    .get(&ik_node_teleop_config.joint_trajectory_client_name)
                    .ok_or(Error::NoJointTrajectoryClient(
                        "joint_trajectory_client_map".to_owned(),
                    ))?
                    .clone(),
                speaker.clone(),
                ik_solver_with_chain_map
                    .get(&ik_node_teleop_config.solver_name)
                    .ok_or(Error::NoJointTrajectoryClient(
                        "ik_solver_with_chain_map".to_owned(),
                    ))?
                    .clone(),
            );
            nodes.push(Arc::new(ik_node));
        }

        if !joints_poses.is_empty() {
            if let Some(sender_config) = &self.joints_pose_sender_config {
                let mut joint_trajectory_clients = HashMap::new();
                for joints_pose in &joints_poses {
                    joint_trajectory_clients.insert(
                        joints_pose.client_name.to_owned(),
                        joint_trajectory_client_map
                            .get(&joints_pose.client_name)
                            .ok_or(Error::NoJointTrajectoryClient(
                                "joint_trajectory_client_map".to_owned(),
                            ))?
                            .clone(),
                    );
                }
                nodes.push(Arc::new(JointsPoseSender::new_from_config(
                    sender_config.clone(),
                    joints_poses,
                    joint_trajectory_clients,
                    speaker.clone(),
                )));
            }
        }

        if !self.command_configs.is_empty() {
            if let Some(base_path) = base_path {
                if let Some(e) = RobotCommandExecutor::new(
                    base_path,
                    self.command_configs.clone(),
                    robot_client,
                    speaker.clone(),
                ) {
                    nodes.push(Arc::new(e));
                }
            }
        }

        Ok(nodes)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use arci::{DummyLocalization, DummyMoveBase, DummyNavigation, DummySpeaker};
    use assert_approx_eq::*;
    use openrr_client::OpenrrClientsConfig;

    use super::*;
    use crate::{default_move_step_angular, default_move_step_linear, default_step_duration_secs};

    #[test]
    fn test_gen() {
        // JoyJointTeleopConfig
        let config = JoyJointTeleopNodeConfig {
            mode: String::from("a"),
            joint_step: 1.0_f64,
            step_duration_secs: default_step_duration_secs(),
        };
        let _joy_tconfig = JoyJointTeleopConfig {
            client_name: String::from("b"),
            config,
        };

        // IkNodeTeleopConfig
        let config = IkNodeConfig {
            mode: String::from("c"),
            move_step_angular: default_move_step_angular(),
            move_step_linear: default_move_step_linear(),
            step_duration_secs: default_step_duration_secs(),
        };
        assert!(config.move_step_angular.get(0).is_some()); // Config default value check(move_step_angular)
        assert_approx_eq!(config.move_step_angular.get(0).unwrap(), 0.05);
        assert!(config.move_step_angular.get(1).is_some());
        assert_approx_eq!(config.move_step_angular.get(1).unwrap(), 0.05);
        assert!(config.move_step_angular.get(2).is_some());
        assert_approx_eq!(config.move_step_angular.get(2).unwrap(), 0.17);
        assert!(config.move_step_angular.get(3).is_none());
        assert!(config.move_step_linear.get(0).is_some()); // Config default value check(move_step_linear)
        assert_approx_eq!(config.move_step_linear.get(0).unwrap(), 0.01);
        assert!(config.move_step_linear.get(1).is_some());
        assert_approx_eq!(config.move_step_linear.get(1).unwrap(), 0.01);
        assert!(config.move_step_linear.get(2).is_some());
        assert_approx_eq!(config.move_step_linear.get(2).unwrap(), 0.01);
        assert!(config.move_step_linear.get(3).is_none());
        assert_approx_eq!(config.step_duration_secs, 0.1); // Config default value check(step_duration_secs)

        let _iknode_tconfig = IkNodeTeleopConfig {
            solver_name: String::from("d"),
            joint_trajectory_client_name: String::from("f"),
            config,
        };
    }

    #[test]
    fn test_control_node_config_create() {
        let jj_config = JoyJointTeleopConfig {
            client_name: String::from("jj config"),
            config: JoyJointTeleopNodeConfig {
                mode: String::from("jjt config"),
                joint_step: 1.0_f64,
                step_duration_secs: default_step_duration_secs(),
            },
        };
        let ik_config = IkNodeTeleopConfig {
            solver_name: String::from("ik config solver"),
            joint_trajectory_client_name: String::from("ik trajectory"),
            config: IkNodeConfig {
                mode: String::from("ik node config"),
                move_step_angular: default_move_step_angular(),
                move_step_linear: default_move_step_linear(),
                step_duration_secs: default_step_duration_secs(),
            },
        };
        let rc_config = RobotCommandConfig {
            name: String::from("rc config"),
            file_path: String::from("sample path"),
        };
        let ctrl_node_config = ControlNodesConfig {
            move_base_mode: Some(String::from("a")),
            joy_joint_teleop_configs: vec![jj_config],
            ik_node_teleop_configs: vec![ik_config],
            joints_pose_sender_config: None,
            command_configs: vec![rc_config],
        };

        // Check create control node
        let speaker = Arc::new(DummySpeaker::default());
        let robot_client = ArcRobotClient::new(
            OpenrrClientsConfig::default(),
            HashMap::new(),
            HashMap::new(),
            None,
            None,
            None,
        );
        assert!(robot_client.is_ok());
        let robot_client = robot_client.unwrap();
        let nodes = ctrl_node_config.create_control_nodes(
            None,
            Arc::new(robot_client),
            speaker,
            &HashMap::new(),
            &HashMap::new(),
            Some(Arc::new(DummyMoveBase::default())),
            Vec::new(),
        );
        assert!(nodes.is_err());
    }
}
