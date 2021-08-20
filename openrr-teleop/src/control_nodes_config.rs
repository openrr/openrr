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
    ) -> Result<Vec<Arc<dyn ControlNode>>, Error> {
        let mut nodes: Vec<Arc<dyn ControlNode>> = vec![];

        for joy_joint_teleop_config in &self.joy_joint_teleop_configs {
            nodes.push(Arc::new(JoyJointTeleopNode::new_from_config(
                joy_joint_teleop_config.config.clone(),
                joint_trajectory_client_map
                    .get(&joy_joint_teleop_config.client_name)
                    .ok_or_else(|| {
                        Error::NoMapKey(
                            format!("joy_joint_teleop_configs[{} l.{}]", file!(), line!()),
                            joy_joint_teleop_config.client_name.to_string(),
                        )
                    })?
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
                    .ok_or_else(|| {
                        Error::NoMapKey(
                            format!(
                                "joint_trajectory_client_map(from ik_node_teleop_configs)[{} l.{}]",
                                file!(),
                                line!()
                            ),
                            ik_node_teleop_config
                                .joint_trajectory_client_name
                                .to_string(),
                        )
                    })?
                    .clone(),
                speaker.clone(),
                ik_solver_with_chain_map
                    .get(&ik_node_teleop_config.solver_name)
                    .ok_or_else(|| {
                        Error::NoMapKey(
                            format!(
                                "ik_solver_with_chain_map(from ik_node_teleop_configs)[{} l.{}]",
                                file!(),
                                line!()
                            ),
                            ik_node_teleop_config.solver_name.to_string(),
                        )
                    })?
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
                            .ok_or_else(|| {
                                Error::NoMapKey(
                                    format!(
                                        "joint_trajectory_client_map(form joint pose)[{} l.{}]",
                                        file!(),
                                        line!()
                                    ),
                                    joints_pose.client_name.to_string(),
                                )
                            })?
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

    use arci::{
        DummyJointTrajectoryClient, DummyLocalization, DummyMoveBase, DummyNavigation, DummySpeaker,
    };
    use openrr_client::{
        create_random_jacobian_ik_solver, IkSolverParameters, IkSolverWithChain,
        OpenrrClientsConfig,
    };

    use super::*;

    #[test]
    fn test_gen() {
        // JoyJointTeleopConfig
        let config = JoyJointTeleopNodeConfig {
            mode: String::from("a"),
            joint_step: 1.0_f64,
            step_duration_secs: 0.1_f64,
        };
        let _joy_tconfig = JoyJointTeleopConfig {
            client_name: String::from("b"),
            config,
        };

        // IkNodeTeleopConfig
        let config = IkNodeConfig {
            mode: String::from("c"),
            move_step_angular: [1.0_f64, 1.0, 1.0],
            move_step_linear: [1.0_f64, 1.0, 1.0],
            step_duration_secs: 0.1_f64,
        };

        let _ik_node_tconfig = IkNodeTeleopConfig {
            solver_name: String::from("d"),
            joint_trajectory_client_name: String::from("f"),
            config,
        };
    }

    #[test]
    fn test_control_node_config_create_error() {
        let jj_config = JoyJointTeleopConfig {
            client_name: String::from("jj config"),
            config: JoyJointTeleopNodeConfig {
                mode: String::from("jjt config"),
                joint_step: 1.0_f64,
                step_duration_secs: 0.1_f64,
            },
        };
        let ik_config = IkNodeTeleopConfig {
            solver_name: String::from("ik config solver"),
            joint_trajectory_client_name: String::from("ik trajectory"),
            config: IkNodeConfig {
                mode: String::from("ik node config"),
                move_step_angular: [1.0_f64, 1.0, 1.0],
                move_step_linear: [1.0_f64, 1.0, 1.0],
                step_duration_secs: 0.1_f64,
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

        // Check create control node(Error:No HashMap item[Key=jj config])
        // HashMap key is required by ControlNodesConfig
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

        // the 4th argument `joint_trajectory_client_map` had to have keys same with the items in `joy_joint_teleop_configs`
        let nodes = ctrl_node_config.create_control_nodes(
            None,
            Arc::new(robot_client),
            speaker.clone(),
            &HashMap::new(),
            &HashMap::new(),
            Some(Arc::new(DummyMoveBase::default())),
            Vec::new(),
        );
        assert!(nodes.is_err());
        let err = nodes.err().unwrap();
        println!("{}", err);

        // Check create control node(Error:No HashMap item[Key=ik_trajectory])
        // HashMap key is required by ControlNodesConfig
        let robot_client = ArcRobotClient::new(
            OpenrrClientsConfig::default(),
            HashMap::new(),
            HashMap::new(),
            None,
            None,
            None,
        );
        let robot_client = robot_client.unwrap();
        let jtc: Arc<dyn JointTrajectoryClient> =
            Arc::new(DummyJointTrajectoryClient::new(vec!["a".to_owned()]));
        let mut jt_map = HashMap::new();
        jt_map.insert("jj config".to_owned(), jtc);

        let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
        let end_link = chain.find("l_tool_fixed").unwrap();
        let arm = k::SerialChain::from_end(end_link);
        let params = IkSolverParameters {
            allowable_position_error: 0.01,
            allowable_angle_error: 0.02,
            jacobian_multiplier: 0.1,
            num_max_try: 100,
        };
        let ik_solver = create_random_jacobian_ik_solver(&params);
        let constraints = k::Constraints::default();
        let ik_solver = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);
        let mut ik_map = HashMap::new();
        ik_map.insert("ik config solver".to_owned(), Arc::new(ik_solver));

        let nodes = ctrl_node_config.create_control_nodes(
            None,
            Arc::new(robot_client),
            speaker,
            &jt_map,
            &ik_map,
            Some(Arc::new(DummyMoveBase::default())),
            Vec::new(),
        );
        assert!(nodes.is_err());
        let err = nodes.err().unwrap();
        println!("{}", err);
    }

    #[test]
    fn test_control_node_config_create_ok() {
        let jj_config = JoyJointTeleopConfig {
            client_name: String::from("jj config"),
            config: JoyJointTeleopNodeConfig {
                mode: String::from("jjt config"),
                joint_step: 1.0_f64,
                step_duration_secs: 0.1_f64,
            },
        };
        let ik_config = IkNodeTeleopConfig {
            solver_name: String::from("ik config solver"),
            joint_trajectory_client_name: String::from("jj config"),
            config: IkNodeConfig {
                mode: String::from("ik node config"),
                move_step_angular: [1.0_f64, 1.0, 1.0],
                move_step_linear: [1.0_f64, 1.0, 1.0],
                step_duration_secs: 0.1_f64,
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

        // Check create control node(Ok)
        let speaker = Arc::new(DummySpeaker::default());
        let robot_client = ArcRobotClient::new(
            OpenrrClientsConfig::default(),
            HashMap::new(),
            HashMap::new(),
            Some(Arc::new(DummyLocalization::default())),
            Some(Arc::new(DummyMoveBase::default())),
            Some(Arc::new(DummyNavigation::default())),
        );
        assert!(robot_client.is_ok());
        let robot_client = robot_client.unwrap();

        let jtc: Arc<dyn JointTrajectoryClient> =
            Arc::new(DummyJointTrajectoryClient::new(vec!["a".to_owned()]));
        let mut jt_map = HashMap::new();
        jt_map.insert("jj config".to_owned(), jtc);

        let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
        let end_link = chain.find("l_tool_fixed").unwrap();
        let arm = k::SerialChain::from_end(end_link);
        let params = IkSolverParameters {
            allowable_position_error: 0.01,
            allowable_angle_error: 0.02,
            jacobian_multiplier: 0.1,
            num_max_try: 100,
        };
        let ik_solver = create_random_jacobian_ik_solver(&params);
        let constraints = k::Constraints::default();
        let ik_solver = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);
        let mut ik_map = HashMap::new();
        ik_map.insert("ik config solver".to_owned(), Arc::new(ik_solver));

        let nodes = ctrl_node_config.create_control_nodes(
            None,
            Arc::new(robot_client),
            speaker,
            &jt_map,
            &ik_map,
            Some(Arc::new(DummyMoveBase::default())),
            Vec::new(),
        );
        assert!(nodes.is_ok());
    }
}
