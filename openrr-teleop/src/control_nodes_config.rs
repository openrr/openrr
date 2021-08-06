use std::{collections::HashMap, path::PathBuf, sync::Arc};

use arci::{JointTrajectoryClient, MoveBase, Speaker};
use openrr_client::{ArcRobotClient, IkSolverWithChain, JointsPose};
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
    ) -> Vec<Arc<dyn ControlNode>> {
        let mut nodes: Vec<Arc<dyn ControlNode>> = vec![];

        for joy_joint_teleop_config in &self.joy_joint_teleop_configs {
            nodes.push(Arc::new(JoyJointTeleopNode::new_from_config(
                joy_joint_teleop_config.config.clone(),
                joint_trajectory_client_map[&joy_joint_teleop_config.client_name].clone(),
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
                joint_trajectory_client_map[&ik_node_teleop_config.joint_trajectory_client_name]
                    .clone(),
                speaker.clone(),
                ik_solver_with_chain_map[&ik_node_teleop_config.solver_name].clone(),
            );
            nodes.push(Arc::new(ik_node));
        }

        if !joints_poses.is_empty() {
            if let Some(sender_config) = &self.joints_pose_sender_config {
                let mut joint_trajectory_clients = HashMap::new();
                for joints_pose in &joints_poses {
                    joint_trajectory_clients.insert(
                        joints_pose.client_name.to_owned(),
                        joint_trajectory_client_map[&joints_pose.client_name].clone(),
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

        nodes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gen() {
        // JoyJointTeleopConfig
        let config = JoyJointTeleopNodeConfig{
            mode :String::from("a"),
            joint_step :1.0_f64,
            step_duration_secs :0.001_f64
        };
        let _joy_tconfig = JoyJointTeleopConfig{
            client_name :String::from("b"),
            config
        };

        // IkNodeTeleopConfig
        let _iknode_tconfig = IkNodeTeleopConfig{

        };
    }
}
