use crate::{
    ControlNode, IkNode, IkNodeConfig, JointsPoseSender, JointsPoseSenderConfig,
    JoyJointTeleopNode, JoyJointTeleopNodeConfig, MoveBaseNode,
};
use arci::{JointTrajectoryClient, MoveBase, Speaker};
use openrr_client::{IkSolverWithChain, JointsPose};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, sync::Arc};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct JoyJointTeleopConfig {
    pub config: JoyJointTeleopNodeConfig,
    pub client_name: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct IkNodeTeleopConfig {
    pub config: IkNodeConfig,
    pub solver_name: String,
    pub joint_trajectory_client_name: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ControlNodesConfig {
    pub joy_joint_teleop_configs: Vec<JoyJointTeleopConfig>,
    pub move_base_mode: Option<String>,
    pub ik_node_teleop_configs: Vec<IkNodeTeleopConfig>,
    pub joints_pose_sender_config: Option<JointsPoseSenderConfig>,
}

impl ControlNodesConfig {
    pub fn create_control_nodes(
        &self,
        speaker: Arc<dyn Speaker>,
        joint_trajectory_client_map: &HashMap<String, Arc<dyn JointTrajectoryClient>>,
        ik_solver_with_chain_map: &HashMap<String, Arc<IkSolverWithChain>>,
        move_base: Option<Arc<dyn MoveBase>>,
        joints_poses: Vec<JointsPose>,
    ) -> Vec<Box<dyn ControlNode>> {
        let mut nodes: Vec<Box<dyn ControlNode>> = vec![];

        for joy_joint_teleop_config in &self.joy_joint_teleop_configs {
            nodes.push(Box::new(JoyJointTeleopNode::new_from_config(
                joy_joint_teleop_config.config.clone(),
                joint_trajectory_client_map[&joy_joint_teleop_config.client_name].clone(),
                speaker.clone(),
            )));
        }

        if let Some(mode) = &self.move_base_mode {
            if let Some(m) = move_base {
                nodes.push(Box::new(MoveBaseNode::new(mode.clone(), m.clone())));
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
            nodes.push(Box::new(ik_node));
        }

        if !joints_poses.is_empty() {
            if let Some(sender_config) = &self.joints_pose_sender_config {
                nodes.push(Box::new(JointsPoseSender::new_from_config(
                    sender_config.clone(),
                    joints_poses,
                    joint_trajectory_client_map,
                    speaker.clone(),
                )));
            }
        }

        nodes
    }
}
