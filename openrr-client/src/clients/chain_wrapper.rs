use std::sync::Arc;

use arci::{JointTrajectoryClient, WaitFuture};

use crate::utils::find_nodes;

pub struct ChainWrapper {
    joint_names: Vec<String>,
    full_chain: Arc<k::Chain<f64>>,
    nodes: Vec<k::Node<f64>>,
}

impl ChainWrapper {
    pub fn new(joint_names: Vec<String>, full_chain: Arc<k::Chain<f64>>) -> Self {
        let nodes = find_nodes(&joint_names, full_chain.as_ref()).unwrap();
        Self {
            joint_names,
            full_chain,
            nodes,
        }
    }
}

impl JointTrajectoryClient for ChainWrapper {
    fn joint_names(&self) -> Vec<String> {
        self.joint_names.clone()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        self.full_chain.update_transforms();
        let mut positions = vec![0.0; self.joint_names.len()];
        for (index, node) in self.nodes.iter().enumerate() {
            positions[index] = node
                .joint_position()
                .ok_or_else(|| anyhow::anyhow!("No joint_position for joint={}", node))?;
        }
        Ok(positions)
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        _duration: std::time::Duration,
    ) -> Result<WaitFuture, arci::Error> {
        for (index, node) in self.nodes.iter().enumerate() {
            node.set_joint_position_clamped(positions[index]);
        }
        self.full_chain.update_transforms();
        Ok(WaitFuture::ready())
    }

    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<WaitFuture, arci::Error> {
        if let Some(last_point) = trajectory.last() {
            self.send_joint_positions(last_point.positions.clone(), last_point.time_from_start)
        } else {
            Ok(WaitFuture::ready())
        }
    }
}
