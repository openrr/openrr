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
                .ok_or_else(|| anyhow::anyhow!("No joint_position for joint={node}"))?;
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

#[cfg(test)]
mod test {
    use std::time::Duration;

    use arci::{TrajectoryPoint, Vector3};
    use assert_approx_eq::assert_approx_eq;
    use k::{Joint, JointType, Node};

    use super::*;

    #[test]
    fn test_chain_wrapper() {
        let chain_wrapper = ChainWrapper::new(
            vec![String::from("joint")],
            Arc::new(k::Chain::from_nodes(vec![Node::new(Joint::new(
                "joint",
                JointType::Linear {
                    axis: Vector3::y_axis(),
                },
            ))])),
        );

        assert_eq!(chain_wrapper.joint_names(), vec![String::from("joint")]);

        let _ = chain_wrapper
            .send_joint_positions(vec![1.0], Duration::from_secs(1))
            .unwrap();
        assert_approx_eq!(chain_wrapper.current_joint_positions().unwrap()[0], 1.0);

        let _ = chain_wrapper
            .send_joint_trajectory(vec![TrajectoryPoint {
                positions: vec![1.5],
                velocities: Some(vec![1.0]),
                time_from_start: Duration::from_secs(1),
            }])
            .unwrap();
        assert_approx_eq!(chain_wrapper.current_joint_positions().unwrap()[0], 1.5);
    }
}
