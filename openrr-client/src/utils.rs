use arci::*;

pub fn wait_joint_positions(
    client: &dyn JointTrajectoryClient,
    target_positions: &[f64],
    timeout: std::time::Duration,
    allowable_total_diff: f64,
) -> Result<(), Error> {
    let sleep_unit = std::time::Duration::from_millis(100);
    let max_num = timeout.as_micros() / sleep_unit.as_micros();
    let dof = target_positions.len();
    let mut sum_err = 0.0;
    for _iteration in 0..max_num {
        let cur = client.current_joint_positions()?;
        sum_err = 0.0;
        for i in 0..dof {
            sum_err += (target_positions[i] - cur[i]).abs();
        }
        if sum_err < allowable_total_diff {
            return Ok(());
        }
        std::thread::sleep(sleep_unit);
    }
    Err(Error::Timeout {
        timeout,
        allowable_total_diff,
        err: sum_err,
    })
}

pub fn find_nodes(joint_names: &[String], chain: &k::Chain<f64>) -> Option<Vec<k::Node<f64>>> {
    let mut nodes = vec![];
    for name in joint_names {
        if let Some(node) = chain.find(name) {
            nodes.push(node.clone());
        } else {
            return None;
        }
    }
    Some(nodes)
}

#[cfg(test)]
mod test {
    use k::{Chain, Joint, JointType, Node};

    use super::*;

    #[test]
    fn test_wait_joint_positions() {
        let client = DummyJointTrajectoryClient::new(vec![String::from("joint")]);
        let target = [1f64];
        let timeout = std::time::Duration::from_millis(1000);
        let allowable_total_diff = 10.;
        let impossible_total_diff = 0.01;
        assert!(wait_joint_positions(&client, &target, timeout, allowable_total_diff).is_ok());
        assert!(wait_joint_positions(&client, &target, timeout, impossible_total_diff).is_err());
    }

    #[test]
    fn test_find_nodes() {
        let joint_names = [String::from("joint")];
        let fake_joint_names = [String::from("fake_joint")];
        let chain: Chain<f64> = Chain::from_nodes(vec![Node::new(Joint::new(
            "joint",
            JointType::Linear {
                axis: Vector3::y_axis(),
            },
        ))]);

        assert!(find_nodes(&joint_names, &chain).is_some());
        assert!(find_nodes(&fake_joint_names, &chain).is_none());
    }
}
