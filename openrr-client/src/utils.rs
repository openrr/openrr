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
