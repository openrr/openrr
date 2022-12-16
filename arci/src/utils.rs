use std::time::Duration;

use tokio::time::interval as AsyncInterval;
use tracing::{debug, info};

use crate::{Error, JointTrajectoryClient, TrajectoryPoint};

pub fn get_joint_index<J>(joint_trajectory_client: &J, joint_name: &str) -> Result<usize, Error>
where
    J: JointTrajectoryClient,
{
    joint_trajectory_client
        .joint_names()
        .iter()
        .position(|name| name == joint_name)
        .ok_or_else(|| Error::NoJoint(joint_name.to_owned()))
}

#[allow(clippy::too_many_arguments)]
pub async fn move_joint_until_stop<J>(
    joint_trajectory_client: &J,
    joint_name: &str,
    target_position: f64,
    target_duration: Duration,
    diff_threshold_for_stop: f64,
    stopped_duration: Duration,
    monitor_interval: Duration,
    duration_to_cancel: Duration,
) -> Result<f64, Error>
where
    J: JointTrajectoryClient,
{
    let joint_index = get_joint_index(joint_trajectory_client, joint_name)?;
    let mut positions = joint_trajectory_client.current_joint_positions()?;
    let mut prev_current_position = positions[joint_index];
    positions[joint_index] = target_position;
    // use send_joint_trajectory, to send target trajectory with max velocity from start time.
    drop(
        joint_trajectory_client
            .send_joint_trajectory(vec![TrajectoryPoint::new(positions, target_duration)])?,
    );
    let mut stopped_count: u64 = 0;
    let stopped_count_threshold =
        (stopped_duration.as_secs_f64() / monitor_interval.as_secs_f64()).ceil() as u64;
    let mut interval = AsyncInterval(monitor_interval);

    loop {
        let current_position = joint_trajectory_client.current_joint_positions()?[joint_index];
        let diff_position = (prev_current_position - current_position).abs();
        debug!(
            "diff={diff_position:.4} diff_threshold={diff_threshold_for_stop:.4} stopped_count={stopped_count}",
        );
        if diff_position < diff_threshold_for_stop {
            stopped_count += 1;
        } else {
            stopped_count = 0;
        }
        if stopped_count > stopped_count_threshold {
            info!("Stopped.");
            break;
        }
        prev_current_position = current_position;
        interval.tick().await;
    }
    let stopped_position = joint_trajectory_client.current_joint_positions()?[joint_index];
    joint_trajectory_client
        .send_joint_positions(
            joint_trajectory_client.current_joint_positions()?,
            duration_to_cancel,
        )?
        .await?;
    Ok(stopped_position)
}
