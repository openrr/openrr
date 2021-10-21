use std::{sync::Arc, time::Duration};

use anyhow::format_err;
use arci::{
    copy_joint_positions, Error, JointPositionLimit, JointPositionLimiter, JointTrajectoryClient,
    JointVelocityLimiter, TrajectoryPoint,
};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::msg::{
    control_msgs::JointTrajectoryControllerState,
    trajectory_msgs::{JointTrajectory, JointTrajectoryPoint},
};

pub(crate) fn extract_current_joint_positions_from_message(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
) -> Result<Vec<f64>, arci::Error> {
    // TODO: cache index map and use it
    let mut result = vec![0.0; client.joint_names().len()];
    copy_joint_positions(
        &state.joint_names,
        &state.actual.positions,
        &client.joint_names(),
        &mut result,
    )?;
    Ok(result)
}

pub(crate) fn create_joint_trajectory_message_for_send_joint_positions(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
    positions: &[f64],
    duration: Duration,
) -> Result<JointTrajectory, arci::Error> {
    let partial_names = client.joint_names();
    if partial_names.len() != positions.len() {
        return Err(arci::Error::LengthMismatch {
            model: partial_names.len(),
            input: positions.len(),
        });
    }
    // TODO: cache index map and use it
    let full_names = state.joint_names;
    let full_dof = full_names.len();

    let mut full_positions = state.actual.positions;
    copy_joint_positions(&partial_names, positions, &full_names, &mut full_positions)?;

    let point_with_full_positions = JointTrajectoryPoint {
        positions: full_positions,
        // add zero velocity to use cubic interpolation in trajectory_controller
        velocities: vec![0.0; full_dof],
        time_from_start: duration.into(),
        ..Default::default()
    };
    Ok(JointTrajectory {
        joint_names: full_names,
        points: vec![point_with_full_positions],
        ..Default::default()
    })
}

pub(crate) fn create_joint_trajectory_message_for_send_joint_trajectory(
    client: &dyn JointTrajectoryClient,
    state: JointTrajectoryControllerState,
    trajectory: &[TrajectoryPoint],
) -> Result<JointTrajectory, arci::Error> {
    // TODO: cache index map and use it
    let current_full_positions = state.actual.positions;
    let full_names = state.joint_names;
    let full_dof = current_full_positions.len();

    Ok(JointTrajectory {
        points: trajectory
            .iter()
            .map(|tp| {
                let mut full_positions = current_full_positions.clone();
                copy_joint_positions(
                    &client.joint_names(),
                    &tp.positions,
                    &full_names,
                    &mut full_positions,
                )?;
                Ok(JointTrajectoryPoint {
                    positions: full_positions,
                    velocities: if let Some(partial_velocities) = &tp.velocities {
                        let mut full_velocities = vec![0.0; full_dof];
                        copy_joint_positions(
                            &client.joint_names(),
                            partial_velocities,
                            &full_names,
                            &mut full_velocities,
                        )?;
                        full_velocities
                    } else {
                        vec![]
                    },
                    time_from_start: tp.time_from_start.into(),
                    ..Default::default()
                })
            })
            .collect::<Result<Vec<_>, arci::Error>>()?,
        joint_names: full_names,
        ..Default::default()
    })
}

fn new_joint_position_limiter<C>(
    client: C,
    position_limits: Option<Vec<JointPositionLimit>>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<JointPositionLimiter<C>, Error>
where
    C: JointTrajectoryClient,
{
    match position_limits {
        Some(position_limits) => Ok(JointPositionLimiter::new(client, position_limits)),
        None => JointPositionLimiter::from_urdf(client, &urdf_robot.unwrap().joints),
    }
}

fn new_joint_velocity_limiter<C>(
    client: C,
    velocity_limits: Option<Vec<f64>>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<JointVelocityLimiter<C>, Error>
where
    C: JointTrajectoryClient,
{
    match velocity_limits {
        Some(velocity_limits) => Ok(JointVelocityLimiter::new(client, velocity_limits)),
        None => JointVelocityLimiter::from_urdf(client, &urdf_robot.unwrap().joints),
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct JointTrajectoryClientWrapperConfig {
    #[serde(default)]
    pub wrap_with_joint_position_limiter: bool,
    #[serde(default)]
    pub wrap_with_joint_velocity_limiter: bool,
    pub joint_velocity_limits: Option<Vec<f64>>,

    // TOML format has a restriction that if a table itself contains tables,
    // all keys with non-table values must be emitted first.
    // Therefore, these fields must be located at the end of the struct.
    pub joint_position_limits: Option<Vec<JointPositionLimit>>,
}

impl JointTrajectoryClientWrapperConfig {
    pub(crate) fn check_urdf_is_not_necessary(&self) -> Result<(), arci::Error> {
        if self.wrap_with_joint_position_limiter && self.joint_position_limits.is_none() {
            return Err(format_err!(
                "`wrap_with_joint_position_limiter=true` requires urdf or joint_position_limits \
                 is specified",
            )
            .into());
        }
        if self.wrap_with_joint_velocity_limiter && self.joint_velocity_limits.is_none() {
            return Err(format_err!(
                "`wrap_with_joint_velocity_limiter=true` requires urdf or joint_velocity_limits \
                 is specified",
            )
            .into());
        }
        Ok(())
    }
}

pub(crate) fn wrap_joint_trajectory_client(
    config: JointTrajectoryClientWrapperConfig,
    client: Arc<dyn JointTrajectoryClient>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<Arc<dyn JointTrajectoryClient>, arci::Error> {
    Ok(if config.wrap_with_joint_velocity_limiter {
        if config.wrap_with_joint_position_limiter {
            Arc::new(new_joint_position_limiter(
                new_joint_velocity_limiter(client, config.joint_velocity_limits, urdf_robot)?,
                config.joint_position_limits,
                urdf_robot,
            )?)
        } else {
            Arc::new(new_joint_velocity_limiter(
                client,
                config.joint_velocity_limits,
                urdf_robot,
            )?)
        }
    } else if config.wrap_with_joint_position_limiter {
        Arc::new(new_joint_position_limiter(
            client,
            config.joint_position_limits,
            urdf_robot,
        )?)
    } else {
        Arc::new(client)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_joint_position_limiter() {
        use arci::{DummyJointTrajectoryClient, JointPositionLimit, JointPositionLimiter};

        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let limits = vec![
            JointPositionLimit::new(-5.0, 5.0),
            JointPositionLimit::new(-5.0, 5.0),
        ];

        let result = new_joint_position_limiter(client, Some(limits.clone()), None);
        assert!(result.is_ok());
        let _result = result.unwrap();

        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let _correct = JointPositionLimiter::new(client, limits);
    }

    #[test]
    #[should_panic]
    fn test_new_joint_position_limiter_error() {
        use arci::DummyJointTrajectoryClient;

        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);

        let _ = new_joint_position_limiter(client, None, None);
    }
}
