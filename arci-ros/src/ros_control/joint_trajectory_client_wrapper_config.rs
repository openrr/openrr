use std::sync::Arc;

use anyhow::format_err;
use arci::{
    Error, JointPositionDifferenceLimiter, JointPositionLimit, JointPositionLimiter,
    JointPositionLimiterStrategy, JointTrajectoryClient, JointVelocityLimiter,
};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

fn new_joint_position_difference_limiter<C>(
    client: C,
    position_difference_limits: Option<Vec<f64>>,
) -> Result<JointPositionDifferenceLimiter<C>, Error>
where
    C: JointTrajectoryClient,
{
    match position_difference_limits {
        Some(position_difference_limits) => Ok(JointPositionDifferenceLimiter::new(
            client,
            position_difference_limits,
        )?),
        None => Err(Error::Other(anyhow::format_err!(
            "No position_difference_limits is specified"
        ))),
    }
}

fn new_joint_position_limiter<C>(
    client: C,
    position_limits: Option<Vec<JointPositionLimit>>,
    strategy: JointPositionLimiterStrategy,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<JointPositionLimiter<C>, Error>
where
    C: JointTrajectoryClient,
{
    match position_limits {
        Some(position_limits) => Ok(JointPositionLimiter::new_with_strategy(
            client,
            position_limits,
            strategy,
        )),
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
    pub joint_position_limiter_strategy: JointPositionLimiterStrategy,

    #[serde(default)]
    pub wrap_with_joint_velocity_limiter: bool,
    pub joint_velocity_limits: Option<Vec<f64>>,

    #[serde(default)]
    pub wrap_with_joint_position_difference_limiter: bool,
    pub joint_position_difference_limits: Option<Vec<f64>>,

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

pub fn wrap_joint_trajectory_client(
    config: JointTrajectoryClientWrapperConfig,
    client: Arc<dyn JointTrajectoryClient>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<Arc<dyn JointTrajectoryClient>, arci::Error> {
    Ok(if config.wrap_with_joint_velocity_limiter {
        if config.wrap_with_joint_position_limiter {
            if config.wrap_with_joint_position_difference_limiter {
                let client = new_joint_position_difference_limiter(
                    client,
                    config.joint_position_difference_limits,
                )?;
                let client =
                    new_joint_velocity_limiter(client, config.joint_velocity_limits, urdf_robot)?;
                let client = new_joint_position_limiter(
                    client,
                    config.joint_position_limits,
                    config.joint_position_limiter_strategy,
                    urdf_robot,
                )?;
                Arc::new(client)
            } else {
                let client =
                    new_joint_velocity_limiter(client, config.joint_velocity_limits, urdf_robot)?;
                let client = new_joint_position_limiter(
                    client,
                    config.joint_position_limits,
                    config.joint_position_limiter_strategy,
                    urdf_robot,
                )?;
                Arc::new(client)
            }
        } else if config.wrap_with_joint_position_difference_limiter {
            Arc::new(new_joint_velocity_limiter(
                new_joint_position_difference_limiter(
                    client,
                    config.joint_position_difference_limits,
                )?,
                config.joint_velocity_limits,
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
        if config.wrap_with_joint_position_difference_limiter {
            Arc::new(new_joint_position_limiter(
                new_joint_position_difference_limiter(
                    client,
                    config.joint_position_difference_limits,
                )?,
                config.joint_position_limits,
                config.joint_position_limiter_strategy,
                urdf_robot,
            )?)
        } else {
            Arc::new(new_joint_position_limiter(
                client,
                config.joint_position_limits,
                config.joint_position_limiter_strategy,
                urdf_robot,
            )?)
        }
    } else if config.wrap_with_joint_position_difference_limiter {
        Arc::new(new_joint_position_difference_limiter(
            client,
            config.joint_position_difference_limits,
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

        let result = new_joint_position_limiter(
            client,
            Some(limits.clone()),
            JointPositionLimiterStrategy::Clamp,
            None,
        );
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

        let _ = new_joint_position_limiter(client, None, JointPositionLimiterStrategy::Clamp, None);
    }
}
