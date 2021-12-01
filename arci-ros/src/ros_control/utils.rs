use std::{collections::HashMap, sync::Arc, time::Duration};

use anyhow::format_err;
use arci::{copy_joint_positions, Error, JointTrajectoryClient, TrajectoryPoint};

use crate::{
    msg::trajectory_msgs::{JointTrajectory, JointTrajectoryPoint},
    wrap_joint_trajectory_client, JointStateProvider, LazyJointStateProvider,
    RosControlClientBuilder,
};

pub(crate) fn extract_current_joint_positions_from_state(
    client: &dyn JointTrajectoryClient,
    state_provider: &dyn JointStateProvider,
) -> Result<Vec<f64>, Error> {
    let (state_joint_names, state_joint_positions) = state_provider.get_joint_state()?;
    // TODO: cache index map and use it
    let mut result = vec![0.0; client.joint_names().len()];
    copy_joint_positions(
        &state_joint_names,
        &state_joint_positions,
        &client.joint_names(),
        &mut result,
    )?;
    Ok(result)
}

pub(crate) fn create_joint_trajectory_message_for_send_joint_positions(
    client: &dyn JointTrajectoryClient,
    state_provider: &dyn JointStateProvider,
    positions: &[f64],
    duration: Duration,
    send_partial_joints_goal: bool,
) -> Result<JointTrajectory, arci::Error> {
    if send_partial_joints_goal {
        Ok(JointTrajectory {
            points: vec![JointTrajectoryPoint {
                positions: positions.to_owned(),
                // add zero velocity to use cubic interpolation in trajectory_controller
                velocities: vec![0.0; client.joint_names().len()],
                time_from_start: duration.into(),
                ..Default::default()
            }],
            joint_names: client.joint_names(),
            ..Default::default()
        })
    } else {
        let (state_joint_names, state_joint_positions) = state_provider.get_joint_state()?;
        let partial_names = client.joint_names();
        if partial_names.len() != positions.len() {
            return Err(arci::Error::LengthMismatch {
                model: partial_names.len(),
                input: positions.len(),
            });
        }
        // TODO: cache index map and use it
        let full_names = state_joint_names;
        let full_dof = full_names.len();

        let mut full_positions = state_joint_positions;
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
}

pub(crate) fn create_joint_trajectory_message_for_send_joint_trajectory(
    client: &dyn JointTrajectoryClient,
    state_provider: &dyn JointStateProvider,
    trajectory: &[TrajectoryPoint],
    send_partial_joints_goal: bool,
) -> Result<JointTrajectory, arci::Error> {
    if send_partial_joints_goal {
        Ok(JointTrajectory {
            points: trajectory
                .iter()
                .map(|tp| JointTrajectoryPoint {
                    positions: tp.positions.clone(),
                    velocities: if let Some(velocities) = &tp.velocities {
                        velocities.clone()
                    } else {
                        vec![]
                    },
                    time_from_start: tp.time_from_start.into(),
                    ..Default::default()
                })
                .collect(),
            joint_names: client.joint_names(),
            ..Default::default()
        })
    } else {
        let (state_joint_names, state_joint_positions) = state_provider.get_joint_state()?;
        // TODO: cache index map and use it
        let current_full_positions = state_joint_positions;
        let full_names = state_joint_names;
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
}

/// Returns a map of clients for each builder.
///
/// The key for the map is [the name of the client](RosControlClientBuilder::name),
/// and in case of conflict, it becomes an error.
///
/// Returns empty map when `builders` are empty.
pub fn create_joint_trajectory_clients<B>(
    builders: Vec<B>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error>
where
    B: RosControlClientBuilder,
{
    create_joint_trajectory_clients_inner(builders, urdf_robot, false)
}

/// Returns a map of clients that will be created lazily for each builder.
///
/// See [create_joint_trajectory_clients] for more.
pub fn create_joint_trajectory_clients_lazy<B>(
    builders: Vec<B>,
    urdf_robot: Option<&urdf_rs::Robot>,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error>
where
    B: RosControlClientBuilder,
{
    create_joint_trajectory_clients_inner(builders, urdf_robot, true)
}

fn create_joint_trajectory_clients_inner<B>(
    builders: Vec<B>,
    urdf_robot: Option<&urdf_rs::Robot>,
    lazy: bool,
) -> Result<HashMap<String, Arc<dyn JointTrajectoryClient>>, arci::Error>
where
    B: RosControlClientBuilder,
{
    if builders.is_empty() {
        return Ok(HashMap::default());
    }

    let mut clients = HashMap::new();
    let mut state_topic_name_to_provider: HashMap<String, Arc<LazyJointStateProvider>> =
        HashMap::new();
    for builder in builders {
        if urdf_robot.is_none() {
            builder.wrapper_config().check_urdf_is_not_necessary()?;
        }
        let state_topic_name = builder.state_topic();
        let joint_state_provider = if let Some(joint_state_provider) =
            state_topic_name_to_provider.get(&state_topic_name)
        {
            joint_state_provider.clone()
        } else {
            let joint_state_provider = builder.build_joint_state_provider(&state_topic_name);
            state_topic_name_to_provider.insert(state_topic_name, joint_state_provider.clone());
            joint_state_provider
        };

        let name = builder.name().to_owned();
        let client = builder.build_joint_trajectory_client(lazy, joint_state_provider)?;
        let client =
            wrap_joint_trajectory_client(builder.wrapper_config().clone(), client, urdf_robot)?;
        if clients.insert(name.clone(), client).is_some() {
            return Err(format_err!("client named '{}' has already been specified", name).into());
        }
    }
    Ok(clients)
}
