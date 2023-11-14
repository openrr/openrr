use std::time::Duration;

use crate::{error::Error, traits::JointTrajectoryClient, TrajectoryPoint, WaitFuture};

const ZERO_VELOCITY_THRESHOLD: f64 = 1.0e-6;

/// JointPositionDifferenceLimiter limits the difference of position between trajectory points and
///  trajectory points are interpolated linearly to satisfy the limits
///  in JointTrajectoryClient::send_joint_positions.
/// In send_joint_trajectory, if no velocities is specified or zero velocities is specified at the
///  last point, trajectory points are interpolated, otherwise simply input trajectory is forwarded to client.

pub struct JointPositionDifferenceLimiter<C>
where
    C: JointTrajectoryClient,
{
    client: C,
    position_difference_limits: Vec<f64>,
}

impl<C> JointPositionDifferenceLimiter<C>
where
    C: JointTrajectoryClient,
{
    /// Create a new `JointPositionDifferenceLimiter` with the given position difference limits.
    pub fn new(client: C, mut position_difference_limits: Vec<f64>) -> Result<Self, Error> {
        if client.joint_names().len() == position_difference_limits.len() {
            let mut is_valid = true;
            position_difference_limits.iter_mut().for_each(|f| {
                let f_abs = f.abs();
                if f_abs < f64::MIN_POSITIVE {
                    is_valid = false;
                }
                *f = f_abs
            });
            if !is_valid {
                return Err(Error::Other(anyhow::format_err!(
                    "Too small position difference limit",
                )));
            }
            Ok(Self {
                client,
                position_difference_limits,
            })
        } else {
            Err(Error::LengthMismatch {
                model: client.joint_names().len(),
                input: position_difference_limits.len(),
            })
        }
    }
}

impl<C> JointTrajectoryClient for JointPositionDifferenceLimiter<C>
where
    C: JointTrajectoryClient,
{
    fn joint_names(&self) -> Vec<String> {
        self.client.joint_names()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        self.client.current_joint_positions()
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, Error> {
        let current = self.client.current_joint_positions()?;
        if current.len() != positions.len() {
            return Err(Error::LengthMismatch {
                model: positions.len(),
                input: current.len(),
            });
        }
        match interpolate(
            current,
            &self.position_difference_limits,
            &positions,
            &Duration::from_secs(0),
            &duration,
        )? {
            Some(trajectory) => self.client.send_joint_trajectory(trajectory),
            None => self.client.send_joint_positions(positions, duration),
        }
    }

    /// If no velocities is specified or zero velocities is specified at the last point,
    /// trajectory points are interpolated, otherwise simply input trajectory is forwarded to client.
    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        let fixed_trajectory = if should_interpolate_joint_trajectory(&trajectory) {
            interpolate_joint_trajectory(
                self.client.current_joint_positions()?,
                &self.position_difference_limits,
                trajectory,
            )?
        } else {
            trajectory
        };
        self.client.send_joint_trajectory(fixed_trajectory)
    }
}

fn interpolate(
    mut current: Vec<f64>,
    position_difference_limits: &[f64],
    positions: &[f64],
    first_time_from_start: &Duration,
    last_time_from_start: &Duration,
) -> Result<Option<Vec<TrajectoryPoint>>, Error> {
    let mut max_diff_step: f64 = 0.0;
    let mut diff = vec![0.0; current.len()];
    for (i, p) in current.iter().enumerate() {
        diff[i] = positions[i] - p;
        let step = diff[i].abs() / position_difference_limits[i].abs();
        if step.is_infinite() {
            return Err(Error::Other(anyhow::format_err!(
                "Invalid position difference limits {} for joint {i} ",
                position_difference_limits[i],
            )));
        }
        max_diff_step = max_diff_step.max(step);
    }
    let max_diff_step = max_diff_step.ceil() as usize;
    Ok(if max_diff_step <= 1 {
        None
    } else {
        diff.iter_mut().for_each(|d| *d /= max_diff_step as f64);
        let step_duration = Duration::from_secs_f64(
            (last_time_from_start.as_secs_f64() - first_time_from_start.as_secs_f64())
                / max_diff_step as f64,
        );
        let mut trajectory = vec![];
        for i in 1..max_diff_step {
            current
                .iter_mut()
                .enumerate()
                .for_each(|(i, c)| *c += diff[i]);
            trajectory.push(TrajectoryPoint {
                positions: current.to_owned(),
                velocities: None,
                time_from_start: *first_time_from_start + step_duration * i as u32,
            })
        }
        trajectory.push(TrajectoryPoint {
            positions: positions.to_vec(),
            velocities: None,
            time_from_start: *last_time_from_start,
        });
        Some(trajectory)
    })
}

fn should_interpolate_joint_trajectory(trajectory: &[TrajectoryPoint]) -> bool {
    if trajectory.is_empty() {
        return false;
    };
    match trajectory.iter().position(|p| p.velocities.is_some()) {
        Some(first_index_of_valid_velocity) => {
            let last_index = trajectory.len() - 1;
            if first_index_of_valid_velocity != last_index {
                false
            } else {
                return !trajectory[last_index]
                    .velocities
                    .as_ref()
                    .unwrap()
                    .iter()
                    .any(|x| x.abs() > ZERO_VELOCITY_THRESHOLD);
            }
        }
        None => true,
    }
}

fn interpolate_joint_trajectory(
    current: Vec<f64>,
    position_difference_limits: &[f64],
    trajectory: Vec<TrajectoryPoint>,
) -> Result<Vec<TrajectoryPoint>, Error> {
    let mut fixed_trajectory = vec![];
    let mut previous_joint_positions = current;
    let mut previous_time_from_start = Duration::from_secs(0);

    for p in trajectory {
        let target = p.positions.clone();
        let velocity = p.velocities.clone();
        let time_from_start = p.time_from_start;
        fixed_trajectory.extend(
            match interpolate(
                previous_joint_positions,
                position_difference_limits,
                &target,
                &previous_time_from_start,
                &time_from_start,
            )? {
                Some(mut interpolated) => {
                    interpolated.last_mut().unwrap().velocities = velocity;
                    interpolated
                }
                None => vec![p],
            },
        );

        previous_joint_positions = target;
        previous_time_from_start = time_from_start;
    }
    Ok(fixed_trajectory)
}

#[cfg(test)]
mod test {
    use std::{sync::Arc, time::Duration};

    use assert_approx_eq::assert_approx_eq;

    use super::{
        interpolate, interpolate_joint_trajectory, should_interpolate_joint_trajectory,
        JointPositionDifferenceLimiter, JointTrajectoryClient, TrajectoryPoint,
    };
    use crate::DummyJointTrajectoryClient;

    #[test]
    fn interpolate_no_interpolation() {
        let interpolated = interpolate(
            vec![0.0, 1.0],
            &[1.0, 1.0],
            &[-1.0, 2.0],
            &Duration::from_secs(0),
            &Duration::from_secs(1),
        );
        assert!(interpolated.is_ok());
        assert!(interpolated.unwrap().is_none());
        assert!(interpolate(
            vec![0.0, 1.0],
            &[0.0, 0.0],
            &[-1.0, 2.0],
            &Duration::from_secs(0),
            &Duration::from_secs(1),
        )
        .is_err());
    }
    #[test]
    fn interpolate_interpolated() {
        let interpolated = interpolate(
            vec![0.0, 1.0],
            &[1.0, 0.5],
            &[-1.0, 2.0],
            &Duration::from_secs(0),
            &Duration::from_secs(1),
        );
        assert!(interpolated.is_ok());
        let interpolated = interpolated.unwrap();
        assert!(interpolated.is_some());
        let interpolated = interpolated.unwrap();
        assert_eq!(interpolated.len(), 2);
        assert_eq!(interpolated[0].positions, vec![-0.5, 1.5]);
        assert!(interpolated[0].velocities.is_none());
        assert_approx_eq!(interpolated[0].time_from_start.as_secs_f64(), 0.5);

        assert_eq!(interpolated[1].positions, vec![-1.0, 2.0]);
        assert!(interpolated[1].velocities.is_none());
        assert_approx_eq!(interpolated[1].time_from_start.as_secs_f64(), 1.0);
    }

    #[test]
    fn joint_position_difference_limiter_new_error() {
        let wrapped_client = Arc::new(DummyJointTrajectoryClient::new(vec![
            "a".to_owned(),
            "b".to_owned(),
        ]));
        assert!(
            JointPositionDifferenceLimiter::new(wrapped_client.clone(), vec![3.0, 1.0, 2.0])
                .is_err()
        );
        assert!(JointPositionDifferenceLimiter::new(wrapped_client, vec![1.0, 0.0]).is_err());
    }
    #[test]
    fn joint_position_difference_limiter_send_joint_trajectory() {
        let wrapped_client = Arc::new(DummyJointTrajectoryClient::new(vec![
            "a".to_owned(),
            "b".to_owned(),
        ]));
        let client = JointPositionDifferenceLimiter::new(wrapped_client.clone(), vec![1.0, 2.0]);
        assert!(client.is_ok());
        let client = client.unwrap();
        assert_eq!(
            client.joint_names().len(),
            wrapped_client.joint_names().len()
        );
        for (c, w) in client
            .joint_names()
            .iter()
            .zip(wrapped_client.joint_names().iter())
        {
            assert_eq!(c, w);
        }

        let trajectory = vec![
            TrajectoryPoint {
                positions: vec![1.0, 2.0],
                velocities: Some(vec![3.0, 4.0]),
                time_from_start: std::time::Duration::from_secs_f64(4.0),
            },
            TrajectoryPoint {
                positions: vec![3.0, 6.0],
                velocities: Some(vec![3.0, 4.0]),
                time_from_start: std::time::Duration::from_secs_f64(8.0),
            },
        ];
        assert!(
            tokio_test::block_on(client.send_joint_trajectory(trajectory.clone()).unwrap()).is_ok()
        );
        for (c, w) in trajectory.iter().zip(
            wrapped_client
                .last_trajectory
                .lock()
                .unwrap()
                .clone()
                .iter(),
        ) {
            assert_eq!(c.positions, w.positions);
            assert_eq!(c.velocities, w.velocities);
            assert_eq!(c.time_from_start, w.time_from_start);
        }
        assert_eq!(
            trajectory.last().unwrap().positions,
            client.current_joint_positions().unwrap()
        );
    }
    #[test]
    fn joint_position_difference_limiter_send_joint_positions_no_interpolation() {
        let wrapped_client = Arc::new(DummyJointTrajectoryClient::new(vec![
            "a".to_owned(),
            "b".to_owned(),
        ]));
        let client = JointPositionDifferenceLimiter::new(wrapped_client.clone(), vec![1.0, 1.0]);
        assert!(client.is_ok());
        let client = client.unwrap();
        assert_eq!(
            client.joint_names().len(),
            wrapped_client.joint_names().len()
        );
        for (c, w) in client
            .joint_names()
            .iter()
            .zip(wrapped_client.joint_names().iter())
        {
            assert_eq!(c, w);
        }
        *wrapped_client.positions.lock().unwrap() = vec![0.0, 1.0];
        assert!(tokio_test::block_on(
            client
                .send_joint_positions(vec![-1.0, 2.0], Duration::from_secs(1))
                .unwrap()
        )
        .is_ok());
        assert!(wrapped_client.last_trajectory.lock().unwrap().is_empty());
        assert_eq!(
            wrapped_client.current_joint_positions().unwrap(),
            vec![-1.0, 2.0]
        );
    }
    #[test]
    fn joint_position_difference_limiter_send_joint_positions_interpolated() {
        let wrapped_client = Arc::new(DummyJointTrajectoryClient::new(vec![
            "a".to_owned(),
            "b".to_owned(),
        ]));
        let client = JointPositionDifferenceLimiter::new(wrapped_client.clone(), vec![1.0, -0.5]);
        assert!(client.is_ok());
        let client = client.unwrap();
        assert_eq!(
            client.joint_names().len(),
            wrapped_client.joint_names().len()
        );
        for (c, w) in client
            .joint_names()
            .iter()
            .zip(wrapped_client.joint_names().iter())
        {
            assert_eq!(c, w);
        }
        *wrapped_client.positions.lock().unwrap() = vec![0.0, 1.0];
        assert!(tokio_test::block_on(
            client
                .send_joint_positions(vec![-1.0, 2.0], Duration::from_secs(1))
                .unwrap()
        )
        .is_ok());
        let actual_trajectory = wrapped_client.last_trajectory.lock().unwrap().clone();
        assert_eq!(actual_trajectory.len(), 2);
        assert_eq!(actual_trajectory[0].positions, vec![-0.5, 1.5]);
        assert_eq!(actual_trajectory[1].positions, vec![-1.0, 2.0]);
        assert!(actual_trajectory[0].velocities.is_none());
        assert!(actual_trajectory[1].velocities.is_none());
        assert_approx_eq!(actual_trajectory[0].time_from_start.as_secs_f64(), 0.5);
        assert_approx_eq!(actual_trajectory[1].time_from_start.as_secs_f64(), 1.0);

        assert_eq!(
            wrapped_client.current_joint_positions().unwrap(),
            vec![-1.0, 2.0]
        );
    }

    #[test]
    fn test_should_interpolate_joint_trajectory() {
        assert!(!should_interpolate_joint_trajectory(&[]));
        assert!(!should_interpolate_joint_trajectory(&[
            TrajectoryPoint {
                positions: vec![],
                velocities: Some(vec![0.0, 0.0]),
                time_from_start: std::time::Duration::from_secs(0),
            },
            TrajectoryPoint {
                positions: vec![],
                velocities: Some(vec![0.0, 0.0]),
                time_from_start: std::time::Duration::from_secs(0),
            }
        ]));
        assert!(!should_interpolate_joint_trajectory(&[
            TrajectoryPoint {
                positions: vec![],
                velocities: None,
                time_from_start: std::time::Duration::from_secs(0),
            },
            TrajectoryPoint {
                positions: vec![],
                velocities: Some(vec![0.0, 0.01]),
                time_from_start: std::time::Duration::from_secs(0),
            }
        ]));
        assert!(should_interpolate_joint_trajectory(&[
            TrajectoryPoint {
                positions: vec![],
                velocities: None,
                time_from_start: std::time::Duration::from_secs(0),
            },
            TrajectoryPoint {
                positions: vec![],
                velocities: Some(vec![0.0, 0.0]),
                time_from_start: std::time::Duration::from_secs(0),
            }
        ]));
    }

    #[test]
    fn test_should_interpolate_joint_trajectory_no_interpolation() {
        let interpolated = interpolate_joint_trajectory(
            vec![0.0, 1.0],
            &[1.0, 1.0],
            vec![
                TrajectoryPoint {
                    positions: vec![-1.0, 2.0],
                    velocities: None,
                    time_from_start: std::time::Duration::from_secs(1),
                },
                TrajectoryPoint {
                    positions: vec![-2.0, 3.0],
                    velocities: Some(vec![0.0, 0.0]),
                    time_from_start: std::time::Duration::from_secs(2),
                },
            ],
        );
        assert!(interpolated.is_ok());
        let interpolated = interpolated.unwrap();

        assert_eq!(interpolated.len(), 2);

        assert_eq!(interpolated[0].positions, vec![-1.0, 2.0]);
        assert!(interpolated[0].velocities.is_none());
        assert_approx_eq!(interpolated[0].time_from_start.as_secs_f64(), 1.0);

        assert_eq!(interpolated[1].positions, vec![-2.0, 3.0]);
        assert!(interpolated[1].velocities.is_some());
        assert_approx_eq!(interpolated[1].time_from_start.as_secs_f64(), 2.0);
    }

    #[test]
    fn test_should_interpolate_joint_trajectory_interpolated() {
        let interpolated = interpolate_joint_trajectory(
            vec![0.0, 1.0],
            &[1.0, 0.5],
            vec![
                TrajectoryPoint {
                    positions: vec![-1.0, 2.0],
                    velocities: None,
                    time_from_start: std::time::Duration::from_secs(1),
                },
                TrajectoryPoint {
                    positions: vec![-2.0, 3.0],
                    velocities: Some(vec![0.0, 0.0]),
                    time_from_start: std::time::Duration::from_secs(2),
                },
            ],
        );
        assert!(interpolated.is_ok());
        let interpolated = interpolated.unwrap();

        assert_eq!(interpolated.len(), 4);

        assert_eq!(interpolated[0].positions, vec![-0.5, 1.5]);
        assert!(interpolated[0].velocities.is_none());
        assert_approx_eq!(interpolated[0].time_from_start.as_secs_f64(), 0.5);

        assert_eq!(interpolated[1].positions, vec![-1.0, 2.0]);
        assert!(interpolated[1].velocities.is_none());
        assert_approx_eq!(interpolated[1].time_from_start.as_secs_f64(), 1.0);

        assert_eq!(interpolated[2].positions, vec![-1.5, 2.5]);
        assert!(interpolated[2].velocities.is_none());
        assert_approx_eq!(interpolated[2].time_from_start.as_secs_f64(), 1.5);

        assert_eq!(interpolated[3].positions, vec![-2.0, 3.0]);
        assert!(interpolated[3].velocities.is_some());
        assert_approx_eq!(interpolated[3].time_from_start.as_secs_f64(), 2.0);
    }
}
