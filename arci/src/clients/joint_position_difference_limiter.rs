use std::time::Duration;

use crate::{error::Error, traits::JointTrajectoryClient, TrajectoryPoint, WaitFuture};

/// JointPositionDifferenceLimiter limits the difference of position between trajectory points in JointTrajectoryClient::send_joint_positions.
/// In send_joint_trajectory, simply input trajectory is forwarded to client.

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
            &duration,
        )? {
            Some(trajectory) => self.client.send_joint_trajectory(trajectory),
            None => self.client.send_joint_positions(positions, duration),
        }
    }

    /// Simply input trajectory is forwarded to client. Design trajectory properly.
    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        self.client.send_joint_trajectory(trajectory)
    }
}

fn interpolate(
    mut current: Vec<f64>,
    position_difference_limits: &[f64],
    positions: &[f64],
    duration: &Duration,
) -> Result<Option<Vec<TrajectoryPoint>>, Error> {
    let mut max_diff_step: f64 = 0.0;
    let mut diff = vec![0.0; current.len()];
    for (i, p) in current.iter().enumerate() {
        diff[i] = positions[i] - p;
        let step = diff[i].abs() / position_difference_limits[i].abs();
        if step.is_infinite() {
            return Err(Error::Other(anyhow::format_err!(
                "Invalid position difference limits {} for joint {} ",
                position_difference_limits[i],
                i,
            )));
        }
        max_diff_step = max_diff_step.max(step);
    }
    let max_diff_step = max_diff_step.ceil() as usize;
    Ok(if max_diff_step <= 1 {
        None
    } else {
        diff.iter_mut().for_each(|d| *d /= max_diff_step as f64);
        let step_duration = Duration::from_secs_f64(duration.as_secs_f64() / max_diff_step as f64);
        let mut trajectory = vec![];
        for i in 1..max_diff_step {
            current
                .iter_mut()
                .enumerate()
                .for_each(|(i, c)| *c += diff[i]);
            trajectory.push(TrajectoryPoint {
                positions: current.to_owned(),
                velocities: None,
                time_from_start: step_duration * i as u32,
            })
        }
        trajectory.push(TrajectoryPoint {
            positions: positions.to_vec(),
            velocities: None,
            time_from_start: *duration,
        });
        Some(trajectory)
    })
}

#[cfg(test)]
mod test {
    use std::{sync::Arc, time::Duration};

    use assert_approx_eq::assert_approx_eq;

    use super::{
        interpolate, JointPositionDifferenceLimiter, JointTrajectoryClient, TrajectoryPoint,
    };
    use crate::DummyJointTrajectoryClient;

    #[test]
    fn interpolate_no_interpolation() {
        let interpolated = interpolate(
            vec![0.0, 1.0],
            &[1.0, 1.0],
            &[-1.0, 2.0],
            &Duration::from_secs(1),
        );
        assert!(interpolated.is_ok());
        assert!(interpolated.unwrap().is_none());
        assert!(interpolate(
            vec![0.0, 1.0],
            &[0.0, 0.0],
            &[-1.0, 2.0],
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
        for (c, w) in trajectory
            .iter()
            .zip(wrapped_client.last_trajectory.lock().clone().iter())
        {
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
        *wrapped_client.positions.lock() = vec![0.0, 1.0];
        assert!(tokio_test::block_on(
            client
                .send_joint_positions(vec![-1.0, 2.0], Duration::from_secs(1))
                .unwrap()
        )
        .is_ok());
        assert!(wrapped_client.last_trajectory.lock().is_empty());
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
        *wrapped_client.positions.lock() = vec![0.0, 1.0];
        assert!(tokio_test::block_on(
            client
                .send_joint_positions(vec![-1.0, 2.0], Duration::from_secs(1))
                .unwrap()
        )
        .is_ok());
        let actual_trajectory = wrapped_client.last_trajectory.lock().clone();
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
}
