use std::{f64, ops::RangeInclusive};

use tracing::debug;
use urdf_rs::JointType;

use crate::{
    error::Error,
    traits::{JointTrajectoryClient, TrajectoryPoint},
    waits::WaitFuture,
};

pub struct JointPositionLimiter<C>
where
    C: JointTrajectoryClient,
{
    client: C,
    limits: Vec<RangeInclusive<f64>>,
    strategy: JointPositionLimiterStrategy,
}

impl<C> JointPositionLimiter<C>
where
    C: JointTrajectoryClient,
{
    pub fn new(client: C, limits: Vec<RangeInclusive<f64>>) -> Self {
        Self::new_with_strategy(client, limits, Default::default())
    }

    pub fn new_with_strategy(
        client: C,
        limits: Vec<RangeInclusive<f64>>,
        strategy: JointPositionLimiterStrategy,
    ) -> Self {
        assert!(client.joint_names().len() == limits.len());
        Self {
            client,
            limits,
            strategy,
        }
    }

    pub fn from_urdf(client: C, joints: &[urdf_rs::Joint]) -> Result<Self, Error> {
        Self::from_urdf_with_strategy(client, joints, Default::default())
    }

    pub fn from_urdf_with_strategy(
        client: C,
        joints: &[urdf_rs::Joint],
        strategy: JointPositionLimiterStrategy,
    ) -> Result<Self, Error> {
        let mut limits = Vec::new();
        for joint_name in client.joint_names() {
            if let Some(i) = joints.iter().position(|j| j.name == *joint_name) {
                let joint = &joints[i];
                let limit = if JointType::Continuous == joint.joint_type {
                    // If limit is not specified, urdf-rs assigns f64::default.
                    -f64::consts::PI..=f64::consts::PI
                } else {
                    joint.limit.lower..=joint.limit.upper
                };
                limits.push(limit);
            } else {
                return Err(Error::NoJoint(joint_name.into()));
            }
        }

        Ok(Self {
            client,
            limits,
            strategy,
        })
    }

    pub fn set_strategy(&mut self, strategy: JointPositionLimiterStrategy) {
        self.strategy = strategy;
    }

    fn check_joint_position(&self, positions: &mut Vec<f64>) -> Result<(), Error> {
        assert_eq!(positions.len(), self.limits.len());
        for (i, (limit, position)) in self.limits.iter().zip(positions).enumerate() {
            if limit.contains(&position) {
                continue;
            }
            match self.strategy {
                JointPositionLimiterStrategy::Error => {
                    return Err(Error::OutOfLimit {
                        name: self.client.joint_names()[i].clone(),
                        position: *position,
                        limit: limit.clone(),
                    });
                }
                JointPositionLimiterStrategy::Clamp => {
                    debug!(
                        "Out of limit: joint={}, position={}, limit={:?}",
                        self.client.joint_names()[i],
                        position,
                        limit,
                    );
                    *position = position.clamp(*limit.start(), *limit.end());
                }
            }
        }
        Ok(())
    }
}

impl<C> JointTrajectoryClient for JointPositionLimiter<C>
where
    C: JointTrajectoryClient,
{
    fn joint_names(&self) -> &[String] {
        self.client.joint_names()
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        self.client.current_joint_positions()
    }

    fn send_joint_positions(
        &self,
        mut positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        self.check_joint_position(&mut positions)?;
        self.client.send_joint_positions(positions, duration)
    }

    fn send_joint_trajectory(
        &self,
        mut trajectory: Vec<TrajectoryPoint>,
    ) -> Result<WaitFuture, Error> {
        for tp in &mut trajectory {
            self.check_joint_position(&mut tp.positions)?;
        }
        self.client.send_joint_trajectory(trajectory)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum JointPositionLimiterStrategy {
    /// If the position is out of the limit, handle it as the same value as the limit.
    Clamp,
    /// If the position is out of the limit, return an error.
    Error,
}

impl Default for JointPositionLimiterStrategy {
    fn default() -> Self {
        Self::Clamp
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use assert_approx_eq::assert_approx_eq;

    use super::*;
    use crate::DummyJointTrajectoryClient;

    const SECOND: Duration = Duration::from_secs(1);

    #[test]
    #[should_panic]
    fn mismatch_size() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        JointPositionLimiter::new(client, vec![1.0..=2.0, 2.0..=3.0]);
    }

    #[test]
    fn joint_names() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let limiter = JointPositionLimiter::new(client, vec![1.0..=2.0, 2.0..=3.0]);
        let joint_names = limiter.joint_names();
        assert_eq!(joint_names.len(), 2);
        assert_eq!(joint_names[0], "a");
        assert_eq!(joint_names[1], "b");
    }

    #[tokio::test]
    async fn send_joint_positions_none_limited() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let mut client = JointPositionLimiter::new(client, vec![1.0..=2.0]);

        for &strategy in &[
            JointPositionLimiterStrategy::Clamp,
            JointPositionLimiterStrategy::Error,
        ] {
            client.set_strategy(strategy);

            client
                .send_joint_positions(vec![1.0], SECOND)
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.0);

            client
                .send_joint_positions(vec![2.0], SECOND)
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
        }
    }

    #[tokio::test]
    async fn send_joint_positions_limited_rounded() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new(client, vec![1.0..=2.0]);

        client
            .send_joint_positions(vec![0.0], SECOND)
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.0);

        client
            .send_joint_positions(vec![3.0], SECOND)
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
    }

    #[tokio::test]
    async fn send_joint_positions_limited_error() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new_with_strategy(
            client,
            vec![1.0..=2.0],
            JointPositionLimiterStrategy::Error,
        );

        let e = client
            .send_joint_positions(vec![0.0], SECOND)
            .err()
            .unwrap();
        assert_error(e, 0.0);

        let e = client
            .send_joint_positions(vec![3.0], SECOND)
            .err()
            .unwrap();
        assert_error(e, 3.0);
    }

    #[tokio::test]
    async fn send_joint_trajectory_none_limited() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let mut client = JointPositionLimiter::new(client, vec![1.0..=2.0]);

        for &strategy in &[
            JointPositionLimiterStrategy::Clamp,
            JointPositionLimiterStrategy::Error,
        ] {
            client.set_strategy(strategy);

            client
                .send_joint_trajectory(vec![
                    TrajectoryPoint::new(vec![1.0], SECOND * 2),
                    TrajectoryPoint::new(vec![1.5], SECOND * 3),
                ])
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.5);

            client
                .send_joint_trajectory(vec![
                    TrajectoryPoint::new(vec![1.7], SECOND * 2),
                    TrajectoryPoint::new(vec![2.0], SECOND * 3),
                ])
                .unwrap()
                .await
                .unwrap();
            assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
        }
    }

    #[tokio::test]
    async fn send_joint_trajectory_limited_rounded() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new(client, vec![1.0..=2.0]);

        client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![0.0], SECOND * 2),
                TrajectoryPoint::new(vec![0.5], SECOND * 3),
            ])
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 1.0);

        client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![2.5], SECOND * 2),
                TrajectoryPoint::new(vec![3.0], SECOND * 3),
            ])
            .unwrap()
            .await
            .unwrap();
        assert_approx_eq!(client.current_joint_positions().unwrap()[0], 2.0);
    }

    #[tokio::test]
    async fn send_joint_trajectory_limited_error() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let client = JointPositionLimiter::new_with_strategy(
            client,
            vec![1.0..=2.0],
            JointPositionLimiterStrategy::Error,
        );

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![0.0], SECOND * 2),
                TrajectoryPoint::new(vec![0.5], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 0.0);

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![1.0], SECOND * 2),
                TrajectoryPoint::new(vec![0.5], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 0.5);

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![2.5], SECOND * 2),
                TrajectoryPoint::new(vec![3.0], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 2.5);

        let e = client
            .send_joint_trajectory(vec![
                TrajectoryPoint::new(vec![2.0], SECOND * 2),
                TrajectoryPoint::new(vec![3.0], SECOND * 3),
            ])
            .err()
            .unwrap();
        assert_error(e, 3.0);
    }

    fn assert_error(e: Error, position: f64) {
        match e {
            Error::OutOfLimit { position: p, .. } => assert_approx_eq!(p, position),
            _ => panic!("{:?}", e),
        }
    }
}
