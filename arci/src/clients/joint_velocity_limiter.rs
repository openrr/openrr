use tracing::debug;

use crate::{
    error::Error,
    traits::{JointTrajectoryClient, TrajectoryPoint},
    waits::WaitFuture,
};

/// JointVelocityLimiter limits the duration to make all joints velocities lower than the given
/// velocities limits at each TrajectoryPoint.
///
/// It does not change TrajectoryPoint velocities.
/// The duration for a TrajectoryPoint\[i\] is set to
/// ```Text
/// duration[i] = max(limited_duration_i[j=0], ...,  limited_duration_i[j=J-1], input_duration[i])
/// where
///  j : joint_index (0 <= j < J),
///  limited_duration_i[j] =
///   abs(TrajectoryPoint[i].positions[j]  - TrajectoryPoint[i-1].positions[j]) / velocity_limits[j]
/// ```
pub struct JointVelocityLimiter<C>
where
    C: JointTrajectoryClient,
{
    client: C,
    velocity_limits: Vec<f64>,
}

impl<C> JointVelocityLimiter<C>
where
    C: JointTrajectoryClient,
{
    /// Creates a new `JointVelocityLimiter` with the given velocity limits.
    ///
    /// # Panics
    ///
    /// Panics if the lengths of `velocity_limits` and joints that `client` handles are different.
    #[track_caller]
    pub fn new(client: C, velocity_limits: Vec<f64>) -> Self {
        assert!(client.joint_names().len() == velocity_limits.len());
        Self {
            client,
            velocity_limits,
        }
    }

    /// Creates a new `JointVelocityLimiter` with the velocity limits defined in URDF.
    pub fn from_urdf(client: C, joints: &[urdf_rs::Joint]) -> Result<Self, Error> {
        let mut velocity_limits = Vec::new();
        for joint_name in client.joint_names() {
            if let Some(i) = joints.iter().position(|j| j.name == *joint_name) {
                let limit = joints[i].limit.velocity;
                velocity_limits.push(limit);
            } else {
                return Err(Error::NoJoint(joint_name));
            }
        }

        Ok(Self {
            client,
            velocity_limits,
        })
    }
}

impl<C> JointTrajectoryClient for JointVelocityLimiter<C>
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
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        self.send_joint_trajectory(vec![TrajectoryPoint {
            positions,
            velocities: None,
            time_from_start: duration,
        }])
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        let mut prev_positions = self.current_joint_positions()?;

        let mut limited_trajectory = vec![];
        let mut limited_duration_from_start = std::time::Duration::from_secs(0);
        let mut original_duration_from_start = std::time::Duration::from_secs(0);
        for (sequence_index, original_trajectory_point) in trajectory.iter().enumerate() {
            let mut limited_duration_from_prev = std::time::Duration::from_secs(0);
            let mut dominant_joint_index = 0;
            for (joint_index, prev_position) in prev_positions.iter().enumerate() {
                let single_duration = std::time::Duration::from_secs_f64(
                    (prev_position - original_trajectory_point.positions[joint_index]).abs()
                        / self.velocity_limits[joint_index],
                );
                limited_duration_from_prev = if single_duration > limited_duration_from_prev {
                    dominant_joint_index = joint_index;
                    single_duration
                } else {
                    limited_duration_from_prev
                }
            }
            let original_duration_from_prev =
                original_trajectory_point.time_from_start - original_duration_from_start;
            original_duration_from_start = original_trajectory_point.time_from_start;

            let use_limited = limited_duration_from_prev > original_duration_from_prev;
            let selected_duration = if use_limited {
                limited_duration_from_prev
            } else {
                original_duration_from_prev
            };
            limited_duration_from_start += selected_duration;
            limited_trajectory.push(TrajectoryPoint {
                positions: original_trajectory_point.positions.clone(),
                velocities: original_trajectory_point.velocities.clone(),
                time_from_start: limited_duration_from_start,
            });
            prev_positions = original_trajectory_point.positions.clone();
            debug!(
                "Sequence{sequence_index} dominant joint_index {dominant_joint_index} duration limited : {limited_duration_from_prev:?}{} original : {original_duration_from_prev:?}{}",
                if use_limited { "(O)" } else { "" },
                if use_limited { "" } else { "(O)" }
            );
        }

        debug!("OriginalTrajectory {trajectory:?}");
        debug!("LimitedTrajectory {limited_trajectory:?}");

        self.client.send_joint_trajectory(limited_trajectory)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use assert_approx_eq::assert_approx_eq;

    use super::*;
    use crate::DummyJointTrajectoryClient;
    #[test]
    #[should_panic]
    fn mismatch_size() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        JointVelocityLimiter::new(client, vec![1.0, 2.0]);
    }
    #[test]
    fn joint_names() {
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned(), "b".to_owned()]);
        let limiter = JointVelocityLimiter::new(client, vec![1.0, 2.0]);
        let joint_names = limiter.joint_names();
        assert_eq!(joint_names.len(), 2);
        assert_eq!(joint_names[0], "a");
        assert_eq!(joint_names[1], "b");
    }
    fn test_send_joint_positions(limits: Vec<f64>, expected_duration_secs: f64) {
        let client = Arc::new(DummyJointTrajectoryClient::new(vec![
            "a".to_owned(),
            "b".to_owned(),
        ]));
        let limiter = JointVelocityLimiter::new(client.clone(), limits);
        assert!(tokio_test::block_on(
            limiter
                .send_joint_positions(vec![1.0, 2.0], std::time::Duration::from_secs_f64(4.0))
                .unwrap()
        )
        .is_ok());
        let joint_positions = limiter.current_joint_positions().unwrap();
        assert_eq!(joint_positions.len(), 2);
        assert_approx_eq!(joint_positions[0], 1.0);
        assert_approx_eq!(joint_positions[1], 2.0);
        let trajectory = client.last_trajectory.lock();
        assert_eq!(trajectory.len(), 1);
        assert_eq!(trajectory[0].positions.len(), 2);
        assert_approx_eq!(trajectory[0].positions[0], 1.0);
        assert_approx_eq!(trajectory[0].positions[1], 2.0);
        assert!(trajectory[0].velocities.is_none());
        assert_approx_eq!(
            trajectory[0].time_from_start.as_secs_f64(),
            expected_duration_secs
        );
    }

    #[test]
    fn send_joint_positions_none_limited() {
        test_send_joint_positions(vec![1.0, 2.0], 4.0);
    }

    #[test]
    fn send_joint_positions_limited() {
        // joint0 is over limit
        test_send_joint_positions(vec![0.1, 2.0], 10.0);
        // joint1 is over limit
        test_send_joint_positions(vec![1.0, 0.2], 10.0);
        // joint0/1 are over limit, joint0 is dominant
        test_send_joint_positions(vec![0.1, 0.6], 10.0);
        // joint0/1 are over limit, joint1 is dominant
        test_send_joint_positions(vec![0.3, 0.2], 10.0);
    }

    fn test_send_joint_trajectory(limits: Vec<f64>, expected_durations_secs: [f64; 2]) {
        let client = Arc::new(DummyJointTrajectoryClient::new(vec![
            "a".to_owned(),
            "b".to_owned(),
        ]));
        let limiter = JointVelocityLimiter::new(client.clone(), limits);
        assert!(tokio_test::block_on(
            limiter
                .send_joint_trajectory(vec![
                    TrajectoryPoint {
                        positions: vec![1.0, 2.0],
                        velocities: Some(vec![3.0, 4.0]),
                        time_from_start: std::time::Duration::from_secs_f64(4.0)
                    },
                    TrajectoryPoint {
                        positions: vec![3.0, 6.0],
                        velocities: Some(vec![3.0, 4.0]),
                        time_from_start: std::time::Duration::from_secs_f64(8.0)
                    }
                ])
                .unwrap()
        )
        .is_ok());
        let joint_positions = limiter.current_joint_positions().unwrap();
        assert_eq!(joint_positions.len(), 2);
        assert_approx_eq!(joint_positions[0], 3.0);
        assert_approx_eq!(joint_positions[1], 6.0);

        let trajectory = client.last_trajectory.lock();
        assert_eq!(trajectory.len(), 2);
        assert_eq!(trajectory[0].positions.len(), 2);
        assert_approx_eq!(trajectory[0].positions[0], 1.0);
        assert_approx_eq!(trajectory[0].positions[1], 2.0);
        assert!(trajectory[0].velocities.is_some());
        assert_approx_eq!(trajectory[0].velocities.as_ref().unwrap()[0], 3.0);
        assert_approx_eq!(trajectory[0].velocities.as_ref().unwrap()[1], 4.0);

        assert_eq!(trajectory[1].positions.len(), 2);
        assert_approx_eq!(trajectory[1].positions[0], 3.0);
        assert_approx_eq!(trajectory[1].positions[1], 6.0);
        assert!(trajectory[1].velocities.is_some());
        assert_approx_eq!(trajectory[1].velocities.as_ref().unwrap()[0], 3.0);
        assert_approx_eq!(trajectory[1].velocities.as_ref().unwrap()[1], 4.0);

        assert_approx_eq!(
            trajectory[0].time_from_start.as_secs_f64(),
            expected_durations_secs[0]
        );

        assert_approx_eq!(
            trajectory[1].time_from_start.as_secs_f64(),
            expected_durations_secs[1]
        );
    }

    #[test]
    fn send_joint_trajectory_none_limited() {
        test_send_joint_trajectory(vec![1.0, 2.0], [4.0, 8.0]);
    }

    #[test]
    fn send_joint_trajectory_limited() {
        // joint0 is over limit
        test_send_joint_trajectory(vec![0.1, 2.0], [10.0, 30.0]);
        // joint1 is over limit
        test_send_joint_trajectory(vec![1.0, 0.2], [10.0, 30.0]);
        // joint0/1 are over limit, joint0 is dominant
        test_send_joint_trajectory(vec![0.1, 0.6], [10.0, 30.0]);
        // joint0/1 are over limit, joint1 is dominant
        test_send_joint_trajectory(vec![0.3, 0.2], [10.0, 30.0]);
        // joint0 / point1 is over limit
        test_send_joint_trajectory(vec![0.3, 2.0], [4.0, 4.0 + 2.0 / 0.3]);
        // joint1 / point1 is over limit
        test_send_joint_trajectory(vec![1.0, 0.8], [4.0, 4.0 + 4.0 / 0.8]);
    }

    #[test]
    fn from_urdf() {
        let s = r#"
            <robot name="robot">
                <joint name="a" type="revolute">
                    <origin xyz="0.0 0.0 0.0" />
                    <parent link="b" />
                    <child link="c" />
                    <axis xyz="0 1 0" />
                    <limit lower="-2" upper="1.0" effort="0" velocity="1.0"/>
                </joint>
            </robot>
        "#;
        let urdf_robot = urdf_rs::read_from_string(s).unwrap();
        let client = DummyJointTrajectoryClient::new(vec!["a".to_owned()]);
        let limiter = JointVelocityLimiter::from_urdf(client, &urdf_robot.joints).unwrap();
        assert_approx_eq!(limiter.velocity_limits[0], 1.0);

        // joint name mismatch
        let urdf_robot = urdf_rs::read_from_string(s).unwrap();
        let client = DummyJointTrajectoryClient::new(vec!["unknown".to_owned()]);
        let e = JointVelocityLimiter::from_urdf(client, &urdf_robot.joints)
            .err()
            .unwrap();
        assert!(matches!(e, Error::NoJoint(..)));
    }
}
