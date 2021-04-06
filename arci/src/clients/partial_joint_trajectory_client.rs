use crate::error::Error;
use crate::traits::{JointTrajectoryClient, TrajectoryPoint};
use crate::waits::WaitFuture;

pub struct PartialJointTrajectoryClient<C>
where
    C: JointTrajectoryClient,
{
    joint_names: Vec<String>,
    shared_client: C,
    full_joint_names: Vec<String>,
}

pub fn copy_joint_positions(
    from_joint_names: &[String],
    from_positions: &[f64],
    to_joint_names: &[String],
    to_positions: &mut [f64],
) {
    for (to_index, to_joint_name) in to_joint_names.iter().enumerate() {
        if let Some(from_index) = from_joint_names.iter().position(|x| x == to_joint_name) {
            to_positions[to_index] = from_positions[from_index];
        }
    }
}

impl<C> PartialJointTrajectoryClient<C>
where
    C: JointTrajectoryClient,
{
    pub fn new(joint_names: Vec<String>, shared_client: C) -> Self {
        let full_joint_names = shared_client.joint_names().to_vec();
        Self {
            joint_names,
            shared_client,
            full_joint_names,
        }
    }
}

impl<C> JointTrajectoryClient for PartialJointTrajectoryClient<C>
where
    C: JointTrajectoryClient,
{
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        let mut result = vec![0.0; self.joint_names.len()];
        copy_joint_positions(
            &self.full_joint_names,
            &self.shared_client.current_joint_positions()?,
            self.joint_names(),
            &mut result,
        );
        Ok(result)
    }
    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<WaitFuture, Error> {
        let mut full_positions = self.shared_client.current_joint_positions()?;
        copy_joint_positions(
            self.joint_names(),
            &positions,
            &self.full_joint_names,
            &mut full_positions,
        );
        self.shared_client
            .send_joint_positions(full_positions, duration)
    }
    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        let full_positions_base = self.shared_client.current_joint_positions()?;
        let mut full_trajectory = vec![];
        let full_dof = full_positions_base.len();
        for point in trajectory {
            let mut full_positions = full_positions_base.clone();
            copy_joint_positions(
                self.joint_names(),
                &point.positions,
                &self.full_joint_names,
                &mut full_positions,
            );
            let mut full_point = TrajectoryPoint::new(full_positions, point.time_from_start);
            if let Some(partial_velocities) = &point.velocities {
                let mut full_velocities = vec![0.0; full_dof];
                copy_joint_positions(
                    self.joint_names(),
                    &partial_velocities,
                    &self.full_joint_names,
                    &mut full_velocities,
                );
                full_point.velocities = Some(full_velocities);
            }
            full_trajectory.push(full_point);
        }
        self.shared_client.send_joint_trajectory(full_trajectory)
    }
}
