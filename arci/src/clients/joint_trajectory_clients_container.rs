use crate::error::Error;
use crate::traits::{JointTrajectoryClient, TrajectoryPoint, Wait};
use crate::waits::WaitContainer;

pub struct JointTrajectoryClientsContainer<T: JointTrajectoryClient> {
    joint_names: Vec<String>,
    clients: Vec<T>,
}

impl<T> JointTrajectoryClientsContainer<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(clients: Vec<T>) -> Self {
        let mut joint_names = vec![];
        for c in &clients {
            joint_names.append(&mut c.joint_names().to_vec());
        }
        Self {
            joint_names,
            clients,
        }
    }
}

impl<T> JointTrajectoryClient for JointTrajectoryClientsContainer<T>
where
    T: JointTrajectoryClient + Sync,
{
    fn joint_names(&self) -> &[String] {
        &self.joint_names
    }
    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        let mut ret = vec![];
        for c in &self.clients {
            let mut positions = c.current_joint_positions()?;
            ret.append(&mut positions);
        }
        Ok(ret)
    }
    fn send_joint_positions(
        &self,
        positions: &[f64],
        duration: std::time::Duration,
    ) -> Result<Wait, Error> {
        let mut offset = 0;
        let mut waits = vec![];
        for c in &self.clients {
            let mut current_positions = c.current_joint_positions()?;
            for i in 0..current_positions.len() {
                if positions.len() > (offset + i) {
                    current_positions[i] = positions[offset + i];
                }
            }
            offset += current_positions.len();
            waits.push(c.send_joint_positions(&current_positions, duration)?);
        }
        Ok(WaitContainer::new_boxed(waits))
    }
    fn send_joint_trajectory(&self, full_trajectory: &[TrajectoryPoint]) -> Result<Wait, Error> {
        let mut offset = 0;
        let full_dof = self.joint_names().len();
        let mut waits = vec![];
        for client in &self.clients {
            let mut current_positions = client.current_joint_positions()?;
            let partial_dof = current_positions.len();
            let mut partial_trajectory: Vec<TrajectoryPoint> = vec![];
            for full_point in full_trajectory {
                for (i, current) in current_positions.iter_mut().enumerate().take(partial_dof) {
                    if full_dof > (offset + i) {
                        *current = full_point.positions[offset + i];
                    }
                }
                let partial_velocities = if let Some(full_velocities) = &full_point.velocities {
                    let mut partial_velocities = vec![0.0; partial_dof];
                    for (i, partial_velocity) in
                        partial_velocities.iter_mut().enumerate().take(partial_dof)
                    {
                        if full_dof > (offset + i) {
                            *partial_velocity = full_velocities[offset + i];
                        }
                    }
                    Some(partial_velocities)
                } else {
                    None
                };
                partial_trajectory.push(TrajectoryPoint {
                    positions: current_positions.clone(),
                    velocities: partial_velocities,
                    time_from_start: full_point.time_from_start,
                });
            }
            waits.push(client.send_joint_trajectory(&partial_trajectory)?);
            offset += partial_dof;
        }
        Ok(WaitContainer::new_boxed(waits))
    }
}
