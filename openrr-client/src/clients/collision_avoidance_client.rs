use std::{path::Path, sync::Arc};

use arci::{Error, JointTrajectoryClient, TrajectoryPoint, WaitFuture};
use openrr_planner::JointPathPlannerBuilder;

// TODO: speed limit
fn trajectory_from_positions(
    positions: &[Vec<f64>],
    total_duration: std::time::Duration,
) -> Vec<TrajectoryPoint> {
    let num_points = positions.len();
    let mut traj = vec![];
    for (i, pos) in positions.iter().enumerate() {
        let time_rate: f64 = ((i + 1) as f64) / (num_points as f64);
        traj.push(TrajectoryPoint::new(
            pos.clone(),
            total_duration.mul_f64(time_rate),
        ));
    }
    traj
}

pub struct CollisionAvoidanceClient<T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    /// using_joints must be a part of planner.collision_check_robot.
    pub using_joints: k::Chain<f64>,
    pub planner: openrr_planner::JointPathPlanner<f64>,
}

impl<T> CollisionAvoidanceClient<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(
        client: T,
        using_joints: k::Chain<f64>,
        planner: openrr_planner::JointPathPlanner<f64>,
    ) -> Self {
        Self {
            client,
            using_joints,
            planner,
        }
    }
}

impl<T> JointTrajectoryClient for CollisionAvoidanceClient<T>
where
    T: JointTrajectoryClient,
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
        self.using_joints
            .set_joint_positions_clamped(&self.current_joint_positions()?);
        let current = self.using_joints.joint_positions();
        let traj = self
            .planner
            .plan_avoid_self_collision(&self.using_joints, &current, &positions)
            .map_err(|e| Error::Other(e.into()))?;
        self.client
            .send_joint_trajectory(trajectory_from_positions(&traj, duration))
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        if trajectory.is_empty() {
            return Ok(WaitFuture::ready());
        }
        self.using_joints
            .set_joint_positions_clamped(&self.current_joint_positions()?);
        let current = self.using_joints.joint_positions();
        let positions = self
            .planner
            .plan_avoid_self_collision(&self.using_joints, &current, &trajectory[0].positions)
            .map_err(|e| Error::Other(e.into()))?;
        let mut trajs = trajectory_from_positions(&positions, trajectory[0].time_from_start);

        for i in 1..trajectory.len() {
            let positions = self
                .planner
                .plan_avoid_self_collision(
                    &self.using_joints,
                    &trajectory[i - 1].positions,
                    &trajectory[i].positions,
                )
                .map_err(|e| Error::Other(e.into()))?;
            trajs.append(&mut trajectory_from_positions(
                &positions,
                trajectory[i].time_from_start,
            ));
        }
        self.client.send_joint_trajectory(trajs)
    }
}

pub fn create_collision_avoidance_client<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: Vec<(String, String)>,
    collision_check_prediction: f64,
    client: Arc<dyn JointTrajectoryClient>,
) -> CollisionAvoidanceClient<Arc<dyn JointTrajectoryClient>> {
    let urdf_robot = urdf_rs::read_file(urdf_path.as_ref()).unwrap();
    let planner_builder = JointPathPlannerBuilder::from_urdf_robot(urdf_robot.clone())
        .self_collision_pairs(self_collision_check_pairs)
        .collision_check_margin(collision_check_prediction);
    CollisionAvoidanceClient::new(
        client,
        k::Chain::<f64>::from(&urdf_robot),
        planner_builder.finalize(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_collision_avoidance_client() {
        let urdf_path = Path::new("sample.urdf");
        let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();
        let robot = Arc::new(k::Chain::<f64>::from(&urdf_robot));
        let client = arci::DummyJointTrajectoryClient::new(
            robot
                .iter_joints()
                .map(|joint| joint.name.clone())
                .collect(),
        );
        client
            .send_joint_positions(vec![0.0; 8], std::time::Duration::new(0, 0))
            .unwrap()
            .await
            .unwrap();

        let collision_avoidance_client = create_collision_avoidance_client(
            urdf_path,
            vec![("root".to_owned(), "l_shoulder_roll".to_owned())],
            0.0001,
            Arc::new(client),
        );

        assert_eq!(
            *collision_avoidance_client
                .current_joint_positions()
                .unwrap(),
            vec![0.0; 8]
        );

        // No collision case
        assert!(collision_avoidance_client
            .send_joint_positions(vec![0.0; 8], std::time::Duration::new(1, 0),)
            .is_ok());
        // Collision occurs in the reference position
        assert!(collision_avoidance_client
            .send_joint_positions(
                vec![1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                std::time::Duration::new(1, 0),
            )
            .is_err());
    }
}
