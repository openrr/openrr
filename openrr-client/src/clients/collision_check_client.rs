use std::{path::Path, sync::Arc};

use arci::{Error, JointTrajectoryClient, TrajectoryPoint, WaitFuture};
use openrr_planner::{
    collision::create_self_collision_checker, SelfCollisionChecker, SelfCollisionCheckerConfig,
};

pub struct CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    pub collision_checker: Arc<SelfCollisionChecker>,
}

impl<T> CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(client: T, collision_checker: Arc<SelfCollisionChecker>) -> Self {
        Self {
            client,
            collision_checker,
        }
    }
}

impl<T> JointTrajectoryClient for CollisionCheckClient<T>
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
        self.collision_checker
            .check_joint_positions(&self.current_joint_positions()?, &positions, duration)
            .map_err(|e| Error::Other(e.into()))?;
        self.client.send_joint_positions(positions, duration)
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        let position_trajectory = trajectory
            .iter()
            .map(|point| {
                openrr_planner::TrajectoryPoint::new(point.positions.clone(), vec![], vec![])
            })
            .collect::<Vec<_>>();
        self.collision_checker
            .check_joint_trajectory(&position_trajectory)
            .map_err(|e| Error::Other(e.into()))?;
        self.client.send_joint_trajectory(trajectory)
    }
}

pub fn create_collision_check_client<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: &[String],
    config: &SelfCollisionCheckerConfig,
    client: Arc<dyn JointTrajectoryClient>,
    full_chain: Arc<k::Chain<f64>>,
) -> CollisionCheckClient<Arc<dyn JointTrajectoryClient>> {
    let joint_names = client.joint_names();
    CollisionCheckClient::new(
        client,
        Arc::new(create_self_collision_checker(
            urdf_path,
            self_collision_check_pairs,
            joint_names,
            &config,
            full_chain,
        )),
    )
}
