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
            config,
            full_chain,
        )),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_create_collision_check_client() {
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
            .send_joint_positions(vec![0.0; 9], std::time::Duration::new(0, 0))
            .unwrap()
            .await
            .unwrap();

        let collision_check_client = create_collision_check_client(
            urdf_path,
            &["root:l_shoulder_roll".into()],
            &SelfCollisionCheckerConfig::default(),
            Arc::new(client),
            robot,
        );

        assert_eq!(
            *collision_check_client.current_joint_positions().unwrap(),
            vec![0.0; 9]
        );

        assert!(collision_check_client
            .send_joint_positions(vec![0.0; 9], std::time::Duration::new(1, 0),)
            .is_ok());
        assert!(collision_check_client
            .send_joint_positions(
                vec![1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                std::time::Duration::new(1, 0),
            )
            .is_err());
    }
}
