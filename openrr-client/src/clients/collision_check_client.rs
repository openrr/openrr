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
    /// using_joints must be a part of collision_checker.collision_check_robot.
    pub using_joints: k::Chain<f64>,
    pub collision_checker: Arc<SelfCollisionChecker<f64>>,
}

impl<T> CollisionCheckClient<T>
where
    T: JointTrajectoryClient,
{
    pub fn new(
        client: T,
        using_joints: k::Chain<f64>,
        collision_checker: Arc<SelfCollisionChecker<f64>>,
    ) -> Self {
        Self {
            client,
            using_joints,
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
            .check_partial_joint_positions(
                &self.using_joints,
                &self.current_joint_positions()?,
                &positions,
                duration,
            )
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
            .check_partial_joint_trajectory(&self.using_joints, &position_trajectory)
            .map_err(|e| Error::Other(e.into()))?;
        self.client.send_joint_trajectory(trajectory)
    }
}

pub fn create_collision_check_client<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: &[String],
    config: &SelfCollisionCheckerConfig,
    client: Arc<dyn JointTrajectoryClient>,
    reference_robot: Arc<k::Chain<f64>>,
) -> CollisionCheckClient<Arc<dyn JointTrajectoryClient>> {
    let collision_checker = Arc::new(create_self_collision_checker(
        urdf_path,
        self_collision_check_pairs,
        config,
        reference_robot,
    ));

    // Reconstruct a robot chain to define using_joints
    let joint_names = client.joint_names();
    let nodes = joint_names
        .iter()
        .map(|joint_name| {
            (*collision_checker
                .robot_collision_detector
                .robot
                .find(joint_name)
                .unwrap())
            .clone()
        })
        .collect();
    let using_joints = k::Chain::<f64>::from_nodes(nodes);

    CollisionCheckClient::new(client, using_joints, collision_checker)
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
            .send_joint_positions(vec![0.0; 8], std::time::Duration::new(0, 0))
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
            vec![0.0; 8]
        );

        assert!(collision_check_client
            .send_joint_positions(vec![0.0; 8], std::time::Duration::new(1, 0),)
            .is_ok());
        assert!(collision_check_client
            .send_joint_positions(
                vec![1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                std::time::Duration::new(1, 0),
            )
            .is_err());
    }

    #[tokio::test]
    async fn test_create_collision_check_client_for_partial_joints() {
        let urdf_path = Path::new("sample.urdf");
        let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();
        let robot = Arc::new(k::Chain::<f64>::from(&urdf_robot));
        let client = arci::DummyJointTrajectoryClient::new(
            robot
                .iter_joints()
                .take(2)
                .map(|joint| joint.name.clone())
                .collect(),
        );
        client
            .send_joint_positions(vec![0.0; 2], std::time::Duration::new(0, 0))
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
            vec![0.0; 2]
        );

        assert!(collision_check_client
            .send_joint_positions(vec![0.0; 2], std::time::Duration::new(1, 0),)
            .is_ok());
        assert!(collision_check_client
            .send_joint_positions(vec![1.57, 0.0], std::time::Duration::new(1, 0),)
            .is_err());

        let point_ok = TrajectoryPoint::new([0.0; 2].to_vec(), std::time::Duration::new(1, 0));
        let trajectory_ok = vec![point_ok];
        assert!(collision_check_client
            .send_joint_trajectory(trajectory_ok)
            .is_ok());

        let point_err = TrajectoryPoint::new([1.57, 0.0].to_vec(), std::time::Duration::new(2, 0));
        let trajectory_err = vec![point_err];
        assert!(collision_check_client
            .send_joint_trajectory(trajectory_err)
            .is_err());
    }
}
