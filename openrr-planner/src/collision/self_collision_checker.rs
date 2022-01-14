use std::{path::Path, sync::Arc, time::Duration};

use k::nalgebra as na;
use na::RealField;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::{
    collision::parse_colon_separated_pairs, errors::*, interpolate, CollisionDetector,
    TrajectoryPoint,
};

pub struct SelfCollisionChecker<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    pub collision_check_robot: Arc<k::Chain<N>>,
    pub collision_detector: CollisionDetector<N>,
    pub collision_pairs: Vec<(String, String)>,

    pub time_interpolate_rate: N,
}

impl<N> SelfCollisionChecker<N>
where
    N: RealField + k::SubsetOf<f64> + num_traits::Float,
{
    #[track_caller]
    pub fn new(
        collision_check_robot: Arc<k::Chain<N>>,
        collision_detector: CollisionDetector<N>,
        collision_pairs: Vec<(String, String)>,
        time_interpolate_rate: N,
    ) -> Self {
        assert!(
            time_interpolate_rate > na::convert(0.0) && time_interpolate_rate <= na::convert(1.0),
            "time_interpolate_rate must be 0.0~1.0 but {}",
            time_interpolate_rate
        );

        Self {
            collision_check_robot,
            collision_detector,
            collision_pairs,
            time_interpolate_rate,
        }
    }

    pub fn check_joint_positions(
        &self,
        current: &[N],
        positions: &[N],
        duration: std::time::Duration,
    ) -> Result<()> {
        self.check_partial_joint_positions(
            &self.collision_check_robot,
            current,
            positions,
            duration,
        )
    }

    pub fn check_partial_joint_positions(
        &self,
        using_joints: &k::Chain<N>,
        current: &[N],
        positions: &[N],
        duration: std::time::Duration,
    ) -> Result<()> {
        let duration_f64 = num_traits::NumCast::from::<f64>(duration.as_secs_f64()).unwrap();
        match interpolate(
            &[current.to_vec(), positions.to_vec()],
            duration_f64,
            self.time_interpolate_rate.mul(duration_f64),
        ) {
            Some(interpolated) => {
                debug!("interpolated len={}", interpolated.len());
                self.check_partial_joint_trajectory(using_joints, &interpolated)
            }
            None => Err(Error::InterpolationError(
                "failed to interpolate".to_owned(),
            )),
        }
    }

    pub fn check_joint_trajectory(&self, trajectory: &[TrajectoryPoint<N>]) -> Result<()> {
        self.check_partial_joint_trajectory(&self.collision_check_robot, trajectory)
    }

    pub fn check_partial_joint_trajectory(
        &self,
        using_joints: &k::Chain<N>,
        trajectory: &[TrajectoryPoint<N>],
    ) -> Result<()> {
        for v in trajectory {
            using_joints.set_joint_positions_clamped(&v.position);
            self.collision_check_robot.update_transforms();
            let mut self_checker = self
                .collision_detector
                .detect_self(&self.collision_check_robot, &self.collision_pairs);
            if let Some(names) = self_checker.next() {
                return Err(Error::Collision {
                    point: UnfeasibleTrajectory::StartPoint,
                    collision_link_names: vec![names.0, names.1],
                });
            }
            let mut vec_used: Vec<_> = self_checker.used_duration().iter().collect();
            vec_used.sort_by(|a, b| b.1.cmp(a.1));
            let sum_duration: Duration = self_checker.used_duration().iter().map(|(_k, v)| v).sum();
            debug!("total: {sum_duration:?}");
            debug!("detailed: {vec_used:?}");
        }
        Ok(())
    }
}

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SelfCollisionCheckerConfig {
    #[serde(default = "default_prediction")]
    pub prediction: f64,
    #[serde(default = "default_time_interpolate_rate")]
    pub time_interpolate_rate: f64,
}

fn default_prediction() -> f64 {
    0.001
}

fn default_time_interpolate_rate() -> f64 {
    0.5
}

impl Default for SelfCollisionCheckerConfig {
    fn default() -> Self {
        Self {
            prediction: default_prediction(),
            time_interpolate_rate: default_time_interpolate_rate(),
        }
    }
}

pub fn create_self_collision_checker<P: AsRef<Path>>(
    urdf_path: P,
    self_collision_check_pairs: &[String],
    config: &SelfCollisionCheckerConfig,
    full_chain: Arc<k::Chain<f64>>,
) -> SelfCollisionChecker<f64> {
    SelfCollisionChecker::new(
        full_chain,
        CollisionDetector::from_urdf_robot(
            &urdf_rs::utils::read_urdf_or_xacro(urdf_path).unwrap(),
            config.prediction,
        ),
        parse_colon_separated_pairs(self_collision_check_pairs).unwrap(),
        config.time_interpolate_rate,
    )
}

#[test]
fn test_create_self_collision_checker() {
    let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
    let robot = Arc::new(k::Chain::<f64>::from(&urdf_robot));
    let self_collision_checker = create_self_collision_checker(
        "sample.urdf",
        &["root:l_shoulder_roll".into()],
        &SelfCollisionCheckerConfig::default(),
        robot.clone(),
    );

    assert!(self_collision_checker
        .check_joint_positions(&[0.0; 8], &[0.0; 8], std::time::Duration::new(1, 0),)
        .is_ok());
    assert!(self_collision_checker
        .check_joint_positions(
            &[0.0; 8],
            &[1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            std::time::Duration::new(1, 0),
        )
        .is_err());

    let nodes = robot.iter().take(2).map(|node| (*node).clone()).collect();
    let using_joints = k::Chain::<f64>::from_nodes(nodes);

    assert!(self_collision_checker
        .check_partial_joint_positions(
            &using_joints,
            &[0.0],
            &[0.0],
            std::time::Duration::new(1, 0),
        )
        .is_ok());
    assert!(self_collision_checker
        .check_partial_joint_positions(
            &using_joints,
            &[0.0],
            &[1.57],
            std::time::Duration::new(1, 0),
        )
        .is_err());
}
