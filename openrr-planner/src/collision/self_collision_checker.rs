use std::{path::Path, time::Duration};

use k::nalgebra as na;
use na::RealField;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::{
    collision::{
        create_robot_collision_detector, parse_colon_separated_pairs, RobotCollisionDetector,
        RobotCollisionDetectorConfig,
    },
    errors::*,
    interpolate, TrajectoryPoint,
};

pub struct SelfCollisionChecker<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    /// Robot collision detector
    pub robot_collision_detector: RobotCollisionDetector<N>,
    /// Rate of time interpolation
    pub time_interpolate_rate: N,
}

impl<N> SelfCollisionChecker<N>
where
    N: RealField + k::SubsetOf<f64> + num_traits::Float,
{
    #[track_caller]
    pub fn new(
        robot_collision_detector: RobotCollisionDetector<N>,
        time_interpolate_rate: N,
    ) -> Self {
        assert!(
            time_interpolate_rate > na::convert(0.0) && time_interpolate_rate <= na::convert(1.0),
            "time_interpolate_rate must be 0.0~1.0 but {}",
            time_interpolate_rate
        );

        Self {
            robot_collision_detector,
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
            &self.robot_collision_detector.robot,
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
        self.check_partial_joint_trajectory(&self.robot_collision_detector.robot, trajectory)
    }

    pub fn check_partial_joint_trajectory(
        &self,
        using_joints: &k::Chain<N>,
        trajectory: &[TrajectoryPoint<N>],
    ) -> Result<()> {
        for v in trajectory {
            using_joints.set_joint_positions_clamped(&v.position);
            self.robot_collision_detector.robot.update_transforms();
            let mut self_checker = self.robot_collision_detector.detect_self();
            if let Some(names) = self_checker.next() {
                return Err(Error::Collision {
                    point: UnfeasibleTrajectory::StartPoint,
                    collision_link_names: vec![names.0, names.1],
                });
            }
            let mut vec_used: Vec<_> = self_checker.used_duration().iter().collect();
            vec_used.sort_by(|a, b| b.1.cmp(a.1));
            let sum_duration: Duration = self_checker.used_duration().iter().map(|(_k, v)| v).sum();
            debug!("total: {:?}", sum_duration);
            debug!("detailed: {:?}", vec_used);
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
) -> SelfCollisionChecker<f64> {
    let robot_collision_detector = create_robot_collision_detector(
        urdf_path,
        RobotCollisionDetectorConfig::new(config.prediction),
        parse_colon_separated_pairs(self_collision_check_pairs).unwrap(),
    );
    SelfCollisionChecker::new(robot_collision_detector, config.time_interpolate_rate)
}

#[test]
fn test_create_self_collision_checker() {
    let urdf_path = Path::new("sample.urdf");
    let self_collision_checker = create_self_collision_checker(
        urdf_path,
        &["root:l_shoulder_roll".into()],
        &SelfCollisionCheckerConfig::default(),
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

    let robot = &self_collision_checker.robot_collision_detector.robot;
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
