use std::{path::Path, sync::Arc, time::Duration};

use k::nalgebra as na;
use na::RealField;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::{
    collision::{parse_colon_separated_pairs, RobotCollisionDetector},
    errors::*,
    funcs::create_chain_from_joint_names,
    interpolate, CollisionDetector, TrajectoryPoint,
};

pub struct SelfCollisionChecker<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    /// Robot for reference (read only and assumed to hold the latest full states)
    reference_robot: Arc<k::Chain<N>>,
    /// Robot collision detector
    robot_collision_detector: RobotCollisionDetector<N>,
    /// Rate of time interpolation
    time_interpolate_rate: N,
}

impl<N> SelfCollisionChecker<N>
where
    N: RealField + k::SubsetOf<f64> + num_traits::Float,
{
    #[track_caller]
    pub fn new(
        reference_robot: Arc<k::Chain<N>>,
        robot_collision_detector: RobotCollisionDetector<N>,
        time_interpolate_rate: N,
    ) -> Self {
        assert!(
            time_interpolate_rate > na::convert(0.0) && time_interpolate_rate <= na::convert(1.0),
            "time_interpolate_rate must be 0.0~1.0 but {}",
            time_interpolate_rate
        );

        Self {
            reference_robot,
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
        self.check_partial_joint_positions_inner(None, current, positions, duration)
    }

    pub fn check_partial_joint_positions(
        &self,
        using_joint_names: &[String],
        current: &[N],
        positions: &[N],
        duration: std::time::Duration,
    ) -> Result<()> {
        self.check_partial_joint_positions_inner(
            Some(using_joint_names),
            current,
            positions,
            duration,
        )
    }

    fn check_partial_joint_positions_inner(
        &self,
        using_joint_names: Option<&[String]>,
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
                self.check_partial_joint_trajectory_inner(using_joint_names, &interpolated)
            }
            None => Err(Error::InterpolationError(
                "failed to interpolate".to_owned(),
            )),
        }
    }

    pub fn check_joint_trajectory(&self, trajectory: &[TrajectoryPoint<N>]) -> Result<()> {
        self.check_partial_joint_trajectory_inner(None, trajectory)
    }

    pub fn check_partial_joint_trajectory(
        &self,
        using_joint_names: &[String],
        trajectory: &[TrajectoryPoint<N>],
    ) -> Result<()> {
        self.check_partial_joint_trajectory_inner(Some(using_joint_names), trajectory)
    }

    fn check_partial_joint_trajectory_inner(
        &self,
        using_joint_names: Option<&[String]>,
        trajectory: &[TrajectoryPoint<N>],
    ) -> Result<()> {
        // Synchronize with the reference robot states for joints not included using_joints
        self.collision_check_robot()
            .set_joint_positions_clamped(self.reference_robot.joint_positions().as_slice());

        let using_joints = match using_joint_names {
            Some(joint_names) => {
                create_chain_from_joint_names(self.collision_check_robot(), joint_names).unwrap()
            }
            None => {
                let nodes = self
                    .collision_check_robot()
                    .iter()
                    .map(|node| (*node).clone())
                    .collect::<Vec<k::Node<N>>>();
                k::Chain::from_nodes(nodes)
            }
        };

        // Check the partial trajectory
        let last_index = trajectory.len() - 1;
        for (i, v) in trajectory.iter().enumerate() {
            using_joints.set_joint_positions(&v.position)?;
            self.collision_check_robot().update_transforms();

            let mut self_checker = self.robot_collision_detector.detect_self();
            if let Some(names) = self_checker.next() {
                return Err(Error::Collision {
                    point: match i {
                        0 => UnfeasibleTrajectoryPoint::Start,
                        index if index == last_index => UnfeasibleTrajectoryPoint::Goal,
                        _ => UnfeasibleTrajectoryPoint::WayPoint,
                    },
                    collision_link_names: vec![names.0, names.1],
                });
            }

            // Summarize the calculation time
            let mut vec_used: Vec<_> = self_checker.used_duration().iter().collect();
            vec_used.sort_by(|a, b| b.1.cmp(a.1));
            let sum_duration: Duration = self_checker.used_duration().iter().map(|(_k, v)| v).sum();
            debug!("total: {sum_duration:?}");
            debug!("detailed: {vec_used:?}");
        }
        Ok(())
    }

    /// Get the robot model used for collision checking
    pub fn collision_check_robot(&self) -> &k::Chain<N> {
        &self.robot_collision_detector.robot
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
    robot: Arc<k::Chain<f64>>,
) -> SelfCollisionChecker<f64> {
    let collision_detector = CollisionDetector::from_urdf_robot(
        &urdf_rs::read_file(urdf_path).unwrap(),
        config.prediction,
    );
    let robot_collision_detector = RobotCollisionDetector::new(
        (*robot).clone(),
        collision_detector,
        parse_colon_separated_pairs(self_collision_check_pairs).unwrap(),
    );

    SelfCollisionChecker::new(
        robot,
        robot_collision_detector,
        config.time_interpolate_rate,
    )
}

#[test]
fn test_create_self_collision_checker() {
    let urdf_path = Path::new("sample.urdf");
    let robot = Arc::new(k::Chain::from_urdf_file(urdf_path).unwrap());
    let self_collision_checker = create_self_collision_checker(
        urdf_path,
        &["root:l_shoulder_roll".into()],
        &SelfCollisionCheckerConfig::default(),
        robot,
    );

    assert!(self_collision_checker
        .check_joint_positions(&[0.0; 16], &[0.0; 16], std::time::Duration::new(1, 0),)
        .is_ok());
    assert!(self_collision_checker
        .check_joint_positions(
            &[0.0; 16],
            &[1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            std::time::Duration::new(1, 0),
        )
        .is_err());

    let l_shoulder_yaw_node = self_collision_checker
        .collision_check_robot()
        .find("l_shoulder_yaw")
        .unwrap();
    let using_joints = k::SerialChain::from_end(l_shoulder_yaw_node);
    let using_joint_names = using_joints
        .iter_joints()
        .map(|j| j.name.to_owned())
        .collect::<Vec<String>>();

    assert!(self_collision_checker
        .check_partial_joint_positions(
            using_joint_names.as_slice(),
            &[0.0],
            &[0.0],
            std::time::Duration::new(1, 0),
        )
        .is_ok());
    assert!(self_collision_checker
        .check_partial_joint_positions(
            using_joint_names.as_slice(),
            &[0.0],
            &[1.57],
            std::time::Duration::new(1, 0),
        )
        .is_err());
}
