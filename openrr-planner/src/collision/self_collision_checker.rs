use std::{path::Path, sync::Arc, time::Duration};

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::{
    collision::parse_colon_separated_pairs, errors::*, interpolate, CollisionChecker,
    TrajectoryPoint,
};

pub struct SelfCollisionChecker {
    pub using_joints: k::Chain<f64>,
    pub collision_check_robot: Arc<k::Chain<f64>>,
    pub collision_checker: CollisionChecker<f64>,
    pub collision_pairs: Vec<(String, String)>,
    pub time_interpolate_rate: f64,
}

impl SelfCollisionChecker {
    #[track_caller]
    pub fn new(
        joint_names: Vec<String>,
        collision_check_robot: Arc<k::Chain<f64>>,
        collision_checker: CollisionChecker<f64>,
        collision_pairs: Vec<(String, String)>,
        time_interpolate_rate: f64,
    ) -> Self {
        assert!(
            time_interpolate_rate > 0.0 && time_interpolate_rate <= 1.0,
            "time_interpolate_rate must be 0.0~1.0 but {}",
            time_interpolate_rate
        );
        let using_joints = k::Chain::<f64>::from_nodes(
            joint_names
                .into_iter()
                .map(|joint_name| collision_check_robot.find(&joint_name).unwrap().clone())
                .collect(),
        );
        Self {
            using_joints,
            collision_check_robot,
            collision_checker,
            collision_pairs,
            time_interpolate_rate,
        }
    }

    pub fn check_joint_positions(
        &self,
        current: &[f64],
        positions: &[f64],
        duration: std::time::Duration,
    ) -> Result<()> {
        match interpolate(
            &[current.to_vec(), positions.to_vec()],
            duration.as_secs_f64(),
            duration.as_secs_f64() * self.time_interpolate_rate,
        ) {
            Some(interpolated) => {
                debug!("interpolated len={}", interpolated.len());
                for v in interpolated {
                    self.using_joints.set_joint_positions_clamped(&v.position);
                    self.collision_check_robot.update_transforms();
                    let mut self_checker = self
                        .collision_checker
                        .check_self(&self.collision_check_robot, &self.collision_pairs);
                    if let Some(names) = self_checker.next() {
                        return Err(Error::Collision {
                            point: UnfeasibleTrajectory::StartPoint,
                            collision_link_names: vec![names.0, names.1],
                        });
                    }
                    let mut vec_used: Vec<_> = self_checker.used_duration().iter().collect();
                    vec_used.sort_by(|a, b| b.1.cmp(a.1));
                    let sum_duration: Duration =
                        self_checker.used_duration().iter().map(|(_k, v)| v).sum();
                    debug!("total: {:?}", sum_duration);
                    debug!("detailed: {:?}", vec_used);
                }
                Ok(())
            }
            None => Err(Error::InterpolationError(
                "failed to interpolate".to_owned(),
            )),
        }
    }

    pub fn check_joint_trajectory(&self, trajectory: &[TrajectoryPoint<f64>]) -> Result<()> {
        for v in trajectory {
            self.using_joints.set_joint_positions(&v.position)?;
            if let Some(names) = self
                .collision_checker
                .check_self(&self.collision_check_robot, &self.collision_pairs)
                .next()
            {
                return Err(Error::Collision {
                    point: UnfeasibleTrajectory::StartPoint,
                    collision_link_names: vec![names.0, names.1],
                });
            }
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
    joint_names: Vec<String>,
    config: &SelfCollisionCheckerConfig,
    full_chain: Arc<k::Chain<f64>>,
) -> SelfCollisionChecker {
    SelfCollisionChecker::new(
        joint_names,
        full_chain,
        CollisionChecker::from_urdf_robot(
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
        robot
            .iter_joints()
            .map(|joint| joint.name.clone())
            .collect(),
        &SelfCollisionCheckerConfig::default(),
        robot,
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
}
