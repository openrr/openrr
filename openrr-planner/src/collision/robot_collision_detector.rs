use std::path::Path;

use k::nalgebra as na;
use na::RealField;
use ncollide3d::shape::{Compound, Shape};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::collision::{CollisionDetector, EnvCollisionNames, SelfCollisionPairs};

pub struct RobotCollisionDetector<N>
where
    N: RealField + k::SubsetOf<f64>,
{
    /// Robot model instance used for collision detection
    pub robot: k::Chain<N>,
    /// Collision detector
    pub collision_detector: CollisionDetector<N>,
    /// Optional self collision check node names
    pub self_collision_pairs: Vec<(String, String)>,
}

/// CollisionDetector holding robot information
impl<N> RobotCollisionDetector<N>
where
    N: RealField + k::SubsetOf<f64>,
{
    pub fn new(
        robot: k::Chain<N>,
        collision_detector: CollisionDetector<N>,
        self_collision_pairs: Vec<(String, String)>,
    ) -> Self {
        RobotCollisionDetector {
            robot,
            collision_detector,
            self_collision_pairs,
        }
    }

    /// Detects collisions of the robot with an environmental object and returns names of the colliding links(joints)
    ///
    /// target_shape: shape of the environmental object
    /// target_pose: pose of the environmental object
    pub fn detect_env<'a>(
        &'a self,
        target_shape: &'a dyn Shape<N>,
        target_pose: &'a na::Isometry3<N>,
    ) -> EnvCollisionNames<'a, 'a, N> {
        self.robot.update_transforms();

        EnvCollisionNames::new(
            &self.collision_detector,
            &self.robot,
            target_shape,
            target_pose,
        )
    }

    /// Detects self collisions and returns name pairs of the self-colliding links(joints)
    pub fn detect_self(&self) -> SelfCollisionPairs<'_, N> {
        self.robot.update_transforms();

        SelfCollisionPairs::new(
            &self.collision_detector,
            &self.robot,
            &self.self_collision_pairs,
        )
    }

    /// Gets names of links colliding with environmental objects
    /// objects: environmental objects
    pub fn env_collision_link_names(&self, objects: &Compound<N>) -> Vec<String> {
        let mut ret = Vec::new();
        for shape in objects.shapes() {
            let mut colliding_names = self.detect_env(&*shape.1, &shape.0).collect();
            ret.append(&mut colliding_names);
        }
        ret
    }

    /// Gets names of self-colliding links
    pub fn self_collision_link_pairs(&self) -> Vec<(String, String)> {
        self.detect_self().collect()
    }

    /// Returns whether any collision of the robot with environmental objects is detected or not
    /// objects: environmental objects
    pub fn is_env_collision_detected(&self, objects: &Compound<N>) -> bool {
        for shape in objects.shapes() {
            if self.detect_env(&*shape.1, &shape.0).next().is_some() {
                return true;
            }
        }
        false
    }

    /// Returns whether any self collision of the robot is detected or not
    pub fn is_self_collision_detected(&self) -> bool {
        self.detect_self().next().is_some()
    }

    /// Returns whether any collision is detected or not
    /// objects: environmental objects
    pub fn is_collision_detected(&self, objects: &Compound<N>) -> bool {
        self.is_env_collision_detected(objects) | self.is_self_collision_detected()
    }
}

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
#[serde(deny_unknown_fields)]
/// Configuration struct for RobotCollisionDetector
pub struct RobotCollisionDetectorConfig {
    #[serde(default = "default_urdf_file_name")]
    pub urdf_file_name: String,
    #[serde(default = "default_prediction")]
    pub prediction: f64,
}

fn default_urdf_file_name() -> String {
    "sample.urdf".to_string()
}

fn default_prediction() -> f64 {
    0.001
}

impl RobotCollisionDetectorConfig {
    pub fn new(urdf_file_name: String, prediction: f64) -> Self {
        RobotCollisionDetectorConfig {
            urdf_file_name,
            prediction,
        }
    }
}

impl Default for RobotCollisionDetectorConfig {
    fn default() -> Self {
        Self {
            urdf_file_name: default_urdf_file_name(),
            prediction: default_prediction(),
        }
    }
}

pub fn create_robot_collision_detector(
    config: RobotCollisionDetectorConfig,
    self_collision_pairs: Vec<(String, String)>,
) -> RobotCollisionDetector<f64> {
    let urdf_path = Path::new(&config.urdf_file_name);
    let urdf_robot = urdf_rs::read_file(urdf_path).unwrap();
    let robot = k::Chain::<f64>::from(&urdf_robot);
    let collision_detector = CollisionDetector::from_urdf_robot(&urdf_robot, config.prediction);

    RobotCollisionDetector::new(robot, collision_detector, self_collision_pairs)
}

#[test]
fn test_robot_collision_detector() {
    let self_collision_pairs = vec![("root".into(), "l_shoulder_roll".into())];
    let robot_collision_detector = create_robot_collision_detector(
        RobotCollisionDetectorConfig::default(),
        self_collision_pairs,
    );

    robot_collision_detector
        .robot
        .set_joint_positions_clamped(&[0.0; 8]);
    assert!(!robot_collision_detector.is_self_collision_detected());

    robot_collision_detector
        .robot
        .set_joint_positions_clamped(&[1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!(robot_collision_detector.is_self_collision_detected());
}
