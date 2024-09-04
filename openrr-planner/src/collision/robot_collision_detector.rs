use std::path::Path;

use k::nalgebra as na;
use na::RealField;
use ncollide3d::shape::{Compound, Shape};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::collision::{CollisionDetector, EnvCollisionNames, SelfCollisionPairs};

pub struct RobotCollisionDetector<N>
where
    N: RealField + Copy + k::SubsetOf<f64>,
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
    N: RealField + Copy + k::SubsetOf<f64>,
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
    #[serde(default = "default_prediction")]
    pub prediction: f64,
}

fn default_prediction() -> f64 {
    0.001
}

impl RobotCollisionDetectorConfig {
    pub fn new(prediction: f64) -> Self {
        RobotCollisionDetectorConfig { prediction }
    }
}

impl Default for RobotCollisionDetectorConfig {
    fn default() -> Self {
        Self {
            prediction: default_prediction(),
        }
    }
}

pub fn create_robot_collision_detector<P: AsRef<Path>>(
    urdf_path: P,
    config: RobotCollisionDetectorConfig,
    self_collision_pairs: Vec<(String, String)>,
) -> RobotCollisionDetector<f64> {
    let urdf_robot = urdf_rs::read_file(&urdf_path).unwrap();
    let robot = k::Chain::<f64>::from(&urdf_robot);
    let collision_detector = CollisionDetector::from_urdf_robot_with_base_dir(
        &urdf_robot,
        urdf_path.as_ref().parent(),
        config.prediction,
    );

    RobotCollisionDetector::new(robot, collision_detector, self_collision_pairs)
}

#[test]
fn test_robot_collision_detector() {
    let urdf_path = Path::new("sample.urdf");
    let self_collision_pairs = vec![("root".into(), "l_shoulder_roll".into())];
    let robot_collision_detector = create_robot_collision_detector(
        urdf_path,
        RobotCollisionDetectorConfig::default(),
        self_collision_pairs,
    );

    robot_collision_detector
        .robot
        .set_joint_positions_clamped(&[0.0; 16]);
    assert!(!robot_collision_detector.is_self_collision_detected());

    robot_collision_detector
        .robot
        .set_joint_positions_clamped(&[
            -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);
    assert!(robot_collision_detector.is_self_collision_detected());
}

/// Lists all potentially-colliding pairs from a robot chain
///
/// robot: robot model
pub fn create_all_collision_pairs<N>(robot: &k::Chain<N>) -> Vec<(String, String)>
where
    N: RealField + Copy + k::SubsetOf<f64>,
{
    let mut pairs: Vec<(String, String)> = Vec::new();

    for node1 in robot.iter() {
        let joint1_name = node1.joint().name.clone();
        for node2 in robot.iter() {
            let joint2_name = node2.joint().name.clone();

            // A flag to ignore a self-pair and a pair of the parent and child
            let is_this_pair_valid = !(joint1_name == joint2_name
                || (node1.parent().is_some()
                    && node1.parent().unwrap().joint().name == joint2_name)
                || (node2.parent().is_some()
                    && node2.parent().unwrap().joint().name == joint1_name));

            // Index the names in dictionary order to clarify duplicates
            if is_this_pair_valid {
                let pair = if joint1_name < joint2_name {
                    (
                        joint1_name.clone().to_owned(),
                        joint2_name.clone().to_owned(),
                    )
                } else {
                    (
                        joint2_name.clone().to_owned(),
                        joint1_name.clone().to_owned(),
                    )
                };
                pairs.push(pair);
            }
        }
    }

    // Remove all duplicates generated by combinatorial symmetry
    pairs.sort();
    pairs.dedup();

    pairs
}

#[test]
fn test_create_all_collision_pairs() {
    let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
    let robot = k::Chain::<f32>::from(&urdf_robot);

    let node = robot.iter().take(1).cloned().collect();
    let trivial_chain = k::Chain::from_nodes(node);
    assert_eq!(create_all_collision_pairs(&trivial_chain).len(), 0);

    let nodes = robot.iter().take(5).cloned().collect();
    let sub_chain = k::Chain::from_nodes(nodes);

    // Created pairs are:
    // [("r_elbow_pitch", "r_shoulder_pitch"), ("r_elbow_pitch", "r_shoulder_yaw"),
    //  ("r_elbow_pitch", "root"), ("r_shoulder_pitch", "root"),
    //  ("r_shoulder_roll", "r_shoulder_yaw"), ("r_shoulder_roll", "root")] .
    assert_eq!(create_all_collision_pairs(&sub_chain).len(), 6);
}

#[test]
fn test_create_all_collision_pairs_without_urdf() {
    use k::{joint::*, node::*};

    let joint0 = NodeBuilder::new()
        .name("joint0")
        .translation(na::Translation3::new(0.1, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint1 = NodeBuilder::new()
        .name("joint1")
        .translation(na::Translation3::new(0.1, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint2 = NodeBuilder::new()
        .name("joint2")
        .translation(na::Translation3::new(0.1, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    let joint3 = NodeBuilder::new()
        .name("joint3")
        .translation(na::Translation3::new(0.1, 0.0, 0.0))
        .joint_type(JointType::Rotational {
            axis: na::Vector3::y_axis(),
        })
        .into_node();
    joint1.set_parent(&joint0);
    joint2.set_parent(&joint1);
    joint3.set_parent(&joint2);

    let chain = k::Chain::from_root(joint0);

    // Created pairs are:
    // [("joint0", "joint2"), ("joint0", "joint3"), ("joint1", "joint3")]
    assert_eq!(create_all_collision_pairs(&chain).len(), 3);
}
