/*
Copyright 2017 Takashi Ogura

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
use std::{
    collections::HashMap,
    path::Path,
    time::{Duration, Instant},
};

use k::nalgebra as na;
use na::RealField;
use ncollide3d::{
    partitioning::{BVH, BVT},
    query,
    shape::{Compound, Shape, ShapeHandle},
};
use tracing::{debug, warn};

use super::urdf::{k_link_geometry_to_shape_handle, urdf_geometry_to_shape_handle};
use crate::errors::*;

type NameShapeMap<T> = HashMap<String, Vec<(ShapeHandle<T>, na::Isometry3<T>)>>;

/// Lists collisions between a robot and an object
pub struct EnvCollisionNames<'a, 'b, T>
where
    T: RealField + Copy,
{
    detector: &'a CollisionDetector<T>,
    target_shape: &'b dyn Shape<T>,
    target_pose: &'b na::Isometry3<T>,
    joints: Vec<&'b k::Node<T>>,
    index: usize,
}

impl<'a, 'b, T> EnvCollisionNames<'a, 'b, T>
where
    T: RealField + Copy + k::SubsetOf<f64>,
{
    pub fn new(
        detector: &'a CollisionDetector<T>,
        robot: &'b k::Chain<T>,
        target_shape: &'b dyn Shape<T>,
        target_pose: &'b na::Isometry3<T>,
    ) -> Self {
        robot.update_transforms();
        let joints = robot.iter().collect();
        Self {
            detector,
            target_shape,
            target_pose,
            joints,
            index: 0,
        }
    }
}

impl<T> Iterator for EnvCollisionNames<'_, '_, T>
where
    T: RealField + Copy + k::SubsetOf<f64>,
{
    type Item = String;

    fn next(&mut self) -> Option<String> {
        if self.joints.len() <= self.index {
            return None;
        }
        let joint = self.joints[self.index];
        self.index += 1;

        // Get the absolute (world) pose and name of the specified node
        let joint_pose = joint.world_transform().unwrap();
        let joint_name = &joint.joint().name;

        match self.detector.name_collision_model_map.get(joint_name) {
            Some(obj_vec) => {
                // Check potential conflicts by an AABB-based sweep and prune algorithm
                for obj in obj_vec {
                    let obj_pose = joint_pose * obj.1;
                    let aabb1 = obj.0.aabb(&(obj_pose));
                    let aabb2 = self.target_shape.aabb(self.target_pose);

                    let bvt = BVT::new_balanced(vec![(0usize, aabb2)]);

                    let mut collector = Vec::<usize>::new();
                    let mut visitor = query::visitors::BoundingVolumeInterferencesCollector::new(
                        &aabb1,
                        &mut collector,
                    );

                    bvt.visit(&mut visitor);

                    if !collector.is_empty() {
                        // Check conflicts precisely
                        let dist = query::distance(
                            &obj_pose,
                            &*obj.0,
                            self.target_pose,
                            self.target_shape,
                        );
                        // proximity and prediction does not work correctly for meshes.
                        if dist < self.detector.prediction {
                            debug!("name: {joint_name}, dist={dist}");
                            return Some(joint_name.to_owned());
                        }
                    }
                }
            }
            None => {
                debug!("collision model {joint_name} not found");
            }
        }
        self.next()
    }
}

/// Lists collisions inside robot links
pub struct SelfCollisionPairs<'a, T>
where
    T: RealField + Copy,
{
    detector: &'a CollisionDetector<T>,
    robot: &'a k::Chain<T>,
    self_collision_pairs: &'a [(String, String)],
    index: usize,
    used_duration: HashMap<String, Duration>,
}

impl<'a, T> SelfCollisionPairs<'a, T>
where
    T: RealField + Copy + k::SubsetOf<f64>,
{
    pub fn new(
        detector: &'a CollisionDetector<T>,
        robot: &'a k::Chain<T>,
        self_collision_pairs: &'a [(String, String)],
    ) -> Self {
        robot.update_transforms();
        Self {
            detector,
            robot,
            self_collision_pairs,
            index: 0,
            used_duration: HashMap::new(),
        }
    }

    /// Get the information about which part is the most heaviest.
    pub fn used_duration(&self) -> &HashMap<String, Duration> {
        &self.used_duration
    }
}

impl<T> Iterator for SelfCollisionPairs<'_, T>
where
    T: RealField + Copy + k::SubsetOf<f64>,
{
    type Item = (String, String);

    fn next(&mut self) -> Option<(String, String)> {
        if self.self_collision_pairs.len() <= self.index {
            return None;
        }
        let (j1, j2) = &self.self_collision_pairs[self.index];
        self.index += 1;

        // Get the collision models of the specified 2 nodes
        let obj_vec1 = match self.detector.name_collision_model_map.get(j1) {
            Some(v) => v,
            None => {
                warn!("Collision model {j1} not found");
                return self.next();
            }
        };
        let obj_vec2 = match self.detector.name_collision_model_map.get(j2) {
            Some(v) => v,
            None => {
                warn!("Collision model {j2} not found");
                return self.next();
            }
        };

        // Get the absolute (world) poses of the specified 2 nodes
        let pose1 = match self.robot.find(j1) {
            Some(v) => v.world_transform().unwrap(),
            None => {
                warn!("self_colliding: joint {j1} not found");
                return self.next();
            }
        };
        let pose2 = match self.robot.find(j2) {
            Some(v) => v.world_transform().unwrap(),
            None => {
                warn!("self_colliding: joint {j2} not found");
                return self.next();
            }
        };

        // Check potential conflicts by an AABB-based sweep and prune algorithm
        let start_time = Instant::now();
        for obj1 in obj_vec1 {
            let aabb1 = obj1.0.aabb(&(pose1 * obj1.1));

            let index_and_aabb = obj_vec2
                .iter()
                .enumerate()
                .map(|(index, obj)| (index, obj.0.aabb(&(pose2 * obj.1))))
                .collect();
            let bvt = BVT::new_balanced(index_and_aabb);

            let mut collector = Vec::<usize>::new();
            let mut visitor =
                query::visitors::BoundingVolumeInterferencesCollector::new(&aabb1, &mut collector);

            bvt.visit(&mut visitor);

            if !collector.is_empty() {
                // Check conflicts precisely
                for index in collector {
                    let obj2 = &obj_vec2[index];

                    // proximity and predict does not work correctly for mesh
                    let dist =
                        query::distance(&(pose1 * obj1.1), &*obj1.0, &(pose2 * obj2.1), &*obj2.0);
                    debug!("name: {j1}, name: {j2} dist={dist}");
                    if dist < self.detector.prediction {
                        return Some((j1.to_owned(), j2.to_owned()));
                    }
                }
            }
        }

        // Record the time used for this collision checking
        let elapsed = start_time.elapsed();
        *self
            .used_duration
            .entry(j1.to_owned())
            .or_insert_with(|| Duration::from_nanos(0)) += elapsed;
        *self
            .used_duration
            .entry(j2.to_owned())
            .or_insert_with(|| Duration::from_nanos(0)) += elapsed;

        self.next()
    }
}

#[derive(Clone)]
/// Collision detector
pub struct CollisionDetector<T>
where
    T: RealField + Copy,
{
    name_collision_model_map: NameShapeMap<T>,
    /// margin length for collision detection
    pub prediction: T,
}

impl<T> CollisionDetector<T>
where
    T: RealField + Copy + k::SubsetOf<f64>,
{
    /// Create CollisionDetector from HashMap
    pub fn new(name_collision_model_map: NameShapeMap<T>, prediction: T) -> Self {
        CollisionDetector {
            name_collision_model_map,
            prediction,
        }
    }

    /// Create CollisionDetector from urdf_rs::Robot
    pub fn from_urdf_robot(urdf_robot: &urdf_rs::Robot, prediction: T) -> Self {
        Self::from_urdf_robot_with_base_dir(urdf_robot, None, prediction)
    }

    /// Create CollisionDetector from urdf_rs::Robot with base_dir support
    ///
    /// base_dir: mesh files are loaded from this dir if the path does not start with "package://"
    pub fn from_urdf_robot_with_base_dir(
        urdf_robot: &urdf_rs::Robot,
        base_dir: Option<&Path>,
        prediction: T,
    ) -> Self {
        let mut name_collision_model_map = HashMap::new();
        let link_joint_map = k::urdf::link_to_joint_map(urdf_robot);
        for l in &urdf_robot.links {
            let col_pose_vec = l
                .collision
                .iter()
                .filter_map(|collision| {
                    urdf_geometry_to_shape_handle(&collision.geometry, base_dir)
                        .map(|col| (col, k::urdf::isometry_from(&collision.origin)))
                })
                .collect::<Vec<_>>();
            debug!("name={}, ln={}", l.name, col_pose_vec.len());
            if !col_pose_vec.is_empty() {
                if let Some(joint_name) = link_joint_map.get(&l.name) {
                    name_collision_model_map.insert(joint_name.to_owned(), col_pose_vec);
                }
            }
        }
        CollisionDetector {
            name_collision_model_map,
            prediction,
        }
    }

    /// Create CollisionDetector from k::Chain
    pub fn from_robot(robot: &k::Chain<T>, prediction: T) -> Self {
        let mut name_collision_model_map = HashMap::new();
        for node in robot.iter() {
            let link = match node.link().clone() {
                Some(v) => v,
                None => break,
            };

            let col_pose_vec = link
                .collisions
                .iter()
                .filter_map(|c| {
                    k_link_geometry_to_shape_handle(&c.geometry).map(|col| (col, *c.origin()))
                })
                .collect::<Vec<_>>();

            debug!("name={}, ln={}", link.name, col_pose_vec.len());
            if !col_pose_vec.is_empty() {
                name_collision_model_map.insert(node.joint().name.to_owned(), col_pose_vec);
            }
        }
        CollisionDetector {
            name_collision_model_map,
            prediction,
        }
    }

    /// Detects collisions of a robot with an environmental object and returns the names of the link(joint) names
    ///
    /// robot: robot model
    /// target_shape: shape of the environmental object
    /// target_pose: pose of the environmental object
    pub fn detect_env<'a>(
        &'a self,
        robot: &'a k::Chain<T>,
        target_shape: &'a dyn Shape<T>,
        target_pose: &'a na::Isometry3<T>,
    ) -> EnvCollisionNames<'a, 'a, T> {
        robot.update_transforms();
        EnvCollisionNames::new(self, robot, target_shape, target_pose)
    }

    /// Detects self collisions and returns the names of the link(joint) names
    ///
    /// robot: robot model
    /// self_collision_pairs: pairs of the names of the link(joint)
    pub fn detect_self<'a>(
        &'a self,
        robot: &'a k::Chain<T>,
        self_collision_pairs: &'a [(String, String)],
    ) -> SelfCollisionPairs<'a, T> {
        robot.update_transforms();
        SelfCollisionPairs::new(self, robot, self_collision_pairs)
    }
}

#[cfg(test)]
mod tests {
    use na::{Isometry3, Vector3};
    use ncollide3d::shape::Cuboid;

    use super::*;

    #[test]
    fn test_environmental_collision_detection() {
        let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
        let robot = k::Chain::<f32>::from(&urdf_robot);
        robot.set_joint_positions(&[0.0; 16]).unwrap();
        let detector = CollisionDetector::from_urdf_robot(&urdf_robot, 0.01);

        // This target is based on `obj2` in `obstacles.urdf`.
        let target = Cuboid::new(Vector3::new(0.2, 0.3, 0.1));
        let target_pose = Isometry3::new(Vector3::new(0.7, 0.0, 0.6), na::zero());

        assert!(detector
            .detect_env(&robot, &target, &target_pose)
            .next()
            .is_none());

        let angles = [
            -0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ];
        robot.set_joint_positions(&angles).unwrap();
        let names: Vec<String> = detector.detect_env(&robot, &target, &target_pose).collect();
        assert_eq!(
            names,
            vec![
                "l_wrist_yaw",
                "l_wrist_pitch",
                "l_gripper_linear1",
                "l_gripper_linear2"
            ]
        );

        let target_pose = Isometry3::new(Vector3::new(0.0, 0.0, 0.0), na::zero());
        let names: Vec<String> = detector.detect_env(&robot, &target, &target_pose).collect();
        assert_eq!(names, vec!["root"]);
    }

    #[test]
    fn test_self_collision_detection() {
        let urdf_robot = urdf_rs::read_file("sample.urdf").unwrap();
        let robot = k::Chain::<f32>::from(&urdf_robot);
        let detector = CollisionDetector::from_urdf_robot(&urdf_robot, 0.01);

        let collision_check_pairs = parse_colon_separated_pairs(&[
            "root:l_shoulder_roll".to_owned(),
            "root:l_elbow_pitch".to_owned(),
            "root:l_wrist_yaw".to_owned(),
            "root:l_wrist_pitch".to_owned(),
        ])
        .unwrap();
        let (correct_collisions, _) = collision_check_pairs.split_at(2);

        let angles = [0.0; 16];
        robot.set_joint_positions(&angles).unwrap();
        assert!(detector
            .detect_self(&robot, &collision_check_pairs)
            .next()
            .is_none());

        let angles = [
            -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ];
        robot.set_joint_positions(&angles).unwrap();
        let result: Vec<(String, String)> = detector
            .detect_self(&robot, &collision_check_pairs)
            .collect();
        assert_eq!(result, correct_collisions.to_vec());
    }
}

/// Convert urdf object into openrr_planner/ncollide3d object
pub trait FromUrdf {
    fn from_urdf_robot(robot: &urdf_rs::Robot) -> Self;
    fn from_urdf_file<P>(path: P) -> ::std::result::Result<Self, urdf_rs::UrdfError>
    where
        Self: ::std::marker::Sized,
        P: AsRef<Path>,
    {
        Ok(Self::from_urdf_robot(&urdf_rs::read_file(path)?))
    }
}

/// Parse args to get self collision pair
///
/// # Example
///
/// ```
/// let pairs = openrr_planner::collision::parse_colon_separated_pairs(&vec!["ab:cd".to_owned(), "ab:ef".to_owned()]).unwrap();
/// assert_eq!(pairs.len(), 2);
/// assert_eq!(pairs[0].0, "ab");
/// assert_eq!(pairs[0].1, "cd");
/// assert_eq!(pairs[1].0, "ab");
/// assert_eq!(pairs[1].1, "ef");
/// ```
pub fn parse_colon_separated_pairs(pair_strs: &[String]) -> Result<Vec<(String, String)>> {
    let mut pairs = Vec::new();
    for pair_str in pair_strs {
        let mut sp = pair_str.split(':');
        if let Some(p1) = sp.next() {
            if let Some(p2) = sp.next() {
                pairs.push((p1.to_owned(), p2.to_owned()));
            } else {
                return Err(Error::ParseError(pair_str.to_owned()));
            }
        } else {
            return Err(Error::ParseError(pair_str.to_owned()));
        }
    }
    Ok(pairs)
}

#[cfg(test)]
mod test {
    use super::parse_colon_separated_pairs;
    #[test]
    fn test_parse_colon_separated_pairs() {
        let pairs = parse_colon_separated_pairs(&["j0:j1".to_owned(), "j2:j0".to_owned()]).unwrap();
        assert_eq!(pairs.len(), 2);
        assert_eq!(pairs[0].0, "j0");
        assert_eq!(pairs[0].1, "j1");
        assert_eq!(pairs[1].0, "j2");
        assert_eq!(pairs[1].1, "j0");
    }
}

/// Create `ncollide::shape::Compound` from URDF file
///
/// The `<link>` elements are used as obstacles. set the origin/geometry of
/// `<visual>` and `<collision>`. You can skip `<inertia>`.
impl FromUrdf for Compound<f64> {
    fn from_urdf_robot(urdf_obstacle: &urdf_rs::Robot) -> Self {
        let compound_data = urdf_obstacle
            .links
            .iter()
            .flat_map(|l| {
                l.collision
                    .iter()
                    .map(|collision| {
                        urdf_geometry_to_shape_handle(&collision.geometry, None)
                            .map(|col| (k::urdf::isometry_from(&collision.origin), col))
                    })
                    .collect::<Vec<_>>()
            })
            .flatten()
            .collect::<Vec<_>>();
        Compound::new(compound_data)
    }
}
