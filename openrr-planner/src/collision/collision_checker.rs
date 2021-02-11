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
use super::urdf::urdf_geometry_to_shape_handle;
use crate::errors::*;
use k::nalgebra as na;
use log::*;
use na::RealField;
use ncollide3d::{
    query,
    shape::{Compound, Shape, ShapeHandle},
};
use std::{
    collections::HashMap,
    path::Path,
    time::{Duration, Instant},
};
type NameShapeMap<T> = HashMap<String, Vec<(ShapeHandle<T>, na::Isometry3<T>)>>;

/// Check collision between robot and object
pub struct EnvCollisionNames<'a, 'b, T>
where
    T: RealField,
{
    checker: &'a CollisionChecker<T>,
    target_shape: &'b dyn Shape<T>,
    target_pose: &'b na::Isometry3<T>,
    joints: Vec<&'b k::Node<T>>,
    index: usize,
}

impl<'a, 'b, T> EnvCollisionNames<'a, 'b, T>
where
    T: RealField + k::SubsetOf<f64>,
{
    pub fn new(
        checker: &'a CollisionChecker<T>,
        robot: &'b k::Chain<T>,
        target_shape: &'b dyn Shape<T>,
        target_pose: &'b na::Isometry3<T>,
    ) -> Self {
        robot.update_transforms();
        let joints = robot.iter().collect();
        Self {
            checker,
            target_shape,
            target_pose,
            joints,
            index: 0,
        }
    }
}

impl<'a, 'b, T> Iterator for EnvCollisionNames<'a, 'b, T>
where
    T: RealField + k::SubsetOf<f64>,
{
    type Item = String;
    fn next(&mut self) -> Option<String> {
        if self.joints.len() <= self.index {
            return None;
        }
        let joint = self.joints[self.index];
        self.index += 1;
        let trans = joint.world_transform().unwrap();
        let joint_name = &joint.joint().name;
        match self.checker.name_collision_model_map.get(joint_name) {
            Some(obj_vec) => {
                for obj in obj_vec {
                    // proximity and prediction does not work for meshes.
                    let dist = query::distance(
                        &(trans * obj.1),
                        &*obj.0,
                        self.target_pose,
                        self.target_shape,
                    );
                    if dist < self.checker.prediction {
                        debug!("name: {}, dist={}", joint_name, dist);
                        return Some(joint_name.to_owned());
                    }
                }
            }
            None => {
                debug!("collision model {} not found", joint_name);
            }
        }
        self.next()
    }
}

/// Check collision inside robot links (self collision checker)
pub struct SelfCollisionPairs<'a, T>
where
    T: RealField,
{
    checker: &'a CollisionChecker<T>,
    collision_check_robot: &'a k::Chain<T>,
    self_collision_pairs: &'a [(String, String)],
    index: usize,
    used_duration: HashMap<String, Duration>,
}

impl<'a, T> SelfCollisionPairs<'a, T>
where
    T: RealField + k::SubsetOf<f64>,
{
    pub fn new(
        checker: &'a CollisionChecker<T>,
        collision_check_robot: &'a k::Chain<T>,
        self_collision_pairs: &'a [(String, String)],
    ) -> Self {
        collision_check_robot.update_transforms();
        Self {
            checker,
            collision_check_robot,
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

impl<'a, T> Iterator for SelfCollisionPairs<'a, T>
where
    T: RealField + k::SubsetOf<f64>,
{
    type Item = (String, String);
    fn next(&mut self) -> Option<(String, String)> {
        if self.self_collision_pairs.len() <= self.index {
            return None;
        }
        let (j1, j2) = &self.self_collision_pairs[self.index];
        self.index += 1;
        let obj_vec1_opt = self.checker.name_collision_model_map.get(j1);
        let obj_vec2_opt = self.checker.name_collision_model_map.get(j2);
        if obj_vec1_opt.is_none() {
            warn!("Collision model {} not found", j1);
            return self.next();
        }
        if obj_vec2_opt.is_none() {
            warn!("Collision model {} not found", j2);
            return self.next();
        }
        let node1_opt = self.collision_check_robot.find(j1);
        let node2_opt = self.collision_check_robot.find(j2);
        if node1_opt.is_none() {
            warn!("self_colliding: joint {} not found", j1);
            return self.next();
        }
        if node2_opt.is_none() {
            warn!("self_colliding: joint {} not found", j2);
            return self.next();
        }
        let obj_vec1 = obj_vec1_opt.unwrap();
        let obj_vec2 = obj_vec2_opt.unwrap();
        let node1 = node1_opt.unwrap();
        let node2 = node2_opt.unwrap();
        let mut last_time = Instant::now();
        for obj1 in obj_vec1 {
            for obj2 in obj_vec2 {
                let trans1 = node1.world_transform().unwrap();
                let trans2 = node2.world_transform().unwrap();
                // proximity and predict does not work correctly for mesh
                let dist =
                    query::distance(&(trans1 * obj1.1), &*obj1.0, &(trans2 * obj2.1), &*obj2.0);
                debug!("name: {}, name: {} dist={}", j1, j2, dist);
                if dist < self.checker.prediction {
                    return Some((j1.to_owned(), j2.to_owned()));
                }
                let elapsed = last_time.elapsed();
                *self
                    .used_duration
                    .entry(j1.to_owned())
                    .or_insert_with(|| Duration::from_nanos(0)) += elapsed;
                *self
                    .used_duration
                    .entry(j2.to_owned())
                    .or_insert_with(|| Duration::from_nanos(0)) += elapsed;
                last_time = Instant::now();
            }
        }
        self.next()
    }
}

#[derive(Clone)]
/// Collision checker for a robot
pub struct CollisionChecker<T>
where
    T: RealField,
{
    name_collision_model_map: NameShapeMap<T>,
    /// margin length for collision check
    pub prediction: T,
    pub self_collision_pairs: Vec<(String, String)>,
}

impl<T> CollisionChecker<T>
where
    T: RealField + k::SubsetOf<f64>,
{
    /// Create CollisionChecker from HashMap
    pub fn new(name_collision_model_map: NameShapeMap<T>, prediction: T) -> Self {
        CollisionChecker {
            name_collision_model_map,
            prediction,
            self_collision_pairs: Vec::new(),
        }
    }

    /// Create CollisionChecker from urdf_rs::Robot
    pub fn from_urdf_robot(urdf_robot: &urdf_rs::Robot, prediction: T) -> Self {
        Self::from_urdf_robot_with_base_dir(urdf_robot, None, prediction)
    }
    /// Create CollisionChecker from urdf_rs::Robot with base_dir support
    ///
    /// base_dir: mesh files are loaded from this dir if the path does not start with "package://"
    pub fn from_urdf_robot_with_base_dir(
        urdf_robot: &urdf_rs::Robot,
        base_dir: Option<&Path>,
        prediction: T,
    ) -> Self {
        let mut name_collision_model_map = HashMap::new();
        let link_joint_map = k::urdf::link_to_joint_map(&urdf_robot);
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
        CollisionChecker {
            name_collision_model_map,
            prediction,
            self_collision_pairs: Vec::new(),
        }
    }
    /// Check collision between environmental object and returns the names of the link(joint) names
    ///
    /// robot: robot model
    /// target_shape: Check collision with this shape and the robot
    /// target_pose: Check collision with this shape in this pose and the robot
    pub fn check_env<'a>(
        &'a self,
        robot: &'a k::Chain<T>,
        target_shape: &'a dyn Shape<T>,
        target_pose: &'a na::Isometry3<T>,
    ) -> EnvCollisionNames<T> {
        robot.update_transforms();
        EnvCollisionNames::new(self, robot, target_shape, target_pose)
    }

    /// Check self collision and return the names of the link(joint) names
    ///
    /// robot: robot model
    /// self_collision_pairs: pairs of the names of the link(joint)
    pub fn check_self<'a>(
        &'a self,
        collision_check_robot: &'a k::Chain<T>,
        self_collision_pairs: &'a [(String, String)],
    ) -> SelfCollisionPairs<T> {
        collision_check_robot.update_transforms();
        SelfCollisionPairs::new(self, collision_check_robot, self_collision_pairs)
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
                        match urdf_geometry_to_shape_handle(&collision.geometry, None) {
                            Some(col) => Some((k::urdf::isometry_from(&collision.origin), col)),
                            None => None,
                        }
                    })
                    .collect::<Vec<_>>()
            })
            .filter_map(|col_tuple| col_tuple)
            .collect::<Vec<_>>();
        Compound::new(compound_data)
    }
}
