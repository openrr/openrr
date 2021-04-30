use std::sync::Arc;

use arci::{Error, JointTrajectoryClient, TrajectoryPoint, WaitFuture};
use k::{nalgebra as na, Constraints, Isometry3};
use schemars::{gen::SchemaGenerator, schema::Schema, JsonSchema};
use serde::{Deserialize, Serialize};

pub fn isometry(x: f64, y: f64, z: f64, roll: f64, pitch: f64, yaw: f64) -> k::Isometry3<f64> {
    k::Isometry3::from_parts(
        k::Translation3::new(x, y, z),
        k::UnitQuaternion::from_euler_angles(roll, pitch, yaw),
    )
}

pub struct IkSolverParameters {
    pub allowable_position_error: f64, // unit: m
    pub allowable_angle_error: f64,    // unit: rad
    pub jacobian_multiplier: f64,
    pub num_max_try: usize,
}

pub fn create_jacobian_ik_solver(parameters: &IkSolverParameters) -> k::JacobianIkSolver<f64> {
    k::JacobianIkSolver::new(
        parameters.allowable_position_error,
        parameters.allowable_angle_error,
        parameters.jacobian_multiplier,
        parameters.num_max_try,
    )
}

pub fn create_random_jacobian_ik_solver(
    parameters: &IkSolverParameters,
) -> openrr_planner::RandomInitializeIkSolver<f64, k::JacobianIkSolver<f64>> {
    openrr_planner::RandomInitializeIkSolver::new(
        create_jacobian_ik_solver(parameters),
        parameters.num_max_try,
    )
}

pub struct IkSolverWithChain {
    ik_arm: k::SerialChain<f64>,
    ik_solver: Arc<dyn k::InverseKinematicsSolver<f64> + Send + Sync>,
    constraints: Constraints,
}

impl IkSolverWithChain {
    pub fn end_transform(&self) -> k::Isometry3<f64> {
        self.ik_arm.end_transform()
    }

    pub fn joint_positions(&self) -> Vec<f64> {
        self.ik_arm.joint_positions()
    }

    pub fn solve_with_constraints(
        &self,
        target_pose: &k::Isometry3<f64>,
        constraints: &Constraints,
    ) -> Result<(), Error> {
        self.ik_solver
            .solve_with_constraints(&self.ik_arm, &target_pose, constraints)
            .map_err(|e| Error::Other(e.into()))
    }

    pub fn solve(&self, target_pose: &k::Isometry3<f64>) -> Result<(), Error> {
        self.solve_with_constraints(target_pose, &self.constraints)
    }

    pub fn set_joint_positions_clamped(&self, positions: &[f64]) {
        self.ik_arm.set_joint_positions_clamped(positions)
    }

    pub fn new(
        arm: k::SerialChain<f64>,
        ik_solver: Arc<dyn k::InverseKinematicsSolver<f64> + Send + Sync>,
        constraints: Constraints,
    ) -> Self {
        Self {
            ik_arm: arm,
            ik_solver,
            constraints,
        }
    }

    pub fn constraints(&self) -> &Constraints {
        &self.constraints
    }

    pub fn generate_trajectory_with_interpolation(
        &self,
        current_pose: &Isometry3<f64>,
        target_pose: &Isometry3<f64>,
        duration_sec: f64,
        max_resolution: f64,
        min_number_of_points: i32,
    ) -> Result<Vec<TrajectoryPoint>, Error> {
        self.generate_trajectory_with_interpolation_and_constraints(
            current_pose,
            target_pose,
            &self.constraints,
            duration_sec,
            max_resolution,
            min_number_of_points,
        )
    }

    pub fn generate_trajectory_with_interpolation_and_constraints(
        &self,
        current_pose: &Isometry3<f64>,
        target_pose: &Isometry3<f64>,
        constraints: &Constraints,
        duration_sec: f64,
        max_resolution: f64,
        min_number_of_points: i32,
    ) -> Result<Vec<TrajectoryPoint>, Error> {
        let target_position = target_pose.translation.vector;
        let target_rotation = target_pose.rotation;
        let current_position = current_pose.translation.vector;
        let current_rotation = current_pose.rotation;

        let position_diff = target_position - current_position;
        let n = std::cmp::max(
            min_number_of_points,
            (position_diff.norm() / max_resolution) as i32,
        );
        let mut traj = vec![];
        for i in 1..n + 1 {
            let t = i as f64 / n as f64;
            let tar_pos = current_position.lerp(&target_position, t);
            let tar_rot = current_rotation.slerp(&target_rotation, t);
            self.solve_with_constraints(
                &k::Isometry3::from_parts(na::Translation3::from(tar_pos), tar_rot),
                constraints,
            )?;
            let trajectory = TrajectoryPoint::new(
                self.joint_positions(),
                std::time::Duration::from_secs_f64(t * duration_sec),
            );
            traj.push(trajectory);
        }
        Ok(traj)
    }
}

pub struct IkClient<T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    pub ik_solver_with_chain: Arc<IkSolverWithChain>,
}

impl<T> IkClient<T>
where
    T: JointTrajectoryClient + Send,
{
    pub fn new(client: T, ik_solver_with_chain: Arc<IkSolverWithChain>) -> Self {
        if ik_solver_with_chain.ik_arm.dof() != client.joint_names().len() {
            panic!(
                "Invalid configuration : ik arm dof {} {:?} != joint_names length {} ({:?})",
                ik_solver_with_chain.ik_arm.dof(),
                ik_solver_with_chain
                    .ik_arm
                    .iter_joints()
                    .map(|j| j.name.to_owned())
                    .collect::<Vec<_>>(),
                client.joint_names().len(),
                client.joint_names()
            );
        }
        Self {
            client,
            ik_solver_with_chain,
        }
    }

    pub fn current_end_transform(&self) -> Result<k::Isometry3<f64>, Error> {
        let current_joint_angles = self.client.current_joint_positions()?;
        self.set_joint_positions_clamped(&current_joint_angles);
        Ok(self.ik_solver_with_chain.end_transform())
    }

    pub fn move_ik_with_constraints(
        &self,
        target_pose: &k::Isometry3<f64>,
        constraints: &Constraints,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        self.ik_solver_with_chain
            .solve_with_constraints(&target_pose, constraints)?;

        let positions = self.ik_solver_with_chain.joint_positions();
        let duration = std::time::Duration::from_secs_f64(duration_sec);
        self.client.send_joint_positions(positions, duration)
    }

    pub fn move_ik_with_interpolation_and_constraints(
        &self,
        target_pose: &k::Isometry3<f64>,
        constraints: &Constraints,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        let mut traj = self
            .ik_solver_with_chain
            .generate_trajectory_with_interpolation_and_constraints(
                &self.current_end_transform()?,
                target_pose,
                constraints,
                duration_sec,
                0.05,
                10,
            )?;
        let dof = self.client.joint_names().len();
        traj.first_mut().unwrap().velocities = Some(vec![0.0; dof]);
        traj.last_mut().unwrap().velocities = Some(vec![0.0; dof]);
        self.client.send_joint_trajectory(traj)
    }

    pub fn move_ik(
        &self,
        target_pose: &k::Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        self.ik_solver_with_chain.solve(&target_pose)?;

        let positions = self.ik_solver_with_chain.joint_positions();
        let duration = std::time::Duration::from_secs_f64(duration_sec);
        self.client.send_joint_positions(positions, duration)
    }

    pub fn move_ik_with_interpolation(
        &self,
        target_pose: &k::Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<WaitFuture, Error> {
        let mut traj = self
            .ik_solver_with_chain
            .generate_trajectory_with_interpolation(
                &self.current_end_transform()?,
                target_pose,
                duration_sec,
                0.05,
                10,
            )?;
        let dof = self.client.joint_names().len();
        traj.first_mut().unwrap().velocities = Some(vec![0.0; dof]);
        traj.last_mut().unwrap().velocities = Some(vec![0.0; dof]);
        self.client.send_joint_trajectory(traj)
    }

    /// Get relative pose from current pose of the IK target
    pub fn transform(&self, relative_pose: &k::Isometry3<f64>) -> Result<k::Isometry3<f64>, Error> {
        Ok(self.current_end_transform()? * relative_pose)
    }

    // Reset the kinematic model for IK calculation like Jacobian method
    pub fn set_zero_pose_for_kinematics(&self) -> Result<(), Error> {
        let zero_angles = vec![0.0; self.ik_solver_with_chain.ik_arm.dof()];
        self.ik_solver_with_chain
            .ik_arm
            .set_joint_positions(&zero_angles)
            .map_err(|e| Error::Other(e.into()))?;
        Ok(())
    }

    pub fn constraints(&self) -> &Constraints {
        &self.ik_solver_with_chain.constraints()
    }

    pub fn set_joint_positions_clamped(&self, positions: &[f64]) {
        self.ik_solver_with_chain
            .set_joint_positions_clamped(positions)
    }
}

impl<T> JointTrajectoryClient for IkClient<T>
where
    T: JointTrajectoryClient,
{
    fn joint_names(&self) -> &[String] {
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
        self.client.send_joint_positions(positions, duration)
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        self.client.send_joint_trajectory(trajectory)
    }
}

#[derive(Clone, Serialize, Deserialize, Debug, JsonSchema)]
pub struct IkSolverConfig {
    pub root_node_name: Option<String>,
    pub ik_target: String,
    #[serde(default)]
    pub use_random_ik: bool,
    #[serde(default = "default_allowable_position_error_m")]
    pub allowable_position_error_m: f64,
    #[serde(default = "default_allowable_angle_error_rad")]
    pub allowable_angle_error_rad: f64,
    #[serde(default = "default_jacobian_multiplier")]
    pub jacobian_multiplier: f64,
    #[serde(default = "default_num_max_try")]
    pub num_max_try: usize,
    #[serde(default)]
    #[schemars(schema_with = "constraints_schema")]
    pub constraints: Constraints,
}

fn default_allowable_position_error_m() -> f64 {
    0.005
}
fn default_allowable_angle_error_rad() -> f64 {
    0.005
}
fn default_jacobian_multiplier() -> f64 {
    0.1
}
fn default_num_max_try() -> usize {
    300
}

fn constraints_schema(gen: &mut SchemaGenerator) -> Schema {
    fn default_true() -> bool {
        true
    }
    // https://docs.rs/k/0.24/k/struct.Constraints.html
    /// A bundle of flags determining which coordinates are constrained for a target
    #[allow(dead_code)]
    #[derive(Serialize, JsonSchema)]
    struct ConstraintsSchema {
        /// true means the constraint is used.
        ///  The coordinates is the world, not the end of the arm.
        #[serde(default = "default_true")]
        position_x: bool,
        #[serde(default = "default_true")]
        position_y: bool,
        #[serde(default = "default_true")]
        position_z: bool,
        #[serde(default = "default_true")]
        rotation_x: bool,
        #[serde(default = "default_true")]
        rotation_y: bool,
        #[serde(default = "default_true")]
        rotation_z: bool,
    }
    impl Default for ConstraintsSchema {
        fn default() -> Self {
            Self {
                position_x: default_true(),
                position_y: default_true(),
                position_z: default_true(),
                rotation_x: default_true(),
                rotation_y: default_true(),
                rotation_z: default_true(),
            }
        }
    }

    ConstraintsSchema::json_schema(gen)
}

pub fn create_ik_solver_with_chain(
    full_chain: &k::Chain<f64>,
    config: &IkSolverConfig,
) -> IkSolverWithChain {
    let chain = if let Some(root_node_name) = &config.root_node_name {
        k::SerialChain::from_end_to_root(
            full_chain.find(&config.ik_target).unwrap(),
            full_chain.find(root_node_name).unwrap(),
        )
    } else {
        k::SerialChain::from_end(full_chain.find(&config.ik_target).unwrap())
    };

    let parameters = IkSolverParameters {
        allowable_position_error: config.allowable_position_error_m,
        allowable_angle_error: config.allowable_angle_error_rad,
        jacobian_multiplier: config.jacobian_multiplier,
        num_max_try: config.num_max_try,
    };

    IkSolverWithChain::new(
        chain,
        if config.use_random_ik {
            Arc::new(create_random_jacobian_ik_solver(&parameters))
        } else {
            Arc::new(create_jacobian_ik_solver(&parameters))
        },
        config.constraints,
    )
}
