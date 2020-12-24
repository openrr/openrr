use arci::{Error, JointTrajectoryClient, TrajectoryPoint};
use async_trait::async_trait;
use k::nalgebra as na;
use k::Isometry3;
use std::sync::Arc;

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

pub fn create_jacobian_ik_solver(parameters: &IkSolverParameters) -> k::JacobianIKSolver<f64> {
    k::JacobianIKSolver::new(
        parameters.allowable_position_error,
        parameters.allowable_angle_error,
        parameters.jacobian_multiplier,
        parameters.num_max_try,
    )
}

pub fn create_random_jacobian_ik_solver(
    parameters: &IkSolverParameters,
) -> openrr_planner::RandomInitializeIKSolver<f64, k::JacobianIKSolver<f64>> {
    openrr_planner::RandomInitializeIKSolver::new(
        create_jacobian_ik_solver(parameters),
        parameters.num_max_try,
    )
}

pub struct IkSolverWithChain {
    ik_arm: k::SerialChain<f64>,
    ik_solver: Arc<dyn k::InverseKinematicsSolver<f64> + Send + Sync>,
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
        constraints: &k::Constraints,
    ) -> Result<(), Error> {
        self.ik_solver
            .solve_with_constraints(&self.ik_arm, &target_pose, constraints)
            .map_err(|e| Error::Other(e.into()))
    }
    pub fn set_joint_positions_clamped(&self, positions: &[f64]) {
        self.ik_arm.set_joint_positions_clamped(positions)
    }
    pub fn new(
        arm: k::SerialChain<f64>,
        ik_solver: Arc<dyn k::InverseKinematicsSolver<f64> + Send + Sync>,
    ) -> Self {
        Self {
            ik_arm: arm,
            ik_solver,
        }
    }
}

pub struct IkClient<T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    pub ik_solver_with_chain: IkSolverWithChain,
    pub chain: k::Chain<f64>,
    pub constraints: k::Constraints,
}

impl<T> IkClient<T>
where
    T: JointTrajectoryClient + Send,
{
    pub fn new(client: T, ik_solver_with_chain: IkSolverWithChain, chain: k::Chain<f64>) -> Self {
        Self {
            client,
            ik_solver_with_chain,
            chain,
            constraints: k::Constraints::default(),
        }
    }

    pub fn current_end_transform(&self) -> Result<k::Isometry3<f64>, Error> {
        let current_joint_angles = self.client.current_joint_positions()?;
        self.chain
            .set_joint_positions_clamped(&current_joint_angles);
        Ok(self.ik_solver_with_chain.end_transform())
    }

    pub async fn move_ik_with_constraints(
        &self,
        target_pose: &k::Isometry3<f64>,
        constraints: &k::Constraints,
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.ik_solver_with_chain
            .solve_with_constraints(&target_pose, constraints)?;

        let positions = self.ik_solver_with_chain.joint_positions();
        let duration = std::time::Duration::from_secs_f64(duration_sec);
        self.client.send_joint_positions(positions, duration).await
    }

    pub async fn move_ik_with_interpolation_and_constraints(
        &self,
        target_pose: &k::Isometry3<f64>,
        constraints: &k::Constraints,
        duration_sec: f64,
    ) -> Result<(), Error> {
        let mut traj = check_ik_with_interpolation_and_constraints(
            &self.current_end_transform()?,
            target_pose,
            &self.ik_solver_with_chain,
            constraints,
            duration_sec,
            0.05,
            10,
        )?;
        let dof = self.client.joint_names().len();
        traj.first_mut().unwrap().velocities = Some(vec![0.0; dof]);
        traj.last_mut().unwrap().velocities = Some(vec![0.0; dof]);
        self.client.send_joint_trajectory(traj).await
    }

    pub async fn move_ik(
        &self,
        target_pose: &k::Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.move_ik_with_constraints(target_pose, &self.constraints, duration_sec)
            .await
    }

    pub async fn move_ik_with_interpolation(
        &self,
        target_pose: &k::Isometry3<f64>,
        duration_sec: f64,
    ) -> Result<(), Error> {
        self.move_ik_with_interpolation_and_constraints(
            target_pose,
            &self.constraints,
            duration_sec,
        )
        .await
    }

    /// Get relative pose from current pose of the IK target
    pub fn transform(&self, relative_pose: &k::Isometry3<f64>) -> Result<k::Isometry3<f64>, Error> {
        let current_joint_angles = self.client.current_joint_positions()?;
        self.chain
            .set_joint_positions_clamped(&current_joint_angles);
        let current_target_pose = self.ik_solver_with_chain.end_transform();
        Ok(current_target_pose * relative_pose)
    }
    // Reset the kinematic model for IK calculation like Jacobian method
    pub fn set_zero_pose_for_kinematics(&self) -> Result<(), Error> {
        let zero_angles = vec![0.0; self.chain.dof()];
        self.chain
            .set_joint_positions(&zero_angles)
            .map_err(|e| Error::Other(e.into()))?;
        Ok(())
    }
}

#[async_trait]
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
    async fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: std::time::Duration,
    ) -> Result<(), Error> {
        self.client.send_joint_positions(positions, duration).await
    }
    async fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<(), Error> {
        self.client.send_joint_trajectory(trajectory).await
    }
}

pub fn check_ik_with_interpolation_and_constraints(
    current_pose: &Isometry3<f64>,
    target_pose: &Isometry3<f64>,
    ik_solver_with_chain: &IkSolverWithChain,
    constraints: &k::Constraints,
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
        ik_solver_with_chain.solve_with_constraints(
            &k::Isometry3::from_parts(na::Translation3::from(tar_pos), tar_rot),
            constraints,
        )?;
        let trajectory = TrajectoryPoint::new(
            ik_solver_with_chain.joint_positions(),
            std::time::Duration::from_secs_f64(t * duration_sec),
        );
        traj.push(trajectory);
    }
    Ok(traj)
}
