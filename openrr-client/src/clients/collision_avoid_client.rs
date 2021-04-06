use arci::{Error, JointTrajectoryClient, TrajectoryPoint, WaitFuture};

// TODO: speed limit
fn trajectory_from_positions(
    positions: &[Vec<f64>],
    total_duration: std::time::Duration,
) -> Vec<TrajectoryPoint> {
    let num_points = positions.len();
    let mut traj = vec![];
    for (i, pos) in positions.iter().enumerate() {
        let time_rate: f64 = ((i + 1) as f64) / (num_points as f64);
        traj.push(TrajectoryPoint::new(
            pos.clone(),
            total_duration.mul_f64(time_rate),
        ));
    }
    traj
}

pub struct CollisionAvoidClient<'a, T>
where
    T: JointTrajectoryClient,
{
    pub client: T,
    /// using_joints and collision_check_robot must share the k::Node instance.
    pub using_joints: k::Chain<f64>,
    pub collision_check_robot: &'a k::Chain<f64>,
    pub planner: openrr_planner::JointPathPlanner<f64>,
}

impl<'a, T> CollisionAvoidClient<'a, T>
where
    T: JointTrajectoryClient,
{
    pub fn new(
        client: T,
        using_joints: k::Chain<f64>,
        collision_check_robot: &'a k::Chain<f64>,
        planner: openrr_planner::JointPathPlanner<f64>,
    ) -> Self {
        Self {
            client,
            collision_check_robot,
            using_joints,
            planner,
        }
    }
}

impl<'a, T> JointTrajectoryClient for CollisionAvoidClient<'a, T>
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
        self.using_joints
            .set_joint_positions_clamped(&self.current_joint_positions()?);
        let current = self.using_joints.joint_positions();
        let traj = self
            .planner
            .plan_avoid_self_collision(&self.using_joints, &current, &positions)
            .map_err(|e| Error::Other(e.into()))?;
        self.client
            .send_joint_trajectory(trajectory_from_positions(&traj, duration))
    }
    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        if trajectory.is_empty() {
            return Ok(WaitFuture::ready());
        }
        self.using_joints
            .set_joint_positions_clamped(&self.current_joint_positions()?);
        let current = self.using_joints.joint_positions();
        let positions = self
            .planner
            .plan_avoid_self_collision(&self.using_joints, &current, &trajectory[0].positions)
            .map_err(|e| Error::Other(e.into()))?;
        let mut trajs = trajectory_from_positions(&positions, trajectory[0].time_from_start);

        for i in 1..trajectory.len() {
            let positions = self
                .planner
                .plan_avoid_self_collision(
                    &self.using_joints,
                    &trajectory[i - 1].positions,
                    &trajectory[i].positions,
                )
                .map_err(|e| Error::Other(e.into()))?;
            trajs.append(&mut trajectory_from_positions(
                &positions,
                trajectory[i].time_from_start,
            ));
        }
        self.client.send_joint_trajectory(trajs)
    }
}
