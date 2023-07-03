#![doc = include_str!("../README.md")]
#![warn(/*missing_docs,*/ rust_2018_idioms)]
// buggy: https://github.com/rust-lang/rust-clippy/issues?q=is%3Aissue+derive_partial_eq_without_eq
#![allow(clippy::derive_partial_eq_without_eq)]

pub mod de;

use std::time::{Duration, SystemTime};

use tracing::trace;

pub struct Tracing<T>(T);

impl<T> Tracing<T> {
    pub fn new(v: T) -> Self {
        Self(v)
    }

    pub fn get_ref(&self) -> &T {
        &self.0
    }

    pub fn get_mut(&mut self) -> &mut T {
        &mut self.0
    }

    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T> From<T> for Tracing<T> {
    fn from(value: T) -> Self {
        Self::new(value)
    }
}

// TODO: generate impls by codegen

impl<T: arci::JointTrajectoryClient> arci::JointTrajectoryClient for Tracing<T> {
    // TODO: deserialize/test
    fn joint_names(&self) -> Vec<String> {
        let names = self.0.joint_names();
        trace!(method = "arci::JointTrajectoryClient::joint_names", ?names);
        names
    }

    // TODO: deserialize/test
    fn current_joint_positions(&self) -> Result<Vec<f64>, arci::Error> {
        let positions = self.0.current_joint_positions()?;
        trace!(
            method = "arci::JointTrajectoryClient::current_joint_positions",
            ?positions
        );
        Ok(positions)
    }

    // TODO: deserialize/test
    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<arci::WaitFuture, arci::Error> {
        trace!(
            method = "arci::JointTrajectoryClient::send_joint_positions",
            ?positions,
            ?duration
        );
        self.0.send_joint_positions(positions, duration)
    }

    // TODO: deserialize/test
    fn send_joint_trajectory(
        &self,
        trajectory: Vec<arci::TrajectoryPoint>,
    ) -> Result<arci::WaitFuture, arci::Error> {
        trace!(
            method = "arci::JointTrajectoryClient::send_joint_trajectory",
            ?trajectory
        );
        self.0.send_joint_trajectory(trajectory)
    }
}

#[arci::async_trait]
impl<T: arci::Gamepad> arci::Gamepad for Tracing<T> {
    // TODO: deserialize/test
    async fn next_event(&self) -> arci::gamepad::GamepadEvent {
        let event = self.0.next_event().await;
        trace!(method = "arci::Gamepad::next_event", ?event);
        event
    }

    // TODO: deserialize/test
    fn stop(&self) {
        trace!(method = "arci::Gamepad::stop");
        self.0.stop()
    }
}

impl<T: arci::Localization> arci::Localization for Tracing<T> {
    fn current_pose(&self, frame_id: &str) -> Result<arci::Isometry2<f64>, arci::Error> {
        let pose = self.0.current_pose(frame_id)?;
        trace!(
            method = "arci::Localization::current_pose",
            frame_id = frame_id,
            pose_rotation_re = pose.rotation.re,
            pose_rotation_im = pose.rotation.im,
            pose_translation_x = pose.translation.x,
            pose_translation_y = pose.translation.y,
        );
        Ok(pose)
    }
}

// TODO: test set_motor_effort and get_motor_effort
impl<T: arci::MotorDriveEffort> arci::MotorDriveEffort for Tracing<T> {
    fn set_motor_effort(&self, effort: f64) -> Result<(), arci::Error> {
        trace!(method = "arci::MotorDriveEffort::set_motor_effort", ?effort);
        self.0.set_motor_effort(effort)
    }

    fn get_motor_effort(&self) -> Result<f64, arci::Error> {
        let effort = self.0.get_motor_effort()?;
        trace!(method = "arci::MotorDriveEffort::get_motor_effort", ?effort);
        Ok(effort)
    }
}

// TODO: test set_motor_position and get_motor_position
impl<T: arci::MotorDrivePosition> arci::MotorDrivePosition for Tracing<T> {
    fn set_motor_position(&self, position: f64) -> Result<(), arci::Error> {
        trace!(
            method = "arci::MotorDrivePosition::set_motor_position",
            ?position
        );
        self.0.set_motor_position(position)
    }

    fn get_motor_position(&self) -> Result<f64, arci::Error> {
        let position = self.0.get_motor_position()?;
        trace!(
            method = "arci::MotorDrivePosition::get_motor_position",
            ?position
        );
        Ok(position)
    }
}

// TODO: test set_motor_velocity and get_motor_velocity
impl<T: arci::MotorDriveVelocity> arci::MotorDriveVelocity for Tracing<T> {
    fn set_motor_velocity(&self, velocity: f64) -> Result<(), arci::Error> {
        trace!(
            method = "arci::MotorDriveVelocity::set_motor_velocity",
            ?velocity
        );
        self.0.set_motor_velocity(velocity)
    }

    fn get_motor_velocity(&self) -> Result<f64, arci::Error> {
        let velocity = self.0.get_motor_velocity()?;
        trace!(
            method = "arci::MotorDriveVelocity::get_motor_velocity",
            ?velocity
        );
        Ok(velocity)
    }
}

// TODO: test current_velocity
impl<T: arci::MoveBase> arci::MoveBase for Tracing<T> {
    fn send_velocity(&self, velocity: &arci::BaseVelocity) -> Result<(), arci::Error> {
        trace!(
            method = "arci::MoveBase::send_velocity",
            velocity_x = velocity.x,
            velocity_y = velocity.y,
            velocity_theta = velocity.theta
        );
        self.0.send_velocity(velocity)
    }

    fn current_velocity(&self) -> Result<arci::BaseVelocity, arci::Error> {
        let velocity = self.0.current_velocity()?;
        trace!(
            method = "arci::MoveBase::current_velocity",
            velocity_x = velocity.x,
            velocity_y = velocity.y,
            velocity_theta = velocity.theta
        );
        Ok(velocity)
    }
}

// TODO: test send_goal_pose and cancel
impl<T: arci::Navigation> arci::Navigation for Tracing<T> {
    fn send_goal_pose(
        &self,
        goal: arci::Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<arci::WaitFuture, arci::Error> {
        trace!(
            method = "arci::Navigation::send_goal_pose",
            goal_rotation_re = goal.rotation.re,
            goal_rotation_im = goal.rotation.im,
            goal_translation_x = goal.translation.x,
            goal_translation_y = goal.translation.y,
            frame_id,
            timeout_secs = timeout.as_secs(),
            timeout_nanos = timeout.subsec_nanos(),
        );
        self.0.send_goal_pose(goal, frame_id, timeout)
    }

    fn cancel(&self) -> Result<(), arci::Error> {
        trace!(method = "arci::Navigation::cancel");
        self.0.cancel()
    }
}

// TODO: test speak
impl<T: arci::Speaker> arci::Speaker for Tracing<T> {
    fn speak(&self, message: &str) -> Result<arci::WaitFuture, arci::Error> {
        trace!(method = "arci::Speaker::speak", ?message);
        self.0.speak(message)
    }
}

// TODO: test resolve_transformation
impl<T: arci::TransformResolver> arci::TransformResolver for Tracing<T> {
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: SystemTime,
    ) -> Result<arci::Isometry3<f64>, arci::Error> {
        let d = time.duration_since(SystemTime::UNIX_EPOCH).unwrap();
        trace!(
            method = "arci::TransformResolver::resolve_transformation",
            ?from,
            ?to,
            time_secs = d.as_secs(),
            time_nanos = d.subsec_nanos(),
        );
        self.0.resolve_transformation(from, to, time)
    }
}
