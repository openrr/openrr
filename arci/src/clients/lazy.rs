use std::{
    sync::Arc,
    time::{Duration, SystemTime},
};

use async_trait::async_trait;
use nalgebra::{Isometry2, Isometry3};
use tracing::error;

use crate::{
    error::Error,
    gamepad::GamepadEvent,
    traits::{
        BaseVelocity, Gamepad, JointTrajectoryClient, Localization, MoveBase, Navigation, Speaker,
        TrajectoryPoint, TransformResolver,
    },
    waits::WaitFuture,
};

#[allow(clippy::type_complexity)]
pub struct Lazy<'a, T>(
    once_cell::sync::Lazy<
        Result<T, Arc<Error>>,
        Box<dyn FnOnce() -> Result<T, Arc<Error>> + Send + Sync + 'a>,
    >,
);

impl<'a, T> Lazy<'a, T> {
    pub fn new(f: impl FnOnce() -> Result<T, Error> + Send + Sync + 'a) -> Self {
        Self(once_cell::sync::Lazy::new(Box::new(|| {
            f().map_err(Arc::new)
        })))
    }

    pub fn get_ref(&self) -> Result<&T, Error> {
        self.0.as_ref().map_err(|e| Error::Arc(e.clone()))
    }
}

impl<T> JointTrajectoryClient for Lazy<'_, T>
where
    T: JointTrajectoryClient,
{
    fn joint_names(&self) -> Vec<String> {
        match self.get_ref() {
            Ok(this) => this.joint_names(),
            Err(e) => {
                error!("{}", e);
                // TODO
                vec![]
            }
        }
    }

    fn current_joint_positions(&self) -> Result<Vec<f64>, Error> {
        self.get_ref()?.current_joint_positions()
    }

    fn send_joint_positions(
        &self,
        positions: Vec<f64>,
        duration: Duration,
    ) -> Result<WaitFuture, Error> {
        self.get_ref()?.send_joint_positions(positions, duration)
    }

    fn send_joint_trajectory(&self, trajectory: Vec<TrajectoryPoint>) -> Result<WaitFuture, Error> {
        self.get_ref()?.send_joint_trajectory(trajectory)
    }
}

impl<T> Localization for Lazy<'_, T>
where
    T: Localization,
{
    fn current_pose(&self, frame_id: &str) -> Result<Isometry2<f64>, Error> {
        self.get_ref()?.current_pose(frame_id)
    }
}

impl<T> MoveBase for Lazy<'_, T>
where
    T: MoveBase,
{
    fn current_velocity(&self) -> Result<BaseVelocity, Error> {
        self.get_ref()?.current_velocity()
    }

    fn send_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        self.get_ref()?.send_velocity(velocity)
    }
}

impl<T> Navigation for Lazy<'_, T>
where
    T: Navigation,
{
    fn send_goal_pose(
        &self,
        goal: Isometry2<f64>,
        frame_id: &str,
        timeout: Duration,
    ) -> Result<WaitFuture, Error> {
        self.get_ref()?.send_goal_pose(goal, frame_id, timeout)
    }

    fn cancel(&self) -> Result<(), Error> {
        self.get_ref()?.cancel()
    }
}

impl<T> Speaker for Lazy<'_, T>
where
    T: Speaker,
{
    fn speak(&self, message: &str) -> Result<WaitFuture, Error> {
        self.get_ref()?.speak(message)
    }
}

impl<T> TransformResolver for Lazy<'_, T>
where
    T: TransformResolver,
{
    fn resolve_transformation(
        &self,
        from: &str,
        to: &str,
        time: SystemTime,
    ) -> Result<Isometry3<f64>, Error> {
        self.get_ref()?.resolve_transformation(from, to, time)
    }
}

#[async_trait]
impl<T> Gamepad for Lazy<'_, T>
where
    T: Gamepad,
{
    async fn next_event(&self) -> GamepadEvent {
        match self.get_ref() {
            Ok(this) => this.next_event().await,
            Err(e) => {
                error!("{}", e);
                GamepadEvent::Unknown
            }
        }
    }

    fn stop(&self) {
        match self.get_ref() {
            Ok(this) => this.stop(),
            Err(e) => {
                error!("{}", e);
            }
        }
    }
}
