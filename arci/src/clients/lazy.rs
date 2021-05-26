use std::{
    sync::Arc,
    time::{Duration, SystemTime},
};

use async_trait::async_trait;
use nalgebra::{Isometry2, Isometry3};

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
pub struct Lazy<'a, T> {
    inner: once_cell::sync::Lazy<
        Result<T, Arc<Error>>,
        Box<dyn FnOnce() -> Result<T, Arc<Error>> + Send + Sync + 'a>,
    >,
    joint_names: Option<Vec<String>>,
}

impl<'a, T> Lazy<'a, T> {
    /// Creates a new `Lazy` with the given constructor.
    pub fn new(constructor: impl FnOnce() -> Result<T, Error> + Send + Sync + 'a) -> Self {
        Self {
            inner: once_cell::sync::Lazy::new(Box::new(|| constructor().map_err(Arc::new))),
            joint_names: None,
        }
    }

    /// Creates a new `Lazy` with the given constructor and joint names.
    ///
    /// The specified joint names will be used as the return value of
    /// `JointTrajectoryClient::joint_names`.
    pub fn with_joint_names(
        constructor: impl FnOnce() -> Result<T, Error> + Send + Sync + 'a,
        joint_names: Vec<String>,
    ) -> Self {
        Self {
            inner: once_cell::sync::Lazy::new(Box::new(|| constructor().map_err(Arc::new))),
            joint_names: Some(joint_names),
        }
    }

    /// Returns a reference to the underlying value.
    ///
    /// - If this lazy value has not been constructed yet, this method will construct it.
    /// - If the constructor of this lazy value fails, this method returns an error.
    pub fn get_ref(&self) -> Result<&T, Error> {
        self.inner.as_ref().map_err(|e| Error::Lazy(e.clone()))
    }
}

impl<T> JointTrajectoryClient for Lazy<'_, T>
where
    T: JointTrajectoryClient,
{
    fn joint_names(&self) -> Vec<String> {
        if let Some(joint_names) = &self.joint_names {
            return joint_names.clone();
        }
        match self.get_ref() {
            Ok(this) => this.joint_names(),
            Err(e) => panic!("{}", e),
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
            Err(e) => panic!("{}", e),
        }
    }

    fn stop(&self) {
        match self.get_ref() {
            Ok(this) => this.stop(),
            Err(e) => panic!("{}", e),
        }
    }
}
