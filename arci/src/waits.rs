use std::{
    pin::Pin,
    task::{Context, Poll},
    time::Duration,
};

use async_trait::async_trait;
use auto_impl::auto_impl;
use futures::{
    future::{self, BoxFuture, Future, FutureExt},
    stream::{Stream, TryStreamExt},
};

use crate::{error::Error, traits::JointTrajectoryClient};

/// Waits until the underlying future is complete.
#[must_use = "You must explicitly choose whether to wait for the complete or do not wait"]
pub struct WaitFuture<'a> {
    future: BoxFuture<'a, Result<(), Error>>,
}

impl<'a> WaitFuture<'a> {
    /// Waits until the `future` is complete.
    pub fn new(future: impl Future<Output = Result<(), Error>> + Send + 'a) -> Self {
        Self {
            future: future.boxed(),
        }
    }

    /// Waits until the `stream` is complete.
    ///
    /// # Example
    ///
    /// ```
    /// # #[tokio::main]
    /// # async fn main() -> Result<(), arci::Error> {
    /// use arci::WaitFuture;
    /// use futures::stream::FuturesOrdered;
    ///
    /// let mut waits = FuturesOrdered::new();
    /// waits.push(WaitFuture::ready());
    /// waits.push(WaitFuture::ready());
    /// WaitFuture::from_stream(waits).await?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_stream(stream: impl Stream<Item = Result<(), Error>> + Send + 'a) -> Self {
        Self::new(async move {
            futures::pin_mut!(stream);
            while stream.try_next().await?.is_some() {}
            Ok(())
        })
    }

    /// Creates a new `WaitFuture` which immediately complete.
    pub fn ready() -> Self {
        Self::new(future::ready(Ok(())))
    }
}

impl Future for WaitFuture<'_> {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.future.as_mut().poll(cx)
    }
}

impl<'a> From<BoxFuture<'a, Result<(), Error>>> for WaitFuture<'a> {
    fn from(future: BoxFuture<'a, Result<(), Error>>) -> Self {
        Self { future }
    }
}

#[async_trait]
#[auto_impl(Box, Arc)]
pub trait CompleteCondition: Send + Sync {
    async fn wait(
        &self,
        client: &dyn JointTrajectoryClient,
        target_positions: &[f64],
        duration_sec: f64,
    ) -> Result<(), Error>;
}

#[derive(Clone, Debug)]
pub struct TotalJointDiffCondition {
    pub allowable_error: f64,
    pub timeout_sec: f64,
}

impl TotalJointDiffCondition {
    pub fn new(allowable_error: f64, timeout_sec: f64) -> Self {
        Self {
            allowable_error,
            timeout_sec,
        }
    }
}

impl Default for TotalJointDiffCondition {
    fn default() -> Self {
        Self::new(0.02, 10.0)
    }
}

#[async_trait]
impl CompleteCondition for TotalJointDiffCondition {
    async fn wait(
        &self,
        client: &dyn JointTrajectoryClient,
        target_positions: &[f64],
        duration_sec: f64,
    ) -> Result<(), Error> {
        let i = std::time::Instant::now();
        const CHECK_UNIT_SEC: f64 = 0.01;
        let check_unit_duration: Duration = Duration::from_secs_f64(CHECK_UNIT_SEC);
        let num_repeat: i32 = ((self.timeout_sec + duration_sec) / CHECK_UNIT_SEC) as i32;
        for _j in 0..num_repeat {
            let curs = client.current_joint_positions()?;
            let sum_err: f64 = target_positions
                .iter()
                .zip(curs.iter())
                .map(|(tar, cur)| (tar - cur).abs())
                .sum();
            if sum_err <= self.allowable_error {
                i.elapsed();
                return Ok(());
            }
            tokio::time::sleep(check_unit_duration).await;
        }
        Err(Error::TimeoutWithDiff {
            target: target_positions.to_vec(),
            current: client.current_joint_positions()?,
            is_reached: vec![false],
        })
    }
}

#[derive(Clone, Debug)]
pub struct EachJointDiffCondition {
    pub allowable_errors: Vec<f64>,
    pub timeout_sec: f64,
}

impl EachJointDiffCondition {
    pub fn new(allowable_errors: Vec<f64>, timeout_sec: f64) -> Self {
        Self {
            allowable_errors,
            timeout_sec,
        }
    }
}

#[async_trait]
impl CompleteCondition for EachJointDiffCondition {
    async fn wait(
        &self,
        client: &dyn JointTrajectoryClient,
        target_positions: &[f64],
        duration_sec: f64,
    ) -> Result<(), Error> {
        if target_positions.len() != self.allowable_errors.len() {
            eprintln!("wait_until_each_error_condition condition size mismatch");
            return Err(Error::LengthMismatch {
                model: target_positions.len(),
                input: self.allowable_errors.len(),
            });
        }
        let dof = target_positions.len();
        let mut is_reached = vec![false; dof];
        const CHECK_UNIT_SEC: f64 = 0.01;
        let check_unit_duration: Duration = Duration::from_secs_f64(CHECK_UNIT_SEC);
        let num_repeat: i32 = ((self.timeout_sec + duration_sec) / CHECK_UNIT_SEC) as i32;

        for _j in 0..num_repeat {
            for i in 0..dof {
                let cur = client.current_joint_positions()?[i];
                let tar = target_positions[i];
                if !is_reached[i] {
                    is_reached[i] = (tar - cur).abs() < self.allowable_errors[i];
                }
            }
            if !is_reached.contains(&false) {
                return Ok(());
            }
            tokio::time::sleep(check_unit_duration).await;
        }
        Err(Error::TimeoutWithDiff {
            target: target_positions.to_vec(),
            current: client.current_joint_positions()?,
            is_reached,
        })
    }
}
