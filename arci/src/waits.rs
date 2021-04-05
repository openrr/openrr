use std::time::Duration;

use crate::error::Error;
use crate::traits::JointTrajectoryClient;
use async_trait::async_trait;
use auto_impl::auto_impl;

#[async_trait]
#[auto_impl(Box, Rc, Arc)]
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
