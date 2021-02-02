use std::time::Duration;

use crate::error::Error;
use crate::traits::JointTrajectoryClient;
use auto_impl::auto_impl;

#[auto_impl(Box, Rc, Arc)]
pub trait CompleteCondition: Send + Sync {
    fn wait(
        &self,
        client: &dyn JointTrajectoryClient,
        target_positions: &[f64],
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

impl CompleteCondition for TotalJointDiffCondition {
    fn wait(
        &self,
        client: &dyn JointTrajectoryClient,
        target_positions: &[f64],
    ) -> Result<(), Error> {
        const CHECK_UNIT_SEC: f64 = 0.01;
        let check_unit_duration: Duration = Duration::from_secs_f64(CHECK_UNIT_SEC);
        let num_repeat: i32 = (self.timeout_sec / CHECK_UNIT_SEC) as i32;
        for _j in 0..num_repeat {
            let curs = client.current_joint_positions()?;
            let sum_err: f64 = target_positions
                .iter()
                .zip(curs.iter())
                .map(|(tar, cur)| (tar - cur).abs())
                .sum();
            if sum_err <= self.allowable_error {
                return Ok(());
            }
            std::thread::sleep(check_unit_duration);
        }
        Err(Error::TimeoutWithDiff {
            target: target_positions.to_vec(),
            current: client.current_joint_positions()?,
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

impl CompleteCondition for EachJointDiffCondition {
    fn wait(
        &self,
        client: &dyn JointTrajectoryClient,
        target_positions: &[f64],
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
        let num_repeat: i32 = (self.timeout_sec / CHECK_UNIT_SEC) as i32;

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
            std::thread::sleep(check_unit_duration);
        }
        Err(Error::TimeoutWithDiff {
            target: target_positions.to_vec(),
            current: client.current_joint_positions()?,
        })
    }
}
