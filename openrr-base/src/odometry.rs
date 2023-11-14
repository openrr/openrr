use std::sync::Mutex;

use arci::{BaseVelocity, Isometry2, Vector2};

use crate::Error;

const DEFAULT_TIMEOUT_MILLIS: u128 = 5000;

#[derive(Debug)]
pub struct Odometry {
    /// If `None`, it is odometry lost.
    position: Mutex<Option<Isometry2<f64>>>,
    last_update_timestamp: Mutex<Option<std::time::Instant>>,
    timeout_millis: Mutex<u128>,
}

impl Odometry {
    pub fn new_from_pose(position: Isometry2<f64>) -> Self {
        Self {
            position: Mutex::new(Some(position)),
            last_update_timestamp: Mutex::new(None),
            /// If `0`, timeout is invalid
            timeout_millis: Mutex::new(DEFAULT_TIMEOUT_MILLIS),
        }
    }

    pub fn update_by_velocity(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let mut position = self.position.lock().unwrap();
        let mut locked_update_time = self.last_update_timestamp.lock().unwrap();
        let timeout_millis = *self.timeout_millis.lock().unwrap();

        let last_update_time = match *locked_update_time {
            Some(t) => t,
            None => std::time::Instant::now(),
        };

        match *position {
            Some(p) => {
                if last_update_time.elapsed().as_millis() > timeout_millis && timeout_millis != 0 {
                    *position = None;
                    *locked_update_time = Some(std::time::Instant::now());
                    Err(Error::CurrentPositionUnknown)
                } else {
                    let dt = last_update_time.elapsed().as_secs_f64();
                    let dx = Isometry2::new(
                        Vector2::new(velocity.x * dt, velocity.y * dt),
                        velocity.theta * dt,
                    );

                    *position = Some(p * dx);
                    *locked_update_time = Some(std::time::Instant::now());

                    Ok(())
                }
            }
            None => Err(Error::CurrentPositionUnknown),
        }
    }

    /// Recover or update odometry
    pub fn resolve_lost(&self, current_pose: Isometry2<f64>) {
        let mut pose = self.position.lock().unwrap();
        let mut update_time = self.last_update_timestamp.lock().unwrap();

        *pose = Some(current_pose);
        *update_time = Some(std::time::Instant::now());
    }

    pub fn set_timeout_millis(&self, millis: u128) {
        let mut timeout_millis = self.timeout_millis.lock().unwrap();
        *timeout_millis = millis;
    }

    pub fn current_pose(&self) -> Result<Isometry2<f64>, Error> {
        match self.position.lock().unwrap().to_owned() {
            Some(pose) => Ok(pose),
            None => Err(Error::CurrentPositionUnknown),
        }
    }
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            position: Mutex::new(None),
            last_update_timestamp: Mutex::new(None),
            timeout_millis: Mutex::new(DEFAULT_TIMEOUT_MILLIS),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_base_odometry() {
        let vel = BaseVelocity::new(100.0, 200.0, 100.0 * std::f64::consts::FRAC_PI_2);
        let odom = Odometry::new_from_pose(Isometry2::default());

        std::thread::sleep(std::time::Duration::from_millis(10));

        odom.update_by_velocity(&vel).unwrap();
        let pose = odom.current_pose();

        assert!(
            pose.as_ref().unwrap().translation.x > 1.00
                || pose.as_ref().unwrap().translation.x < 1.05
        );
        assert!(
            pose.as_ref().unwrap().translation.y > 2.00
                || pose.as_ref().unwrap().translation.y < 2.10
        );
    }
}
