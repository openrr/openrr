use arci::{BaseVelocity, Isometry2, Vector2};
use parking_lot::Mutex;

use crate::Error;

const ODOMETRY_TIMEOUT_MILLIS: u128 = 5000;

#[derive(Debug)]
pub struct Odometry {
    /// If `None`, it is odometry lost.
    position: Mutex<Option<Isometry2<f64>>>,
    last_update_timestamp: Mutex<Option<std::time::Instant>>,
}

impl Odometry {
    pub fn new_from_position_with_now(position: Isometry2<f64>) -> Self {
        Self {
            position: Mutex::new(Some(position)),
            last_update_timestamp: Mutex::new(None),
        }
    }

    pub fn translate(&self, velocity: &BaseVelocity) -> Result<(), Error> {
        let mut position = self.position.lock();
        let mut locked_update_time = self.last_update_timestamp.lock();

        let last_update_time = match *locked_update_time {
            Some(t) => t,
            None => std::time::Instant::now(),
        };

        match *position {
            Some(p) => {
                if last_update_time.elapsed().as_millis() > ODOMETRY_TIMEOUT_MILLIS {
                    *position = None;
                    *locked_update_time = Some(std::time::Instant::now());
                    Err(Error::CurrentPositionUnknown)
                } else {
                    let x = p.translation.x;
                    let y = p.translation.y;
                    let theta = p.rotation.angle();
                    let dt = last_update_time.elapsed().as_secs_f64();

                    *position = Some(Isometry2::new(
                        Vector2::new(
                            x + (velocity.x * theta.cos() - velocity.y * theta.sin()) * dt,
                            y + (velocity.x * theta.sin() + velocity.y * theta.cos()) * dt,
                        ),
                        theta + velocity.theta * dt,
                    ));
                    *locked_update_time = Some(std::time::Instant::now());

                    Ok(())
                }
            }
            None => Err(Error::CurrentPositionUnknown),
        }
    }

    /// Recover odometry from different location estimation information.
    pub fn resolve_lost(&self, current_pose: Isometry2<f64>) {
        let mut pose = self.position.lock();
        let mut update_time = self.last_update_timestamp.lock();

        *pose = Some(current_pose);
        *update_time = Some(std::time::Instant::now());
    }

    pub fn current_pose(&self) -> Result<Isometry2<f64>, Error> {
        match self.position.lock().to_owned() {
            Some(pose) => Ok(pose),
            None => Err(Error::CurrentPositionUnknown),
        }
    }
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            position: Default::default(),
            last_update_timestamp: Mutex::new(None),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_base_odometry() {
        let vel = BaseVelocity::new(100.0, 200.0, 100.0 * std::f64::consts::FRAC_PI_2);
        let odom = Odometry::new_from_position_with_now(Isometry2::default());

        std::thread::sleep(std::time::Duration::from_millis(10));

        odom.translate(&vel).unwrap();
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
