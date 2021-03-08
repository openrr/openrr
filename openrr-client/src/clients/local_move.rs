use arci::{BaseVelocity, MoveBase, Navigation};
use k::nalgebra as na;
use na::Isometry2;
use serde::{Deserialize, Serialize};

use crate::Error;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct LocalMoveConfig {
    pub reach_distance_threshold: f64,
    pub reach_angle_threshold: f64,
    pub control_frequency: f64,
    pub linear_gain: f64,
    pub angular_gain: f64,
    pub max_linear_vel: f64,
    pub max_angular_vel: f64,
}

impl LocalMoveConfig {
    pub fn try_new<P: AsRef<std::path::Path>>(path: P) -> Result<Self, Error> {
        let config: LocalMoveConfig = toml::from_str(
            &std::fs::read_to_string(&path)
                .map_err(|e| Error::NoFile(path.as_ref().to_owned(), e))?,
        )
        .map_err(|e| Error::TomlParseFailure(path.as_ref().to_owned(), e))?;
        Ok(config)
    }
}

pub struct LocalMove<N, M>
where
    N: Navigation,
    M: MoveBase,
{
    pub nav_client: N,
    pub vel_client: M,
    pub config: LocalMoveConfig,
}

impl<N, M> LocalMove<N, M>
where
    N: Navigation,
    M: MoveBase,
{
    pub fn new(nav_client: N, vel_client: M, config: LocalMoveConfig) -> Self {
        Self {
            nav_client,
            vel_client,
            config,
        }
    }

    pub fn is_reached(&self, pose_error: Isometry2<f64>) -> bool {
        pose_error.translation.vector.norm() < self.config.reach_distance_threshold
            && pose_error.rotation.angle().abs() < self.config.reach_angle_threshold
    }

    pub fn send_zero_velocity(&self) -> Result<(), arci::Error> {
        self.vel_client
            .send_velocity(&BaseVelocity::new(0.0, 0.0, 0.0))
    }

    pub fn send_control_velocity_from_pose_error(
        &self,
        pose_error: Isometry2<f64>,
    ) -> Result<(), arci::Error> {
        // Calculate control velocity
        // NOTE: pose_error = pose_reference - pose_current
        let vel_x = na::clamp(
            self.config.linear_gain * pose_error.translation.vector[0],
            -self.config.max_linear_vel,
            self.config.max_linear_vel,
        );
        let vel_y = na::clamp(
            self.config.linear_gain * pose_error.translation.vector[1],
            -self.config.max_linear_vel,
            self.config.max_linear_vel,
        );
        let vel_theta = na::clamp(
            self.config.angular_gain * pose_error.rotation.angle(),
            -self.config.max_angular_vel,
            self.config.max_angular_vel,
        );

        self.vel_client
            .send_velocity(&BaseVelocity::new(vel_x, vel_y, vel_theta))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arci::{DummyMoveBase, DummyNavigation};
    use na::Vector2;
    #[test]
    fn test_config() {
        let path = std::path::Path::new("tests/local_move_sample.toml");
        let config = LocalMoveConfig::try_new(path).unwrap();
        assert_eq!(config.reach_distance_threshold, 0.1);
        assert_eq!(config.control_frequency, 10.0);
        assert_eq!(config.linear_gain, 1.0);
        assert_eq!(config.max_linear_vel, 1.0);
    }

    #[test]
    fn test() {
        let path = std::path::Path::new("tests/local_move_sample.toml");
        let config = LocalMoveConfig::try_new(path).unwrap();
        assert_eq!(config.linear_gain, 1.0);
        let move_base = DummyMoveBase::new();
        let nav = DummyNavigation::new();
        let local_move = LocalMove::new(Box::new(nav), Box::new(move_base), config);

        // Check convergence evaluations
        assert_eq!(
            local_move.is_reached(Isometry2::new(Vector2::new(0.0, 0.0), 0.0)),
            true
        );
        assert_eq!(
            local_move.is_reached(Isometry2::new(Vector2::new(0.05, 0.05), 0.05)),
            true
        );
        assert_eq!(
            local_move.is_reached(Isometry2::new(Vector2::new(0.11, 0.0), 0.0)),
            false
        ); // reach_distance_threshold = 0.1
        assert_eq!(
            local_move.is_reached(Isometry2::new(Vector2::new(0.0, 0.0), 0.11)),
            false
        ); // reach_angle_threshold = 0.1

        // Set control velocity
        local_move
            .send_control_velocity_from_pose_error(Isometry2::new(Vector2::new(1.0, -1.0), 1.0))
            .unwrap();
        let vel0 = local_move.vel_client.current_velocity().unwrap();
        assert_eq!(vel0.x, 1.0); // linear_gain = 1.0
        assert_eq!(vel0.y, -1.0); // linear_gain = 1.0
        assert_eq!(vel0.theta, 1.0); // angular_gain = 1.0

        local_move
            .send_control_velocity_from_pose_error(Isometry2::new(Vector2::new(2.0, -2.0), 2.0))
            .unwrap(); // saturated case
        let vel1 = local_move.vel_client.current_velocity().unwrap();
        assert_eq!(vel1.x, 1.0); // max_linear_vel = 1.0
        assert_eq!(vel1.y, -1.0); // max_linear_vel = 1.0
        assert_eq!(vel1.theta, 1.0); // max_angular_vel = 1.0

        // Set zero velocity
        local_move.send_zero_velocity().unwrap();
        let vel2 = local_move.vel_client.current_velocity().unwrap();
        assert_eq!(vel2.x, 0.0);
        assert_eq!(vel2.y, 0.0);
        assert_eq!(vel2.theta, 0.0);
    }
}
