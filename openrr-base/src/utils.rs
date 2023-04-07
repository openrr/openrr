use arci::BaseVelocity;

#[derive(Debug, Clone, Copy)]
pub struct BaseVelocityTimestamped {
    pub base_velocity: BaseVelocity,
    pub timestamp: std::time::Instant,
}

pub type BaseAcceleration = BaseVelocity;

pub trait Limiter {
    fn velocity_limiter(&self, velocity: &BaseVelocity) -> BaseVelocity;
    fn acceleration_limiter(
        &self,
        current_velocity: &BaseVelocity,
        last_velocity: &BaseVelocity,
        delta: f64,
    ) -> BaseVelocity;
}

pub trait VelocityTransformer {
    fn transform_velocity_base_to_wheel(&self, velocity: &BaseVelocity) -> Vec<f64>;
    fn transform_velocity_wheel_to_base(&self, wheels_vel: &[f64]) -> BaseVelocity;
}
