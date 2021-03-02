use arci::gamepad::GamepadEvent;
use async_trait::async_trait;
use auto_impl::auto_impl;

#[async_trait]
#[auto_impl(Box)]
pub trait ControlNode: Send + Sync {
    async fn set_event(&mut self, event: GamepadEvent);
    async fn proc(&self);
    fn mode(&self) -> &str;
    fn submode(&self) -> &str;
}
