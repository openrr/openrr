use arci::gamepad::GamepadEvent;
use auto_impl::auto_impl;

#[auto_impl(Box)]
pub trait ControlNode: Send + Sync {
    fn set_event(&mut self, event: GamepadEvent);
    fn proc(&self);
    fn mode(&self) -> &str;
    fn submode(&self) -> &str;
}
