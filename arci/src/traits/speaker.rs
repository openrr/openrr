use auto_impl::auto_impl;

#[auto_impl(Box, Rc, Arc)]
pub trait Speaker {
    fn speak(&self, message: &str);
}
