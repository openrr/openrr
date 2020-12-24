use auto_impl::auto_impl;

#[auto_impl(Box, Arc)]
pub trait Speaker: Send + Sync {
    fn speak(&self, message: &str);
}
