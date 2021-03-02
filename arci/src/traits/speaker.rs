use crate::error::Error;
use async_trait::async_trait;
use auto_impl::auto_impl;

#[async_trait]
#[auto_impl(Box, Arc)]
pub trait Speaker: Send + Sync {
    async fn speak(&self, message: &str) -> Result<(), Error>;
}
