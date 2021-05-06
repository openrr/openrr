//! [`arci`] implementation using [urdf-viz](https://github.com/openrr/urdf-viz).

mod client;
mod utils;

pub use crate::client::*;

#[cfg(target_arch = "wasm32")]
mod wasm {
    use futures::{
        future::{Future, FutureExt},
        task::{noop_waker, Context, Poll},
    };

    // TODO: this is not good way...
    pub(crate) fn run<F: Future>(mut f: F) -> F::Output {
        futures::pin_mut!(f);
        let w = noop_waker();
        let mut cx = Context::from_waker(&w);
        loop {
            if let Poll::Ready(x) = f.poll_unpin(&mut cx) {
                return x;
            }
        }
    }
}