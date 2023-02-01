/*
Example of velocity_sender with dummy MoveBase.

```bash
cargo run -p openrr-gui --example velocity_sender
```
*/

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            std::env::var("RUST_LOG").unwrap_or_else(|_| "openrr_gui=debug".to_owned()),
        )
        .init();

    openrr_gui::velocity_sender(arci::DummyMoveBase::new());
}
