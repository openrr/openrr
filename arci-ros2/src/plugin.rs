use std::{
    cell::RefCell,
    sync::atomic::{AtomicUsize, Ordering},
};

use crate::{Ros2CmdVelMoveBase, Ros2CmdVelMoveBaseConfig};

openrr_plugin::export_plugin!(Ros2Plugin {});

struct Ros2Plugin {}

static COUNTER: AtomicUsize = AtomicUsize::new(0);

thread_local! {
    // r2r::Node is not thread-safe, so create a node per thread.
    static NODE: RefCell<r2r::Node> = {
        let ctx = r2r::Context::create().unwrap();
        // Name a different node name per thread. Note that this is not a unique
        // name. If we need a really unique node name, we may need to include
        // the process id as well.
        // See also https://github.com/ros2/design/issues/187.
        let name = format!("arci_ros2_plugin_node_{}", COUNTER.fetch_add(1, Ordering::Relaxed));
        let node = r2r::Node::create(ctx, &name, "").unwrap();
        RefCell::new(node)
    }
}

impl openrr_plugin::Plugin for Ros2Plugin {
    fn name(&self) -> String {
        "arci-ros2-plugin".into()
    }

    fn new_move_base(&self, args: String) -> Result<Option<Box<dyn arci::MoveBase>>, arci::Error> {
        let config: Ros2CmdVelMoveBaseConfig =
            toml::from_str(&args).map_err(anyhow::Error::from)?;
        Ok(Some(NODE.with(|node| {
            Box::new(Ros2CmdVelMoveBase::new(
                &mut *node.borrow_mut(),
                &config.topic,
            ))
        })))
    }
}
