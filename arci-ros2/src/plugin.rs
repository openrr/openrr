use std::sync::Mutex;

use once_cell::sync::Lazy;

use crate::{Ros2CmdVelMoveBase, Ros2CmdVelMoveBaseConfig};

openrr_plugin::export_plugin!(Ros2Plugin {});

struct Ros2Plugin {}

static NODE: Lazy<Mutex<r2r::Node>> = Lazy::new(|| {
    let ctx = r2r::Context::create().unwrap();
    // Note that this is not a unique name. If we need a really unique node
    // name, we may need to include the process id as well.
    // See also https://github.com/ros2/design/issues/187.
    let node = r2r::Node::create(ctx, "arci_ros2_plugin_node", "").unwrap();
    Mutex::new(node)
});

impl openrr_plugin::Plugin for Ros2Plugin {
    fn name(&self) -> String {
        "arci-ros2-plugin".into()
    }

    fn new_move_base(&self, args: String) -> Result<Option<Box<dyn arci::MoveBase>>, arci::Error> {
        let config: Ros2CmdVelMoveBaseConfig =
            toml::from_str(&args).map_err(anyhow::Error::from)?;
        let mut node = NODE.lock().unwrap();
        Ok(Some(Box::new(Ros2CmdVelMoveBase::new(
            &mut *node,
            &config.topic,
        ))))
    }
}
