use crate::{
    Ros2CmdVelMoveBase, Ros2CmdVelMoveBaseConfig, Ros2ControlClient, Ros2ControlConfig,
    Ros2Navigation, Ros2NavigationConfig,
};

openrr_plugin::export_plugin!(Ros2Plugin {});

struct Ros2Plugin {}

impl openrr_plugin::Plugin for Ros2Plugin {
    fn new_joint_trajectory_client(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::JointTrajectoryClient>>, arci::Error> {
        let config: Ros2ControlConfig = toml::from_str(&args).map_err(anyhow::Error::from)?;
        let ctx = r2r::Context::create().unwrap();
        let all_client = Ros2ControlClient::new(ctx, &config.action_name);
        if config.joint_names.is_empty() {
            Ok(Some(Box::new(all_client)))
        } else {
            Ok(Some(Box::new(arci::PartialJointTrajectoryClient::new(
                config.joint_names,
                all_client,
            )?)))
        }
    }

    fn new_move_base(&self, args: String) -> Result<Option<Box<dyn arci::MoveBase>>, arci::Error> {
        let config: Ros2CmdVelMoveBaseConfig =
            toml::from_str(&args).map_err(anyhow::Error::from)?;
        let ctx = r2r::Context::create().unwrap();
        Ok(Some(Box::new(Ros2CmdVelMoveBase::new(ctx, &config.topic))))
    }

    fn new_navigation(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::Navigation>>, arci::Error> {
        let config: Ros2NavigationConfig = toml::from_str(&args).map_err(anyhow::Error::from)?;
        let ctx = r2r::Context::create().unwrap();
        Ok(Some(Box::new(Ros2Navigation::new(
            ctx,
            &config.action_name,
        ))))
    }
}
