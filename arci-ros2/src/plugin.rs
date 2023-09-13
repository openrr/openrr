use crate::{
    Node, Ros2CmdVelMoveBase, Ros2CmdVelMoveBaseConfig, Ros2ControlClient, Ros2ControlConfig,
    Ros2LaserScan2D, Ros2LaserScan2DConfig, Ros2LocalizationClient, Ros2LocalizationClientConfig,
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
        let node = Node::new("plugin_ros2_control_node", "arci_ros2").unwrap();
        let all_client = Ros2ControlClient::new(node, &config.action_name);
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
        let node = Node::new("plugin_cmd_vel_node", "arci_ros2").unwrap();
        Ok(Some(Box::new(Ros2CmdVelMoveBase::new(node, &config.topic))))
    }

    fn new_navigation(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::Navigation>>, arci::Error> {
        let config: Ros2NavigationConfig = toml::from_str(&args).map_err(anyhow::Error::from)?;
        let node = Node::new("plugin_nav2_node", "arci_ros2").unwrap();
        Ok(Some(Box::new(Ros2Navigation::new(
            node,
            &config.action_name,
        ))))
    }

    fn new_localization(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::Localization>>, arci::Error> {
        let config: Ros2LocalizationClientConfig =
            toml::from_str(&args).map_err(anyhow::Error::from)?;
        let ctx = r2r::Context::create().unwrap();
        Ok(Some(Box::new(Ros2LocalizationClient::new(
            ctx,
            config.request_final_nomotion_update_hack,
            &config.nomotion_update_service_name,
            &config.amcl_pose_topic_name,
        ))))
    }

    fn new_laser_scan2_d(
        &self,
        args: String,
    ) -> Result<Option<Box<dyn arci::LaserScan2D>>, arci::Error> {
        let config: Ros2LaserScan2DConfig = toml::from_str(&args).map_err(anyhow::Error::from)?;
        let node = Node::new("plugin_ros2_laser_scan_node", "arci_ros2").unwrap();
        Ok(Some(Box::new(Ros2LaserScan2D::new(node, &config.topic))))
    }
}
