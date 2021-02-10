use log::debug;
use openrr_apps::RobotConfig;
use openrr_client::BoxRobotClient;
use openrr_gui::joint_position_sender;
use structopt::StructOpt;

/// An openrr GUI tool.
#[derive(StructOpt, Debug)]
#[structopt(name = env!("CARGO_BIN_NAME"))]
struct Opt {
    /// Path to the setting file.
    config_path: String,
}

fn main() -> anyhow::Result<()> {
    env_logger::init();
    let opt = Opt::from_args();
    debug!("opt: {:?}", opt);
    let config = RobotConfig::try_new(&opt.config_path)?;
    #[cfg(feature = "ros")]
    if config.has_ros_clients() {
        arci_ros::init("openrr_apps_joint_position_sender");
    }
    let client: BoxRobotClient = config.create_robot_client()?;
    joint_position_sender(
        client,
        config
            .openrr_clients_config
            .urdf_full_path()
            .as_ref()
            .unwrap(),
    )?;
    Ok(())
}
