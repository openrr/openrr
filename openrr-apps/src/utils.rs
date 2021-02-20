use crate::RobotConfig;
use log::debug;
use rand::prelude::*;
use std::path::PathBuf;

const OPENRR_APPS_CONFIG_ENV_NAME: &str = "OPENRR_APPS_ROBOT_CONFIG_PATH";

/// Get robot config from input or env OPENRR_APPS_ROBOT_CONFIG_PATH
pub fn get_apps_robot_config(config: Option<PathBuf>) -> Option<PathBuf> {
    if config.is_some() {
        config
    } else {
        std::env::var(OPENRR_APPS_CONFIG_ENV_NAME)
            .map(|s| {
                log::warn!("### ENV VAR {} is used ###", s);
                PathBuf::from(s)
            })
            .ok()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_get_apps_robot_config() {
        let path = get_apps_robot_config(Some(PathBuf::from("a.toml")));
        assert!(path.is_some());
        assert_eq!(path.unwrap(), PathBuf::from("a.toml"));
        //
        std::env::set_var(OPENRR_APPS_CONFIG_ENV_NAME, "b.yaml");
        let path = get_apps_robot_config(Some(PathBuf::from("a.toml")));
        assert!(path.is_some());
        assert_eq!(path.unwrap(), PathBuf::from("a.toml"));
        std::env::remove_var(OPENRR_APPS_CONFIG_ENV_NAME);

        let path = get_apps_robot_config(None);
        assert!(path.is_none());

        std::env::set_var(OPENRR_APPS_CONFIG_ENV_NAME, "b.yaml");
        let path = get_apps_robot_config(None);
        assert!(path.is_some());
        assert_eq!(path.unwrap(), PathBuf::from("b.yaml"));
        std::env::remove_var(OPENRR_APPS_CONFIG_ENV_NAME);
    }
}

/// Do something needed to start the program
pub fn init(name: &str, config: &RobotConfig) {
    #[cfg(feature = "ros")]
    if config.has_ros_clients() {
        arci_ros::init(name);
    }
    debug!("init {} with {:?}", name, config);
}

/// Do something needed to start the program for multiple
pub fn init_with_anonymize(name: &str, config: &RobotConfig) {
    let suffix: u64 = rand::thread_rng().gen();
    let anon_name = format!("{}_{}", name, suffix);
    init(&anon_name, config);
}
