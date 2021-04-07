use openrr_apps::RobotConfig;

#[test]
fn verify_sample_configs() {
    let files = vec![
        "config/pr2_robot_client_config_for_ros.toml",
        "config/pr2_robot_client_config_for_urdf_viz.toml",
        "config/sample_robot_client_config_for_urdf_viz.toml",
        "config/sample_robot_client_config_for_urdf_viz_with_multiple_speaker.toml",
        "config/ur10_robot_client_config_for_ros.toml",
        "config/ur10_robot_client_config_for_urdf_viz.toml",
    ];
    for f in files {
        let result = RobotConfig::try_new(f);
        if cfg!(not(feature = "ros")) && f.contains("ros") {
            assert!(
                matches!(result, Err(openrr_apps::Error::ConfigRequireRos(..))),
                "{:?} {:?}",
                f,
                result
            );
        } else {
            assert!(result.is_ok(), "{:?} {:?}", f, result);
            let ser_result = toml::to_string(&result.unwrap());
            assert!(ser_result.is_ok(), "{:?} {:?}", f, ser_result);
        }
    }
}

#[test]
fn ser_default_config() {
    toml::to_string(&RobotConfig::default()).unwrap();
}
