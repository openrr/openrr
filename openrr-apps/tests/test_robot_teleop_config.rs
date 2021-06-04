use openrr_apps::RobotTeleopConfig;

#[test]
fn verify_sample_configs() {
    let files = vec![
        "config/pr2_teleop_config_ros.toml",
        "config/pr2_teleop_config_urdf_viz.toml",
        "config/sample_teleop_config_urdf_viz.toml",
        "config/ur10_teleop_config_ros.toml",
        "config/ur10_teleop_config_urdf_viz.toml",
    ];

    for f in files {
        let result = RobotTeleopConfig::new(f);
        assert!(result.is_ok(), "{:?} {:?}", f, result);
        let ser_result = toml::to_string(&result.unwrap());
        assert!(ser_result.is_ok(), "{:?} {:?}", f, ser_result);
    }
}

#[test]
fn ser_default_config() {
    toml::to_string(&RobotTeleopConfig::default()).unwrap();
}
