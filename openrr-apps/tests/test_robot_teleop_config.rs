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
        let result = RobotTeleopConfig::try_new(f);
        assert!(result.is_ok(), "{:?} {:?}", f, result);
    }
}
