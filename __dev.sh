#!/bin/bash

urdf-viz ./openrr-planner/sample.urdf &

cargo run -p openrr-apps --bin openrr_apps_joint_position_sender -- --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml &
cargo run -p openrr-apps --bin openrr_apps_velocity_sender -- --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml &

cargo run -r -p openrr-apps --bin openrr_apps_joint_position_sender -- --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml &
cargo run -r -p openrr-apps --bin openrr_apps_velocity_sender -- --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml &

./target/release/openrr_apps_joint_position_sender --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml

cargo run -p openrr-apps --bin openrr_apps_robot_command -- \
  --config-path=./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml \
  load_commands ./openrr-apps/command/sample_cmd_urdf_viz.txt
