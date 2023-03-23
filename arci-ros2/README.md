# arci-ros2

[![crates.io](https://img.shields.io/crates/v/arci-ros2.svg?logo=rust)](https://crates.io/crates/arci-ros2) [![docs](https://docs.rs/arci-ros2/badge.svg)](https://docs.rs/arci-ros2) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/arci_ros2)

ROS2 implementation for arci.

## Dependencies

* ROS2 [Foxy](https://docs.ros.org/en/foxy/Installation.html) or [Galactic](https://docs.ros.org/en/galactic/Installation.html)
* [r2r](https://github.com/sequenceplanner/r2r)

## Install

```bash
sudo apt install ros-foxy-nav2-msgs ros-foxy-geometry-msgs # for navigation
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers # for ros2_control
sudo apt install libclang-dev # for r2r
```

## Build

Build `openrr` with the feature `ros2`

```bash
source /opt/ros/foxy/setup.bash
cargo build --features ros2
```

## How to use (navigation2)

### Install navigation2

Read [here](https://navigation.ros.org/getting_started/index.html) and install navigation2 and simulator.

### Launch simulator

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

### Run command line tools

#### Send navigation goal via command line

Don't forget to set the initial pose of the robot before sending the goal pose using the `2D Pose Estimate` button of `rviz`.

```bash
./target/debug/openrr_apps_robot_command --config-path ./openrr-apps/config/turtlebot3_robot_client_config_ros2.toml send_navigation_goal -- -0.5 0.2 -1.5
```

#### GUI to send velocity

```bash
./target/debug/openrr_apps_velocity_sender --config-path ./openrr-apps/config/turtlebot3_robot_client_config_ros2.toml
```

## How to use (ros2_control)

### Install ros2_control_demos

Clone and build [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)

* if `ROS2_DISTRO` is foxy: checkout foxy branch, install it, and source `install/setup.bash`.
* if `ROS2_DISTRO` is galactic: replace all "foxy" in the repository with "galactic", install it, and source `install/setup.bash`.

### Launch rrbot example and controllers

Launch rrbot example

```bash
ros2 launch ros2_control_demo_bringup rrbot.launch.py
```

Stop existing controllers.

```bash
ros2 control set_controller_state joint_state_broadcaster stop
ros2 control set_controller_state forward_position_controller stop
```

Start `position_trajectory_controller` and `joint_state_broadcaster`.

```bash
ros2 control load_controller position_trajectory_controller --set-state start
ros2 control set_controller_state joint_state_broadcaster start
```

### Run command line tools

<!--
TODO: add usage of openrr-apps + ros2_control
-->

#### Run ros2_control example of arci-ros2

```sh
cargo run --package arci-ros2 --example ros2_control --features ros2
```

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
