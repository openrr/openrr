# arci-ros2

[![crates.io](https://img.shields.io/crates/v/arci-ros2.svg)](https://crates.io/crates/arci-ros2) [![docs](https://docs.rs/arci-ros2/badge.svg)](https://docs.rs/arci-ros2)

ROS2 implementation for arci.

## Dependencies

* ROS2 Foxy
* [r2r](https://github.com/sequenceplanner/r2r)

## Install

```bash
sudo apt install ros-foxy-nav2-msgs ros-foxy-geometry-msgs
sudo apt install libclang-dev  # for r2r
```

## Build

Build `openrr` with the feature `ros2`

```bash
source /opt/ros/foxy/setup.bash
cargo build --features ros2
```

## How to use

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
