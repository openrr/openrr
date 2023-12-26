# arci-ros2

[![crates.io](https://img.shields.io/crates/v/arci-ros2.svg?logo=rust)](https://crates.io/crates/arci-ros2) [![docs](https://docs.rs/arci-ros2/badge.svg)](https://docs.rs/arci-ros2) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/arci_ros2)

ROS2 implementation for arci.

## Dependencies

* ROS2 [Humble](https://docs.ros.org/en/humble/Installation.html)

## Install

```bash
sudo apt install ros-humble-nav2-msgs ros-humble-geometry-msgs # for navigation
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers # for ros2_control
sudo apt install ros-humble-tf2-msgs # for tf2
```

## Build

Build `openrr`

```bash
source /opt/ros/humble/setup.bash
cargo build
```

## How to use (navigation2)

### Install navigation2

Read [here](https://navigation.ros.org/getting_started/index.html) and install navigation2 and simulator.

### Launch simulator

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

### Run command line tools

#### Run navigation example of arci-ros2

```sh
cargo run --package arci-ros2 --example navigation -- -- 0.6 0.2 1.0
```

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

* Clone [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)
* Checkout [the corresponding branch](https://github.com/ros-controls/ros2_control_demos/blob/HEAD/README.md#build-status) and install it
* Source `install/setup.bash`.

### Launch rrbot example and controllers

Launch rrbot example

```bash
ros2 launch ros2_control_demo_example_1 rrbot.launch.py
```

Then, inactive `forward_position_controller` and active `joint_trajectory_position_controller`.

```bash
ros2 control set_controller_state forward_position_controller inactive
ros2 control load_controller joint_trajectory_position_controller --set-state active
```

`joint_state_broadcaster` and `joint_trajectory_position_controller` should now be activated, and `forward_position_controller` should now be inactivated.

```console
$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] inactive
joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### Run command line tools

<!--
TODO: add usage of openrr-apps + ros2_control
-->

#### Run ros2_control example of arci-ros2

```sh
cargo run --package arci-ros2 --example ros2_control
```

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
