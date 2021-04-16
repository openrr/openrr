# OpenRR applications

[![crates.io](https://img.shields.io/crates/v/openrr-apps.svg)](https://crates.io/crates/openrr-apps) [![docs](https://docs.rs/openrr-apps/badge.svg)](https://docs.rs/openrr-apps)

## Preprare

### Install urdf-viz

```bash
cargo install urdf-viz
```

### Install

```bash
cargo install openrr-apps
```

If you are Windows user, ROS is not supported.
So remove it.

```bash
cargo install openrr-apps --no-default-features --features gui
```

### Option: For UR10 sample

Install [Universal Robot software](https://github.com/ros-industrial/universal_robot).

### Option: For PR2 sample

Install ros-melodic-pr2-gazebo / ros-melodic-topic-tools.

## How to run openrr_apps_robot_command

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf &
```

- Run sample commands.

```bash
openrr_apps_robot_command \
  --config-path=./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml \
  load_commands ./openrr-apps/command/sample_cmd_urdf_viz.txt
```

#### Environmental Variables

If you set `export OPENRR_APPS_ROBOT_CONFIG_PATH=some_path_to_config.toml`, you can skip
`--config-path`. If you give `--config-path` explicitly, the env var is ignored.

- Run sample commands with env var

```bash
export OPENRR_APPS_ROBOT_CONFIG_PATH=$(pwd)/openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml
openrr_apps_robot_command load_commands ./openrr-apps/command/sample_cmd_urdf_viz.txt
```

Do not forget to unset OPENRR_APPS_ROBOT_CONFIG_PATH before try other settings

### UR10 (urdf-viz)

- Launch urdf-viz.

```bash
urdf-viz $(rospack find ur_description)/urdf/ur10_robot.urdf.xacro
```

- Run sample commands.

Change urdf path in [the setting file](./config/ur10_robot_client_config_for_urdf_viz.toml) for your environment.

```bash
openrr_apps_robot_command \
  --config-path=./openrr-apps/config/ur10_robot_client_config_for_urdf_viz.toml \
  load_commands ./openrr-apps/command/ur10_cmd_urdf_viz.txt
```

### UR10 (ROS gazebo)

- Launch gazebo.

```bash
roslaunch ur_gazebo ur10.launch
```

- Run sample commands.

Change urdf path in [the setting file](./config/ur10_robot_client_config_for_ros.toml) for your environment.

```bash
openrr_apps_robot_command \
  --config-path=./openrr-apps/config/ur10_robot_client_config_for_ros.toml \
  load_commands ./openrr-apps/command/ur10_cmd_ros.txt
```

### PR2 (urdf-viz)

- Launch urdf-viz.

```bash
urdf-viz $(rospack find pr2_description)/robots/pr2.urdf.xacro
```

- Run sample commands.

Change urdf path in [the setting file](./config/pr2_robot_client_config_for_urdf_viz.toml) for your environment.

```bash
openrr_apps_robot_command \
  --config-path=./openrr-apps/config/pr2_robot_client_config_for_urdf_viz.toml \
  load_commands ./openrr-apps/command/pr2_cmd_urdf_viz.txt
```

### PR2 (ROS gazebo)

- Launch gazebo.

```bash
cd openrr-apps/launch/
roslaunch ./pr2.launch wait_time_secs:=10
```

- Run sample commands.

Change urdf path in [the setting file](./config/pr2_robot_client_config_for_ros.toml) for your environment.

```bash
openrr_apps_robot_command \
  --config-path=./openrr-apps/config/pr2_robot_client_config_for_ros.toml \
  load_commands ./openrr-apps/command/pr2_cmd_ros.txt
```

## How to run openrr_apps_robot_teleop

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf &
```

- <a id="joystick">Connect joystick</a>.

Change below joystick settings in [the setting file](./config/sample_teleop_config_urdf_viz.toml) for your device.
Default value is for 'Sony DualShock 4'.

```TOML
gil_gamepad_config.device_id = 0
gil_gamepad_config.map.button_map = ...
gil_gamepad_config.map.axis_map = ...
gil_gamepad_config.map.axis_value_map = ...
```

- Run teleop.

```bash
openrr_apps_robot_teleop --config-path=./openrr-apps/config/sample_teleop_config_urdf_viz.toml
```

### UR10 (urdf-viz)

- Launch urdf-viz.

```bash
urdf-viz $(rospack find ur_description)/urdf/ur10_robot.urdf.xacro
```

- Run teleop.

Change urdf path and joystick settings (see [here](#joystick)) in [the setting file](./config/ur10_robot_client_config_for_urdf_viz.toml) for your environment.


```bash
openrr_apps_robot_teleop --config-path=./openrr-apps/config/ur10_teleop_config_urdf_viz.toml
```

### UR10 (ROS gazebo)

- Launch gazebo.

```bash
roslaunch ur_gazebo ur10.launch
```

- Run teleop.

Change urdf path and joystick settings (see [here](#joystick)) in [the setting file](./config/ur10_robot_client_config_for_ros.toml) for your environment.


```bash
openrr_apps_robot_teleop --config-path=./openrr-apps/config/ur10_teleop_config_ros.toml
```

### PR2 (urdf-viz)

- Launch urdf-viz.

```bash
urdf-viz $(rospack find pr2_description)/robots/pr2.urdf.xacro
```

- Run teleop.

Change urdf path and joystick settings (see [here](#joystick)) in [the setting file](./config/pr2_robot_client_config_for_urdf_viz.toml) for your environment.


```bash
openrr_apps_robot_teleop --config-path=./openrr-apps/config/pr2_teleop_config_urdf_viz.toml
```

### PR2 (ROS gazebo)

- Launch gazebo.

```bash
cd openrr-apps/launch/
roslaunch ./pr2.launch wait_time_secs:=10
```

- Run teleop.

Change urdf path and joystick settings (see [here](#joystick)) in [the setting file](./config/pr2_robot_client_config_for_ros.toml) for your environment.


```bash
openrr_apps_robot_teleop --config-path=./openrr-apps/config/pr2_teleop_config_ros.toml
```

## How to run openrr_apps_joint_position_sender

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf &
```

- Launch openrr_apps_joint_position_sender.

```bash
openrr_apps_joint_position_sender \
  --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml
```

### Troubleshooting

See [openrr-gui](../openrr-gui/README.md#troubleshooting) crate for troubleshooting on GUI.

## Environmental Variables

If you set `export OPENRR_APPS_ROBOT_CONFIG_PATH=some_path_to_config.toml`, you can skip
`--config-path`. If you give `--config-path` explicitly, the env var is ignored.

- Run for sample urdf with env var

```bash
export OPENRR_APPS_ROBOT_CONFIG_PATH=$(pwd)/openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml
openrr_apps_joint_position_sender
```

Do not forget to unset OPENRR_APPS_ROBOT_CONFIG_PATH before try other settings
