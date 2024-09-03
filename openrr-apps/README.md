# OpenRR applications

[![crates.io](https://img.shields.io/crates/v/openrr-apps.svg?logo=rust)](https://crates.io/crates/openrr-apps) [![docs](https://docs.rs/openrr-apps/badge.svg)](https://docs.rs/openrr-apps) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/openrr_apps)

## Prepare

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

### Option: Install bash completion for openrr_apps_robot_command

If you are using `bash`,

```bash
openrr_apps_robot_command shell_completion bash > ~/.openrr_command
source ~/.openrr_command
```

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

If you use gamepad, refer to [README of openrr-teleop](../openrr-teleop/).

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

### iRobot Create

[What is `iRobot Create`...](https://iroboteducation.github.io/create3_docs/)

- Build again to feature ROS2.

```bash
cargo build --release --feature ros2
```

- Run teleop.

```bash
openrr_apps_robot_teleop \
  --config-path ./openrr-apps/config/irobot_create_teleop_config_ros2.toml
```

```bash
PrintSpeaker: base
PrintSpeaker: command dock command
PrintSpeaker: command undock command
```

When the `dock command` is executed, `iRobot Create` docks to the station; when the `undock command` is executed, the robot undocks. These are based ROS2 action and follow [official description](https://iroboteducation.github.io/create3_docs/api/docking/).

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

## How to run openrr_apps_velocity_sender

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf &
```

- Launch openrr_apps_velocity_sender.

```bash
openrr_apps_velocity_sender \
  --config-path ./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml
```

### iRobot Create

- Build again to feature ROS2.

```bash
cargo build --release --feature ros2
```

- Launch openrr_apps_velocity_sender.

```bash
openrr_apps_velocity_sender \
  --config-path ./openrr-apps/config/irobot_create_robot_client_config_ros2.toml
```

## Environmental Variables

If you set `export OPENRR_APPS_ROBOT_CONFIG_PATH=some_path_to_config.toml`, you can skip
`--config-path`. If you give `--config-path` explicitly, the env var is ignored.

- Run for sample urdf with env var

```bash
export OPENRR_APPS_ROBOT_CONFIG_PATH=$(pwd)/openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml
openrr_apps_joint_position_sender
```

Do not forget to unset OPENRR_APPS_ROBOT_CONFIG_PATH before try other settings

## Overwrite configuration at startup

By using `--config` flag, you can overwrite the configuration at startup.

For example, to replace the urdf path:

```bash
openrr_apps_robot_command \
  --config-path=./openrr-apps/config/sample_robot_client_config_for_urdf_viz.toml \
  --config='openrr_clients_config.urdf_path="path/to/urdf"' \
  load_commands ./openrr-apps/command/sample_cmd_urdf_viz.txt
```

In `openrr_apps_robot_teleop`, there are two flags: `--robot-config` to overwrite robot config and `--teleop-config` to overwrite teleop config.

For example, to run `openrr_apps_robot_teleop` with `arci-gamepad-keyboard`:

```bash
openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/sample_teleop_config_urdf_viz.toml \
  --teleop-config='gamepad="Keyboard"'
```

To disable joint_position_limiter:

```bash
openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/sample_teleop_config_urdf_viz.toml \
  --robot-config='urdf_viz_clients_configs[0].wrap_with_joint_position_limiter=false'
```

To overwrite multiple configs, separate the scripts with a semicolon or a newline. For example:

```bash
# semicolon-separated
openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/sample_teleop_config_urdf_viz.toml \
  --robot-config='urdf_viz_clients_configs[0].wrap_with_joint_position_limiter=false;openrr_clients_config.urdf_path="path/to/urdf"'

# newline-separated
{
  echo 'urdf_viz_clients_configs[0].wrap_with_joint_position_limiter=false'
  echo 'openrr_clients_config.urdf_path="path/to/urdf"'
} > overwrite.txt
openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/sample_teleop_config_urdf_viz.toml \
  --robot-config="$(cat ./overwrite.txt)"
```

## Schemas for config files

The `schema` directory contains the JSON schemas for the config files used by openrr, and when combined with an extension of the editor that supports completion using the JSON schema, completion can be enabled.

### Visual Studio Code

In VS Code, you can enable completion and validation by installing the [Even Better TOML] extension and using the `evenBetterToml.schema.associations` configuration object in `settings.json`.

For example:

```json
{
  "evenBetterToml.schema.associations": {
    ".*robot_client_config.*\\.toml": "https://raw.githubusercontent.com/openrr/openrr/main/openrr-apps/schema/robot_config.json",
    ".*teleop_config.*\\.toml": "https://raw.githubusercontent.com/openrr/openrr/main/openrr-apps/schema/robot_teleop_config.json",
  },
}
```

<img width="581" alt="" src="https://user-images.githubusercontent.com/43724913/116380268-c458c700-a84e-11eb-83d2-3fd33b74183c.png">

[Even Better TOML]: https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
