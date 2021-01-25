# OpenRR applications

## Preprare

### Install urdf-viz

```bash
cargo install urdf-viz
```

### Build

- Without ROS

```bash
cargo build --release
```

- With ROS

```bash
cd openrr-apps
cargo build --release --features ros
```

### For UR10 sample

Install [Universal Robot software](https://github.com/ros-industrial/universal_robot).

## How to run openrr_apps_robot_command

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf
```

- Run sample commands.

```bash
./target/release/openrr_apps_robot_command \
  --config-path=./openrr-apps/config/sample_robot_client_config_for_command_urdf_viz.toml \
  load_commands \
  ./openrr-apps/config/sample_cmd_urdf_viz.txt
```

### UR10 (urdf-viz)

- Launch urdf-viz.

```bash
urdf-viz $(rospack find ur_description)/urdf/ur10_robot.urdf.xacro
```

- Run sample commands.

Change urdf path in [the setting file](./config/ur10_robot_client_config_for_command_urdf_viz.toml) for your environment.

```bash
./target/release/openrr_apps_robot_command \
  --config-path=./openrr-apps/config/ur10_robot_client_config_for_command_urdf_viz.toml \
  load_commands \
  ./openrr-apps/config/ur10_cmd_urdf_viz.txt
```

### UR10 (ROS gazebo)

- Launch gazebo.

```bash
roslaunch ur_gazebo ur10.launch
```

- Run sample commands.

Change urdf path in [the setting file](./config/ur10_robot_client_config_for_command_ros.toml) for your environment.

```bash
./target/release/openrr_apps_robot_command \
  --config-path=./openrr-apps/config/ur10_robot_client_config_for_command_ros.toml \
  load_commands \
  ./openrr-apps/config/ur10_cmd_ros.txt
```

## How to run openrr_apps_robot_teleop

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf
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
./target/release/openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/sample_teleop_config_urdf_viz.toml
```

### UR10 (urdf-viz)

- Launch urdf-viz.

```bash
urdf-viz $(rospack find ur_description)/urdf/ur10_robot.urdf.xacro
```

- Run teleop.

Change urdf path and joystick settings (see [here](#joystick)) in [the setting file](./config/ur10_robot_client_config_for_teleop_urdf_viz.toml) for your environment.


```bash
./target/release/openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/ur10_teleop_config_urdf_viz.toml \
```

### UR10 (ROS gazebo)

- Launch gazebo.

```bash
roslaunch ur_gazebo ur10.launch
```

- Run teleop.

Change urdf path and joystick settings (see [here](#joystick)) in [the setting file](./config/ur10_robot_client_config_for_teleop_ros.toml) for your environment.


```bash
./target/release/openrr_apps_robot_teleop \
  --config-path=./openrr-apps/config/ur10_teleop_config_ros.toml \
```
