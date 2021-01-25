# Hot to run.

## openrr_apps_robot_command

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

## openrr_apps_robot_teleop

### Sample robot

- Launch urdf-viz.

```bash
urdf-viz ./openrr-planner/sample.urdf
```

- Connect joystick.

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
