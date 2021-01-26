# OpenRR GUI

## Examples

### Sample robot

- Launch urdf-viz.

  ```bash
  urdf-viz ./openrr-planner/sample.urdf
  ```

- Launch joint_position_sender.

  ```bash
  cargo run -p openrr-gui --example joint_position_sender -- \
    ./openrr-apps/config/sample_robot_client_config_for_command_urdf_viz.toml \
    ./openrr-planner/sample.urdf
  ```

  or

  ```bash
  cargo build --release -p openrr-gui --example joint_position_sender
  ./target/release/examples/joint_position_sender \
    ./openrr-apps/config/sample_robot_client_config_for_command_urdf_viz.toml \
    ./openrr-planner/sample.urdf
  ```

## Troubleshooting

- Q. Fails to compile.

  A. Try to install dependencies mentioned in root [README.md](../README.md).

- Q. Fails to launch GUI.

  A. There are several possibilities:

  - If the error is command-line arguments or IO related, it's likely that the argument is wrong.
  - If you get the error that "GraphicsAdapterNotFound", try passing the "--features iced/glow" argument to `cargo build` or `cargo run`.
  - If that doesn't fix the problem, try passing the `LIBGL_ALWAYS_SOFTWARE=1` environment variable.
  - On VM (e.g., VirtualBox), you may need to disable hardware acceleration.
