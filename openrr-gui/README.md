# OpenRR GUI

## Troubleshooting

- Q. Fails to compile.

  A. Try to install dependencies mentioned in root [README.md](../README.md).

- Q. Fails to launch GUI.

  A. There are several possibilities:

  - If the error is command-line arguments or IO related, it's likely that the argument is wrong.
  - If you get the error that "GraphicsAdapterNotFound", try passing the "--features iced/glow" argument to `cargo build` or `cargo run`.
  - If that doesn't fix the problem, try passing the `LIBGL_ALWAYS_SOFTWARE=1` environment variable.
  - On VM (e.g., VirtualBox), you may need to disable hardware acceleration.
