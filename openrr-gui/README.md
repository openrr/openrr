# OpenRR GUI

[![crates.io](https://img.shields.io/crates/v/openrr-gui.svg)](https://crates.io/crates/openrr-gui) [![docs](https://docs.rs/openrr-gui/badge.svg)](https://docs.rs/openrr-gui) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/openrr_gui)

## Troubleshooting

- Q. Fails to compile.

  A. Try to install dependencies mentioned in root [README.md](../README.md).

- Q. Fails to launch GUI.

  A. There are several possibilities:

  - If the error is command-line arguments or IO related, it's likely that the argument is wrong.
  - If you get the error that "GraphicsAdapterNotFound", try enabling the `glow` feature of openrr-gui or openrr-apps.
  - If that doesn't fix the problem, try passing the `LIBGL_ALWAYS_SOFTWARE=1` environment variable.
  - On VM (e.g., VirtualBox), you may need to disable hardware acceleration.

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).

### Third party software

This product includes copies and modifications of software developed by third parties:

- [`assets/material-design-icons`](assets/material-design-icons) includes copies and modifications of icons from [google/material-design-icons](https://github.com/google/material-design-icons), licensed under the Apache License, Version 2.0.

See the license files included in these directories for more details.
