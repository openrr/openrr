# OpenRR GUI

[![crates.io](https://img.shields.io/crates/v/openrr-gui.svg)](https://crates.io/crates/openrr-gui) [![docs](https://docs.rs/openrr-gui/badge.svg)](https://docs.rs/openrr-gui)


## Wasm example

```bash
cd openrr-gui
# cargo build --target wasm32-unknown-unknown --no-default-features --example joint_position_sender --release
cargo build --target wasm32-unknown-unknown --no-default-features --example joint_position_sender
# wasm-bindgen target/wasm32-unknown-unknown/release/openrr_apps_joint_position_sender.wasm --out-dir openrr_apps_joint_position_sender --web
wasm-bindgen ../target/wasm32-unknown-unknown/debug/examples/joint_position_sender.wasm --out-dir joint_position_sender --web
echo '<!DOCTYPE html>
<html>
  <head>
    <meta http-equiv="Content-type" content="text/html; charset=utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Urdf Viz</title>
  </head>
  <body>
    <script type="module">
      import init from "./joint_position_sender/joint_position_sender.js";
      init("./joint_position_sender/joint_position_sender.wasm");
    </script>
  </body>
</html>' > index.html
cargo run -p openrr-gui --example server
```

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

Licensed under the [Apache License, Version 2.0](LICENSE).

### Third party software

This product includes copies and modifications of software developed by third parties:

- [`assets/material-design-icons`](assets/material-design-icons) includes copies and modifications of icons from [google/material-design-icons](https://github.com/google/material-design-icons), licensed under the Apache License, Version 2.0.

See the license files included in these directories for more details.
