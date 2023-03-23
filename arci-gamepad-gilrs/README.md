# arci-gamepad-gilrs

[![crates.io](https://img.shields.io/crates/v/arci-gamepad-gilrs.svg?logo=rust)](https://crates.io/crates/arci-gamepad-gilrs) [![docs](https://docs.rs/arci-gamepad-gilrs/badge.svg)](https://docs.rs/arci-gamepad-gilrs) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/arci_gamepad_gilrs)

[`arci::Gamepad`](https://docs.rs/arci/*/arci/trait.Gamepad.html) implementation using [gilrs](https://gitlab.com/gilrs-project/gilrs).

## Dependencies

### Linux

```bash
sudo apt install libudev-dev
```

## FAQ

Q. I can't open any devices on Linux
A. You need to join `input` group if you don't have the permissions.

```bash
sudo adduser $USER input
```

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
