# arci-gamepad-gilrs

[![crates.io](https://img.shields.io/crates/v/arci-gamepad-gilrs.svg)](https://crates.io/crates/arci-gamepad-gilrs) [![docs](https://docs.rs/arci-gamepad-gilrs/badge.svg)](https://docs.rs/arci-gamepad-gilrs)

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
