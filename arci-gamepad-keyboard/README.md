# arci-gamepad-keyboard

[![crates.io](https://img.shields.io/crates/v/arci-gamepad-keyboard.svg)](https://crates.io/crates/arci-gamepad-keyboard) [![docs](https://docs.rs/arci-gamepad-keyboard/badge.svg)](https://docs.rs/arci-gamepad-keyboard)

[`arci::Gamepad`](https://docs.rs/arci/*/arci/trait.Gamepad.html) implementation for keyboard.

Currently, this crate only supports Unix-like operating systems.

## Examples

```bash
urdf-viz ./openrr-planner/sample.urdf &
cargo run -p openrr-apps --bin openrr_apps_robot_teleop_keyboard -- --config-path ./openrr-apps/config/sample_teleop_config_urdf_viz.toml
```

## Key mappings

```text
LeftStick:
   q    w    e
   a    s    d
   z    x    c

RightStick:
   u    i    o
   j    k    l
   m    ,    .

5 : ^ (DPadUp)
r : < (DPadLeft)
t : > (DPadRight)
f : v (DPadDown)

y : △ (North)
g : □ (West)
h : ○ (East)
b : x (South)

1 : L1 (LeftTrigger)
2 : L2 (LeftTrigger2)
3 : L3 (LeftThumb)

8 : R1 (RightTrigger)
9 : R2 (RightTrigger2)
0 : R3 (RightThumb)

6 : Select
7 : Start
```

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
