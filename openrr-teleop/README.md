# openrr-teleop

[![crates.io](https://img.shields.io/crates/v/openrr-teleop.svg?logo=rust)](https://crates.io/crates/openrr-teleop) [![docs](https://docs.rs/openrr-teleop/badge.svg)](https://docs.rs/openrr-teleop) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/openrr_teleop)

## How to control using Gamepad

In this section, button names follow [gilrs layout](https://docs.rs/gilrs/latest/gilrs/#controller-layout).

### General

| Button  | Function    |
| ------- | ----------- |
| `North` | Change mode |

### `ik` mode

| Button          | Function                                  |
| --------------- | ----------------------------------------- |
| `South`         | Positive yaw translation of hand posture  |
| `West`          | Negative yaw translation of hand posture  |
| `LeftTrigger2`  | Boost speed                               |
| `RightTrigger2` | Enable switch                             |
| `LeftStick`     | Roll or Pitch rotation of hand posture    |
| `RightStick`    | Roll or Pitch translation of hand posture |
| `DPadRight`     | Positive yaw rotation of hand posture     |
| `DPadLeft`      | Negative yaw rotation of hand posture     |

### `joints_pose_sender` mode

| Button          | Function                         |
| --------------- | -------------------------------- |
| `East`          | Selection of registered postures |
| `West`          | Allow move to registered posture |
| `RightTrigger2` | Enable switch                    |

### `joints` mode

| Button          | Function                   |
| --------------- | -------------------------- |
| `South`         | Negative movement of joint |
| `East`          | Select joint               |
| `West`          | Positive movement of joint |
| `LeftTrigger2`  | Boost speed                |
| `RightTrigger2` | Enable switch              |
| `RightStick`    | Movement of joint          |

### `move_base` mode

| Button          | Function      |
| --------------- | ------------- |
| `LeftTrigger2`  | Boost speed   |
| `RightTrigger2` | Enable switch |
| `LeftStick`     | Translation   |
| `RightStick`    | Rotation      |

### `robot_command_executer` mode

| Button          | Function       |
| --------------- | -------------- |
| `East`          | Select command |
| `West`          | Send command   |
| `RightTrigger2` | Enable switch  |

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
