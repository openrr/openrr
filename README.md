# openrr: `Open Rust Robotics`

[![Build Status](https://img.shields.io/github/actions/workflow/status/openrr/openrr/ci.yml?branch=main&logo=github)](https://github.com/openrr/openrr/actions) [![crates.io](https://img.shields.io/crates/v/openrr.svg?logo=rust)](https://crates.io/crates/openrr) [![codecov](https://codecov.io/gh/openrr/openrr/branch/main/graph/badge.svg?token=28GTOOT4RY)](https://codecov.io/gh/openrr/openrr) [![docs](https://docs.rs/openrr/badge.svg)](https://docs.rs/openrr)

**For developers and future users**

[![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/openrr) [![discord](https://dcbadge.vercel.app/api/server/8DAFFKc88B?style=flat)](https://discord.gg/8DAFFKc88B) [![tutorial](https://img.shields.io/badge/OpenRR-Tutorial-red)](https://github.com/openrr/openrr-tutorial)

OpenRR (pronounced like "opener") is Open Rust Robotics platform.

**It's heavily under development.**

## Supported Platforms

|OS|Core|GUI|ROS|ROS2|
|--|----|---|---|---|
|Linux (Ubuntu)|✔|✔|✔|✔|
|macOS         |✔|✔|✔|  |
|Windows       |✔|✔|  |  |

* You can use ROS *without ROS installation* on Linux/macOS.
* ROS2 Support is experimental. See [arci-ros2](https://github.com/openrr/openrr/tree/main/arci-ros2) for details.

## Dependencies

### Linux

```bash
sudo apt install cmake build-essential libudev-dev xorg-dev libglu1-mesa-dev libasound2-dev libxkbcommon-dev protobuf-compiler
```

* libudev-dev (arci-gamepad-gilrs)
* cmake build-essential xorg-dev libglu1-mesa-dev libxkbcommon-dev (openrr-gui (egui))
* libasound2-dev (arci-speak-audio)
* protobuf-compiler (openrr-remote)

## Architecture

![architecture](https://raw.githubusercontent.com/openrr/openrr/main/img/architecture.png)

`arci` is a hardware abstraction layer for openrr.
Currently [ROS1](https://ros.org) and [urdf-viz](https://github.com/openrr/urdf-viz) (as a static simulator (actually it's just a viewer)) are implemented.

You can write platform/hardware independent code if you use `arci` traits.

## What is OpenRR?

OpenRR contains..

* abstract robot interfaces (`arci`)
* concrete implementation of the interfaces (`arci-ros`, `arci-urdf-viz`, ...)
* library which uses the interfaces (`openrr-client`, ...)
* tools (`openrr-apps`)
* pure libraries nothing to do with `arci` (`openrr-planner`, ...)

## Tools

Currently we have some tools to control real/sim robots.

See [openrr-apps](https://github.com/openrr/openrr/tree/main/openrr-apps) for details.

### joint_trajectory_sender

Inspired by [joint_state_publisher_gui](http://wiki.ros.org/joint_state_publisher)

<img width="400" alt="joint_sender" src="https://user-images.githubusercontent.com/43724913/106704866-27600680-6630-11eb-91ee-5eb29515fe46.png">

You can use this GUI not only for ROS but anything if you implement `arci::JointTrajectoryClient` and write a small binary main function.

### robot_command

General CLI to access `arci` robot clients. It supports not only sending joint trajectory directly but it supports inverse kinematics with self-collision check, and navigation.

## Format

To format use nightly rustfmt,

```bash
cargo +nightly fmt
```

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).

## Related openrr repositories

* [k](https://github.com/OpenRR/k) : kinematics library
* [ros-nalgebra](https://github.com/OpenRR/ros-nalgebra) : rosrust nalgebra converter generator
* [rrt](https://github.com/OpenRR/rrt) : RRT-dual-connect path planner
* [trajectory](https://github.com/OpenRR/trajectory) : trajectory interpolator
* [urdf-rs](https://github.com/OpenRR/urdf-rs) : URDF parser
* [urdf-viz](https://github.com/OpenRR/urdf-viz): URDF visualizer
* ~~[gear](https://github.com/OpenRR/gear)~~ : (deprecated) motion planning library, but it is openrr-planner now.

## Why OpenRR?

We strongly believe that Rust is the future of robotics.
OpenRR is the world first robotics platform which is made by Rust, made for Rust.
It can be a reference, a base for the future robotic people, like us.

## Contribution

We appreciate for your any contributions!
[Create an issue](https://github.com/openrr/openrr/issues/new) at first!

[Here](https://discord.gg/8DAFFKc88B) is a discord server.

## Using OpenRR

You can read the tutorial books at the following links.

* [English](https://openrr.github.io/openrr-tutorial/en/html)
* [Japanese](https://openrr.github.io/openrr-tutorial/ja/html)
