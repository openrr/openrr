# OpenRR Planner

[![crates.io](https://img.shields.io/crates/v/openrr-planner.svg)](https://crates.io/crates/openrr-planner) [![docs](https://docs.rs/openrr-planner/badge.svg)](https://docs.rs/openrr-planner) [![docs](https://img.shields.io/badge/docs-main-blue)](https://openrr.github.io/openrr/openrr_planner)

Collision Avoidance Path Planning for robotics in Rust-lang.
This starts as a copy of [`gear`](https://github.com/openrr/gear) crate.

[![Video](https://j.gifs.com/kZZyJK.gif)](http://www.youtube.com/watch?v=jEu3EfpVAI8)

[Documents](https://docs.rs/openrr_planner)

## Code example

### [minimum code example](examples/minimum.rs)

```rust,no_run
use std::{path::Path, sync::Arc};

use k::nalgebra as na;
use ncollide3d::shape::Compound;
use openrr_planner::FromUrdf;

fn main() {
    let urdf_path = Path::new("sample.urdf");
    let robot = Arc::new(k::Chain::from_urdf_file(urdf_path).unwrap());

    // Create path planner with loading urdf file and set end link name
    let planner = openrr_planner::JointPathPlannerBuilder::from_urdf_file(urdf_path)
        .expect("failed to create planner from urdf file")
        .collision_check_margin(0.01)
        .reference_robot(robot.clone())
        .finalize()
        .unwrap();
    // Create inverse kinematics solver
    let solver = openrr_planner::JacobianIkSolver::default();
    let solver = openrr_planner::RandomInitializeIkSolver::new(solver, 100);
    // Create path planner with IK solver
    let mut planner = openrr_planner::JointPathPlannerWithIk::new(planner, solver);
    let target_name = "l_tool_fixed";
    // Create obstacles
    let obstacles = Compound::from_urdf_file("obstacles.urdf").expect("obstacle file not found");

    // Set IK target transformation
    let mut ik_target_pose = na::Isometry3::from_parts(
        na::Translation3::new(0.40, 0.20, 0.3),
        na::UnitQuaternion::from_euler_angles(0.0, -0.1, 0.0),
    );
    // Plan the path, path is the vector of joint angles for root to target_name
    let plan1 = planner
        .plan_with_ik(target_name, &ik_target_pose, &obstacles)
        .unwrap();
    println!("plan1 = {plan1:?}");

    // Apply plan1 to the reference robot (regarded as the real robot)
    let arm = k::Chain::from_end(robot.find(target_name).unwrap());
    arm.set_joint_positions_clamped(plan1.iter().last().unwrap());

    // Plan the path from previous result
    ik_target_pose.translation.vector[2] += 0.50;
    let plan2 = planner
        .plan_with_ik(target_name, &ik_target_pose, &obstacles)
        .unwrap();
    println!("plan2 = {plan2:?}");
}
```

## Run example with GUI

### How to run

```bash
cargo run --release --example reach
```

### GUI control

* Up/Down/Left/Right/`f`/`b` to translate IK target
* Shift + Up/Down/Left/Right/`f`/`b` to rotate IK target
* type `g` to move the end of the arm to the target
* type `i` to just solve inverse kinematics for the target without collision check
* type `r` to set random pose
* type `c` to check collision
* type `v` to toggle shown element collision<->visual

### Use your robot

The example can handle any urdf files (sample.urdf is used as default).
It requires the name of the target end link name.

```bash
cargo run --release --example reach YOUR_URDF_FILE_PATH END_LINK_NAME
```

For example,

#### PR2

```bash
cargo run --release --example reach $(rospack find pr2_description)/robots/pr2.urdf.xacro l_gripper_palm_link
```

[![Video](https://j.gifs.com/kZZyJK.gif)](http://www.youtube.com/watch?v=jEu3EfpVAI8)

#### Universal Robot: UR10

```bash
cargo run --release --example reach $(rospack find ur_description)/urdf/ur10_robot.urdf.xacro ee_link
```

[![Sawyer movie](https://j.gifs.com/ZVVqDw.gif)](https://www.youtube.com/watch?v=0YujRKUto-4)

#### Sawyer

```bash
cargo run --release --example reach $(rospack find sawyer_description)/urdf/sawyer.urdf right_hand
```

[![UR5 movie](https://j.gifs.com/G55yxL.gif)](https://www.youtube.com/watch?v=0YujRKUto-4)

## License

Licensed under the [Apache License, Version 2.0](https://github.com/openrr/openrr/blob/main/LICENSE).
