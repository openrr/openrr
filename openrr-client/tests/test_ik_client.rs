use assert_approx_eq::assert_approx_eq;
use std::sync::Arc;

use openrr_client::*;

#[test]
fn test_isometry() {
    let iso3 = isometry(0.0, -1.0, 1.0, 0.0, 1.0, -1.0);
    assert_eq!(iso3.translation.vector, k::Vector3::new(0.0, -1.0, 1.0));
    let (roll, pitch, yaw) = iso3.rotation.euler_angles();
    assert_approx_eq!(roll, 0.0);
    assert_approx_eq!(pitch, 1.0);
    assert_approx_eq!(yaw, -1.0);
}

fn ik_solver_parameters(
    allowable_position_error: f64,
    allowable_angle_error: f64,
    jacobian_multiplier: f64,
    num_max_try: usize,
) -> IkSolverParameters {
    IkSolverParameters {
        allowable_position_error,
        allowable_angle_error,
        jacobian_multiplier,
        num_max_try,
    }
}

#[test]
fn test_create_jacobian_ik_solver() {
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let solver = create_jacobian_ik_solver(&params);
    assert_approx_eq!(solver.allowable_target_distance, 0.01);
    assert_approx_eq!(solver.allowable_target_angle, 0.02);
    assert_approx_eq!(solver.jacobian_multiplier, 0.1);
    assert_eq!(solver.num_max_try, 100);
}

#[test]
fn test_create_random_jacobian_ik_solver() {
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let solver = create_random_jacobian_ik_solver(&params);
    assert_approx_eq!(solver.solver.allowable_target_distance, 0.01);
    assert_approx_eq!(solver.solver.allowable_target_angle, 0.02);
    assert_approx_eq!(solver.solver.jacobian_multiplier, 0.1);
    assert_eq!(solver.num_max_try, 100);
}

#[test]
fn test_ik_solver_with_chain_new() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let _ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);
}

#[test]
fn test_ik_solver_with_chain_end_transform() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let t = ik_solver_with_chain.end_transform();
    assert_approx_eq!(t.translation.vector.x, 0.9);
    assert_approx_eq!(t.translation.vector.y, 0.4);
    assert_approx_eq!(t.translation.vector.z, 0.5);
    let (roll, pitch, yaw) = t.rotation.euler_angles();
    assert_approx_eq!(roll, 0.0);
    assert_approx_eq!(pitch, 0.0);
    assert_approx_eq!(yaw, 0.0);
}

#[test]
fn test_ik_solver_with_chain_joint_positions() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&positions).unwrap();
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let jp = ik_solver_with_chain.joint_positions();
    assert_eq!(jp, positions);
}

#[test]
fn test_ik_solver_with_chain_set_joint_positions_clamped() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let positions = vec![4.0, 3.0, 3.0, 3.0, 9.0, 3.0];
    ik_solver_with_chain.set_joint_positions_clamped(&positions);
    let jp = ik_solver_with_chain.joint_positions();
    assert_eq!(jp, vec![3.0, 1.5, 2.0, 1.5, 3.0, 2.0]);
}

#[test]
fn test_ik_solver_with_chain_solve() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&positions).unwrap();
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let mut target = ik_solver_with_chain.end_transform();
    target.translation.vector.y -= 0.1;
    target.translation.vector.z += 0.1;
    let result = ik_solver_with_chain.solve(&target);
    assert!(result.is_ok());
}

#[test]
fn test_ik_solver_with_chain_solve_error() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 10);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let mut target = ik_solver_with_chain.end_transform();
    target.translation.vector.x += 1.0;
    let result = ik_solver_with_chain.solve(&target);
    assert!(result.is_err());
}

#[test]
fn test_ik_solver_with_chain_solve_with_constraints() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&positions).unwrap();
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let mut target = ik_solver_with_chain.end_transform();
    target.translation.vector.y -= 0.5;
    target.translation.vector.z += 0.5;
    let constraints = k::Constraints {
        position_x: false,
        position_y: true,
        position_z: true,
        rotation_x: true,
        rotation_y: true,
        rotation_z: true,
    };
    let result = ik_solver_with_chain.solve_with_constraints(&target, &constraints);
    assert!(result.is_ok());
}

#[test]
fn test_ik_solver_with_chain_constraints() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&positions).unwrap();
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints {
        position_x: true,
        position_y: false,
        position_z: true,
        rotation_x: false,
        rotation_y: true,
        rotation_z: false,
    };
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let c = ik_solver_with_chain.constraints();
    assert_eq!(c.position_x, true);
    assert_eq!(c.position_y, false);
    assert_eq!(c.position_z, true);
    assert_eq!(c.rotation_x, false);
    assert_eq!(c.rotation_y, true);
    assert_eq!(c.rotation_z, false);
}

#[test]
fn test_ik_solver_with_chain_generate_trajectory_with_interpolation() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&positions).unwrap();
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let current = ik_solver_with_chain.end_transform();
    let mut target = current;
    target.translation.vector.y -= 0.1;
    target.translation.vector.z += 0.1;
    let result = ik_solver_with_chain
        .generate_trajectory_with_interpolation(&current, &target, 1.0, 0.05, 10)
        .unwrap();
    assert!(!result.is_empty());
}

#[test]
fn test_ik_solver_with_chain_generate_trajectory_with_interpolation_and_constraints() {
    let chain = k::Chain::<f64>::from_urdf_file("../openrr-planner/sample.urdf").unwrap();
    let end_link = chain.find("l_tool_fixed").unwrap();
    let arm = k::SerialChain::from_end(end_link);
    let positions = vec![0.1, 0.2, 0.0, -0.5, 0.0, -0.3];
    arm.set_joint_positions(&positions).unwrap();
    let params = ik_solver_parameters(0.01, 0.02, 0.1, 100);
    let ik_solver = create_random_jacobian_ik_solver(&params);
    let constraints = k::Constraints::default();
    let ik_solver_with_chain = IkSolverWithChain::new(arm, Arc::new(ik_solver), constraints);

    let current = ik_solver_with_chain.end_transform();
    let mut target = current;
    target.translation.vector.y -= 0.5;
    target.translation.vector.z += 0.5;
    let constraints = k::Constraints {
        position_x: false,
        position_y: true,
        position_z: true,
        rotation_x: true,
        rotation_y: true,
        rotation_z: true,
    };
    let result = ik_solver_with_chain
        .generate_trajectory_with_interpolation_and_constraints(
            &current,
            &target,
            &constraints,
            5.0,
            0.05,
            10,
        )
        .unwrap();
    assert!(!result.is_empty());
}
