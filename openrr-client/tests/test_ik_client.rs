use assert_approx_eq::assert_approx_eq;

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

#[test]
fn test_create_jacobian_ik_solver() {
    let params = IkSolverParameters {
        allowable_position_error: 0.01,
        allowable_angle_error: 0.02,
        jacobian_multiplier: 0.1,
        num_max_try: 100,
    };
    let solver = create_jacobian_ik_solver(&params);
    assert_approx_eq!(solver.allowable_target_distance, 0.01);
    assert_approx_eq!(solver.allowable_target_angle, 0.02);
    assert_approx_eq!(solver.jacobian_multiplier, 0.1);
    assert_eq!(solver.num_max_try, 100);
}

#[test]
fn test_create_random_jacobian_ik_solver() {
    let params = IkSolverParameters {
        allowable_position_error: 0.01,
        allowable_angle_error: 0.02,
        jacobian_multiplier: 0.1,
        num_max_try: 100,
    };
    let solver = create_random_jacobian_ik_solver(&params);
    assert_approx_eq!(solver.solver.allowable_target_distance, 0.01);
    assert_approx_eq!(solver.solver.allowable_target_angle, 0.02);
    assert_approx_eq!(solver.solver.jacobian_multiplier, 0.1);
    assert_eq!(solver.num_max_try, 100);
}
