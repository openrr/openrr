#[macro_use]
extern crate approx;

use openrr_planner::*;

#[test]
fn test_funcs() {
//	use crate::errors::*;
	use std::f64::consts::PI;

	let limits: Vec<Option<k::joint::Range<f64>>> = vec![
		None,
		Some(k::joint::Range::new(-1.0, 1.0)),
		Some(k::joint::Range::new(0.0, 0.1)),
		];
		for _ in 0..1000 {
			let angles = generate_random_joint_positions_from_limits(&limits);
				assert_eq!(angles.len(), limits.len());
				assert!(angles[0] >= -PI && angles[0] < PI);
				assert!(angles[1] >= -1.0 && angles[1] < 1.0);
				assert!(angles[2] >= 0.0 && angles[2] < 0.1);
		}
		let angles_fail = vec![0.1];
		assert!(generate_clamped_joint_positions_from_limits(&angles_fail, &limits).is_err());

	let angles1 = vec![100.0, -2.0, 0.5];
	let clamped = generate_clamped_joint_positions_from_limits(&angles1, &limits).unwrap();
	const TORELANCE: f64 = 0.00001;
	assert!((clamped[0] - 100.0).abs() < TORELANCE);
	assert!((clamped[1] - (-1.0)).abs() < TORELANCE);
	assert!((clamped[2] - 0.1).abs() < TORELANCE);
}

#[test]
fn test_interpolate()
{
	use trajectory::{CubicSpline, Trajectory};

// parametor for test
    let  total_duration = 4.0_f64;
    let  unit_duration = 0.01_f64;
	let  points = vec![
							vec![0.0, -1.0],
							vec![2.0, -3.0],
							vec![3.0, 3.0],
							vec![1.0, 5.0],
						];

// preparing vector of times for interpolate
	let key_frame_unit_duration = total_duration / (points.len() as f64 - 1.0_f64);
	let time = (0_usize..points.len())
					.map( |t| t as f64*key_frame_unit_duration)
					.collect::<Vec<f64>>();
	assert_eq!(time.len(), points.len());

// interpolate
	let func_result = interpolate(&points, total_duration, unit_duration);
	let spline = CubicSpline::new(time, points).expect("failed interpolation");
	let t_points;
	match func_result
	{
		Some(some) =>	{
								t_points = some;
							},
		None => panic!("failed interpolation(Trajectory Points)")
	}

// preparing vector of times by unit_duration
	let time = (0..(total_duration / unit_duration) as i32)
					.map( |t| t as f64*unit_duration)
					.collect::<Vec<f64>>();

// check trajectory point, position, velocity and acceleration
// check all time points
	for (t_point, t) in t_points.iter().zip(time)
	{
		// check value of all dimension
		t_point.position
			.iter()
			.zip(spline.position(t).unwrap())
			.for_each( |(t_point, correct)| assert_relative_eq!(*t_point, correct, max_relative = 1.0) );
		t_point.velocity
			.iter()
			.zip(spline.velocity(t).unwrap())
			.for_each( |(t_point, correct)| assert_relative_eq!(*t_point, correct, max_relative = 1.0) );
		t_point.acceleration
			.iter()
			.zip(spline.acceleration(t).unwrap())
			.for_each( |(t_point, correct)| assert_relative_eq!(*t_point, correct, max_relative = 1.0) );
	}
}

