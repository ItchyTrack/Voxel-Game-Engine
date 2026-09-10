use super::*;

#[test]
fn timestep_square_uses_the_wide_format() {
	assert!(!usable_timestep(fixed_duration(std::time::Duration::ZERO)));
	assert!(!usable_timestep(fixed_duration(std::time::Duration::from_nanos(1))));
	assert!(usable_timestep(fixed_duration(std::time::Duration::from_micros(10))));
	assert!(usable_timestep(Fixed::EPSILON));
	assert_eq!(Wide::from(Fixed::EPSILON) * Wide::from(Fixed::EPSILON), Wide::EPSILON);
}

#[test]
fn dot_cancels_large_products() {
	let large = Wide::from_num(100_000_000u64);
	let a = Vec6::new(large, large, Wide::ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	let b = Vec6::new(large, -large, Wide::ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	assert_eq!(a.dot(&b), Wide::ONE);
}

#[test]
#[should_panic(expected = "solver multiplication overflow")]
fn dot_rejects_wide_overflow() {
	Vec6::splat(Wide::MAX).dot(&Vec6::splat(Wide::MAX));
}

#[test]
#[should_panic(expected = "solver addition overflow")]
fn addition_checks_overflow_in_release() {
	let _ = Wide::MAX + Wide::EPSILON;
}

#[test]
#[should_panic(expected = "solver output exceeds Q40.24 range")]
fn public_output_does_not_saturate() {
	Wide::from_num(1_000_000_000_000u64).to_fixed();
}

#[test]
fn widening_is_exact_and_narrowing_rounds_ties_even() {
	for value in [Fixed::MIN, Fixed::MAX, Fixed::EPSILON, Fixed::ZERO, Fixed::NEG_ONE] {
		assert_eq!(Wide::from(value).to_fixed(), value);
	}
	let half = Wide::from(Fixed::EPSILON) / Wide::from_num(2);
	assert_eq!(half.to_fixed(), Fixed::ZERO);
	assert_eq!((-half).to_fixed(), Fixed::ZERO);
	assert_eq!((half * Wide::from_num(3)).to_fixed(), Fixed::from_bits(2));
	assert_eq!((-half * Wide::from_num(3)).to_fixed(), Fixed::from_bits(-2));
}

#[test]
fn multiplication_and_norm_do_not_need_narrow_products() {
	let large = Wide::from_num(100_000_000_000_000u64);
	assert_eq!(large * Wide::from_num(1_000_000), Wide::from_num(100_000_000_000_000_000_000u128));
	let force = WideVec3::new(Wide::from_num(3_000_000_000_000_000_000u64), Wide::from_num(4_000_000_000_000_000_000u64), Wide::ZERO);
	let length = force.length();
	let expected = Wide::from_num(5_000_000_000_000_000_000u64);
	assert!((length / expected - Wide::ONE).abs() < Wide::from_num(0.000000001));
	assert!((force.clamp_length_max(Wide::from_num(100)).length() - Wide::from_num(100)).abs() < Wide::from_num(0.000001));
}

fn assert_close(a: Vec6, b: Vec6, tolerance: Wide) {
	for i in 0..6 { assert!((a.get(i) - b.get(i)).abs() <= tolerance, "axis {i}: {} != {}", a.get(i), b.get(i)); }
}

#[test]
fn coupled_solve_handles_scene_masses_and_large_inertia() {
	let dt = Wide::from(fixed_duration(std::time::Duration::from_nanos(1_000_000_000 / 120)));
	for mass in [100_000u64, 400_000] {
		for inertia in [3_000_000u64, 100_000_000_000_000] {
			let mut h = Mat6::from_mat3(Wide::from_num(mass) * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, Mat3::from_inertia(DMat3::IDENTITY * inertia as f64)) / (dt * dt);
			let j = Vec6::new(Wide::ONE, Wide::NEG_ONE, Wide::ONE, Wide::from_num(20), Wide::from_num(-30), Wide::from_num(10));
			for i in 0..6 { *h.col_mut(i) += j * (j.get(i) * Wide::from_num(10_000_000_000u64)); }
			let expected = Vec6::new(Wide::from_num(0.25), Wide::from_num(-0.5), Wide::ONE, Wide::from_num(0.02), Wide::from_num(-0.03), Wide::from_num(0.01));
			let result = solve_symmetric(h, h * expected);
			assert_close(result, expected, Wide::from_num(0.000001));
		}
	}
}

#[test]
fn ldl_projects_zero_and_quantized_singular_axes() {
	assert_eq!(solve_symmetric(Mat6::ZERO, Vec6::ONE), Vec6::ZERO);
	let mut h = Mat6::IDENTITY;
	*h.col_mut(0) = Vec6::ZERO;
	let solved = solve_symmetric(h, Vec6::ONE);
	assert_eq!(solved.get(0), Wide::ZERO);
	for i in 1..6 { assert_eq!(solved.get(i), Wide::ONE); }

	// A rank-one block, including either sign of quantization at its flat pivot.
	for error in [Wide::ZERO, Wide::EPSILON, -Wide::EPSILON] {
		let mut h = Mat6::IDENTITY;
		*h.col_mut(0).get_mut(1) = Wide::ONE;
		*h.col_mut(1).get_mut(0) = Wide::ONE;
		*h.col_mut(1).get_mut(1) += error;
		let solved = solve_symmetric(h, Vec6::ONE);
		assert_close(h * solved, Vec6::ONE, Wide::EPSILON * Wide::from_num(32));
		assert!(solved.length() < Wide::from_num(3));
	}
}

#[test]
fn tiny_inverse_inertia_retains_angular_response() {
	let inverse = Mat3::from_inertia(DMat3::IDENTITY * 1e-12);
	assert!(inverse.x_axis.x > Wide::ZERO);
	let impulse = WideVec3::X * Wide::from_num(100_000_000u64);
	let velocity = (inverse * impulse).to_fixed();
	assert!(velocity.x > Fixed::ZERO);
	assert!(velocity.x.abs_diff_eq(Fixed::from_num(0.0001), Fixed::from_num(0.000001)));

	// Direct inertia solving avoids even the small error from a wide inverse.
	let h = Mat6::from_mat3(Mat3::ZERO, Mat3::ZERO, Mat3::ZERO, Mat3::from_inertia(DMat3::IDENTITY * 1e12));
	let response = solve_symmetric(h, Vec6::from_vec3(WideVec3::ZERO, impulse)).lower_vec3().to_fixed();
	assert!(response.x.abs_diff_eq(Fixed::from_num(0.0001), Fixed::EPSILON));
}

#[test]
fn coupled_angular_axes_keep_large_off_diagonal_inertia() {
	use bevy::math::DVec3;
	let inertia = Mat3::from_inertia(DMat3::from_cols(DVec3::new(4e14, 1e14, 2e14), DVec3::new(1e14, 3e14, 1e14), DVec3::new(2e14, 1e14, 5e14)));
	let h = Mat6::from_mat3(Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, inertia);
	let expected = Vec6::from_vec3(WideVec3::ONE, WideVec3::new(Wide::from_num(0.02), Wide::from_num(-0.03), Wide::from_num(0.01)));
	assert_close(solve_symmetric(h, h * expected), expected, Wide::from_num(0.000000001));
}
