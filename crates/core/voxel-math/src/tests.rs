use super::*;

#[test]
fn arithmetic_rounds_halfway_to_even() {
	let half = Fixed::from_num(0.5);
	assert_eq!(Fixed::EPSILON * half, Fixed::ZERO);
	assert_eq!(Fixed::from_bits(3) * half, Fixed::from_bits(2));
	assert_eq!(Fixed::from_bits(-3) * half, Fixed::from_bits(-2));
	assert_eq!(Fixed::from_bits(3) / Fixed::from_num(2), Fixed::from_bits(2));
	assert_eq!(Fixed::from_num(2.5).round(), Fixed::from_num(2));
	assert_eq!(Fixed::from_num(-3.5).round(), Fixed::from_num(-4));
}

#[test]
fn checked_overflow_is_not_wrapped() {
	assert_eq!(Fixed::MAX.checked_add(Fixed::EPSILON), None);
	assert_eq!(Fixed::MIN.checked_sub(Fixed::EPSILON), None);
	assert_eq!(Fixed::MAX.checked_mul(Fixed::from_num(2)), None);
	assert_eq!(Fixed::ONE.checked_div(Fixed::ZERO), None);
	assert!(std::panic::catch_unwind(|| Fixed::MAX + Fixed::EPSILON).is_err());
	assert!(std::panic::catch_unwind(|| -Fixed::MIN).is_err());
}

#[test]
fn square_roots_and_normalization_use_wide_values() {
	assert_eq!(Fixed::from_num(9).sqrt(), Fixed::from_num(3));
	assert_eq!(Fixed::EPSILON.sqrt().to_bits(), 4096);
	let large = FixedVec3::new(Fixed::from_num(300_000_000), Fixed::from_num(400_000_000), Fixed::ZERO);
	assert_eq!(large.length(), Fixed::from_num(500_000_000));
	assert!(large.normalize().abs_diff_eq(FixedVec3::new(Fixed::from_num(0.6), Fixed::from_num(0.8), Fixed::ZERO), Fixed::EPSILON));
	assert!(FixedVec3::MAX.normalize().is_normalized());
	assert!(FixedVec3::MIN.normalize().is_normalized());
	let tiny = FixedVec3::new(Fixed::EPSILON, Fixed::EPSILON, Fixed::ZERO).normalize();
	assert_eq!(tiny.x, tiny.y);
	assert!(tiny.is_normalized());
}

#[test]
fn quaternion_rotation_keeps_large_coordinates_in_fixed_point() {
	let position = FixedVec3::new(Fixed::from_num(100_000_000) + Fixed::EPSILON, Fixed::from_num(2), Fixed::from_num(3));
	assert_eq!(Quat::IDENTITY * position, position);
	let half_turn = Quat::from_xyzw(0.0, 0.0, 1.0, 0.0);
	assert_eq!(half_turn * position, FixedVec3::new(-position.x, -position.y, position.z));
}

#[test]
fn parallel_slabs_and_distances_are_fixed_point() {
	let bounds = (FixedVec3::ZERO, FixedVec3::ONE);
	let origin = FixedVec3::new(Fixed::from_num(0.5), Fixed::from_num(0.5), Fixed::NEG_ONE);
	assert_eq!(ray_aabb_intersection(&origin, &FixedVec3::Z, &bounds), Some(Fixed::ONE));
	assert_eq!(ray_aabb_intersection(&(origin + FixedVec3::X), &FixedVec3::Z, &bounds), None);
	assert_eq!(ray_aabb_intersection(&FixedVec3::ZERO, &FixedVec3::Z, &bounds), Some(Fixed::ZERO));
}
