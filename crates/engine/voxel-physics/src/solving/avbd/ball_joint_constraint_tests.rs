use super::*;

#[test]
fn attachment_center_shift_is_in_body_space() {
	let attachment = Transform::from_xyz(4, 5, 6).with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2));
	let joint = BallJoint::new(Entity::PLACEHOLDER, Entity::PLACEHOLDER, &attachment, &attachment, Fixed::MAX, Fixed::ZERO);
	let com = FixedVec3::new(Fixed::from_num(1), Fixed::from_num(2), Fixed::from_num(3));
	let mut constraint = AvbdBallJointConstraint::from_ball_joint(&joint);
	constraint.update_attachment_com(&joint, &com, &com);
	assert_eq!(constraint.body_1_attachment_com.translation, attachment.translation - com);
	assert_eq!(constraint.body_2_attachment_com.translation, attachment.translation - com);
	assert_eq!(constraint.body_1_attachment_com.rotation, attachment.rotation);
}

#[test]
fn hard_joint_initialization_uses_finite_penalties() {
	let joint = BallJoint::new(Entity::PLACEHOLDER, Entity::PLACEHOLDER, &Transform::IDENTITY, &Transform::IDENTITY, Fixed::MAX, Fixed::MAX);
	let mut constraint = AvbdBallJointConstraint::from_ball_joint(&joint);
	constraint.init(&Transform::IDENTITY, &Transform::IDENTITY);
	assert_eq!(constraint.penalty_linear, WideVec3::ONE);
	assert_eq!(constraint.penalty_angular, WideVec3::ONE);
	let huge = WideVec3::splat(Wide::MAX);
	assert_eq!(clamp_stiffness(huge, Fixed::MAX), huge);
}

#[test]
fn hard_joint_force_and_hessian_exceed_the_public_range() {
	let attachment = Transform::from_xyz(1000, 2000, 3000);
	let joint = BallJoint::new(Entity::PLACEHOLDER, Entity::PLACEHOLDER, &attachment, &attachment, Fixed::MAX, Fixed::ZERO);
	let mut constraint = AvbdBallJointConstraint::from_ball_joint(&joint);
	constraint.update_attachment_com(&joint, &FixedVec3::ZERO, &FixedVec3::ZERO);
	let initial = Transform::from_xyz(100_000_000_000i64, 0, 0);
	constraint.init(&initial, &initial);
	constraint.penalty_linear = WideVec3::splat(Wide::from_num(10_000_000_000u64));
	let displaced = initial.with_translation(initial.translation + FixedVec3::new(Fixed::from_num(1000), Fixed::ZERO, Fixed::ZERO));
	let (force, h) = constraint.get_updated(&displaced, &initial, &initial, &initial, Wide::ZERO, true).unwrap();
	assert!(force.upper_vec3().x > Wide::from(Fixed::MAX));
	assert!(h.col(5).get(5) > Wide::from(Fixed::MAX));
	constraint.update_dual(&displaced, &initial, &initial, &initial, Wide::ZERO);
	assert_eq!(constraint.lambda_linear.x, force.upper_vec3().x);
	assert!(constraint.penalty_linear.length() <= Wide::from_num(10_000_000_001u64));
}
