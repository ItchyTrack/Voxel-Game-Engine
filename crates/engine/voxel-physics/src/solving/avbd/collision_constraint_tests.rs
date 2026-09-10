use super::*;
use bevy::prelude::{Entity, IVec3};
use voxel_math::{Fixed, FixedVec3};

#[test]
fn warm_contact_force_friction_and_hessian_use_wide_values() {
	let initial = Transform::from_xyz(100_000_000_000i64, 0, 0);
	let local = FixedVec3::new(Fixed::from_num(1000), Fixed::from_num(2000), Fixed::from_num(3000));
	let part = |local| collision::HalfCollision {
		body_id: Entity::PLACEHOLDER,
		grid_id: Entity::PLACEHOLDER,
		voxel_pos: IVec3::ZERO,
		feature: collision::CubeFeature::Face { xyzs: 0 },
		collision: initial * local,
		local_collision: local,
	};
	let collision = collision::Collision { part1: part(local), part2: part(local + FixedVec3::Y) };
	let penalty = WideVec3::splat(Wide::from_num(10_000_000_000u64));
	let lambda = WideVec3::new(Wide::from_num(-10_000_000_000_000i64), Wide::from_num(10_000_000_000_000u64), Wide::from_num(10_000_000_000_000u64));
	let mut constraint = CollisionConstraint::new(collision, &penalty, &lambda);
	constraint.init(&initial, &initial);
	let (force, h) = constraint.get_updated(&initial, &initial, &initial, &initial, Wide::ZERO, true).unwrap();
	assert!(force.upper_vec3().length() > Wide::from(Fixed::MAX));
	assert!(h.col(5).get(5) > Wide::from(Fixed::MAX));
	constraint.update_dual(&initial, &initial, &initial, &initial, Wide::ZERO);
	assert!(constraint.lambda.x < -Wide::from(Fixed::MAX));
	let tangential = WideVec3::new(Wide::ZERO, constraint.lambda.y, constraint.lambda.z).length();
	assert!(tangential <= constraint.lambda.x.abs() * constraint.friction + Wide::from_num(1));
	assert!(constraint.penalty.x <= Wide::from_num(10_000_000_000u64));
}
