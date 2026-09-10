use super::{GridTree64, U16Cell};
use crate::{signed_grid_tree::SignedGridTree, views::GridTreeView};
use bevy::math::{I8Vec3, IVec3, UVec3};
use voxel_math::{Fixed, FixedVec3, Ray, Transform};

fn point(x: i32, y: i32, z: i32) -> FixedVec3 { FixedVec3::from(IVec3::new(x, y, z)) }

#[test]
fn parallel_ray_hits_at_exact_distance_and_respects_limit() {
	let mut tree = GridTree64::<U16Cell>::new();
	tree.insert(&UVec3::new(0, 0, 3), 1);
	let ray = Ray { origin: point(0, 0, -1), direction: FixedVec3::Z };
	let transform: &dyn Transform = &ray;
	assert_eq!(tree.raycast(transform, Some(Fixed::from_num(4))), Some((UVec3::new(0, 0, 3), I8Vec3::NEG_Z, Fixed::from_num(4))));
	assert_eq!(tree.raycast(transform, Some(Fixed::from_num(4) - Fixed::EPSILON)), None);
}

#[test]
fn negative_ray_starts_ahead_of_integer_boundary() {
	let mut tree = GridTree64::<U16Cell>::new();
	tree.insert(&UVec3::ZERO, 1);
	let ray = Ray { origin: point(1, 0, 0), direction: FixedVec3::NEG_X };
	assert_eq!(tree.raycast(&ray, None), Some((UVec3::ZERO, I8Vec3::X, Fixed::ZERO)));
	let away = Ray { origin: point(0, 0, 0), direction: FixedVec3::NEG_X };
	assert_eq!(tree.raycast(&away, None), None);
}

#[test]
fn empty_cell_skips_preserve_tied_crossings() {
	let mut tree = GridTree64::<U16Cell>::new();
	tree.insert(&UVec3::new(17, 17, 0), 1);
	let ray = Ray { origin: point(0, 0, 0), direction: point(1, 1, 0) };
	let hit = tree.raycast(&ray, None).unwrap();
	assert_eq!(hit.0, UVec3::new(17, 17, 0));
	assert_eq!(hit.2, Fixed::from_num(17));
}

#[test]
fn reflected_parallel_boundaries_keep_world_voxel_ownership() {
	let mut tree = SignedGridTree::<U16Cell>::new();
	tree.insert(IVec3::new(-1, -1, 0), 1);
	let ray = Ray { origin: point(1, -1, 0), direction: FixedVec3::NEG_X };
	assert_eq!(tree.raycast(&ray, None), Some((IVec3::new(-1, -1, 0), I8Vec3::X, Fixed::ONE)));
	let outside = Ray { origin: point(1, 0, 0), direction: FixedVec3::NEG_X };
	assert_eq!(tree.raycast(&outside, None), None);
}
