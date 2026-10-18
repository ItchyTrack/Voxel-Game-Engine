use bevy::math::{IVec3, UVec3};
use voxel_trees::grid_tree::NonZeroRegion;

#[test]
fn region_rejects_empty_or_negative_sizes() {
	assert!(NonZeroRegion::new(IVec3::ZERO, IVec3::ZERO).is_none());
	assert!(NonZeroRegion::new(IVec3::ZERO, IVec3::new(1, 0, 1)).is_none());
	assert!(NonZeroRegion::from_min_end(IVec3::splat(2), IVec3::splat(1)).is_none());
}

#[test]
fn region_reports_size_and_inclusive_max() {
	let region = NonZeroRegion::new(IVec3::new(-1, 2, 3), IVec3::new(4, 5, 6)).unwrap();
	assert_eq!(region.size(), UVec3::new(4, 5, 6));
	assert_eq!(region.max(), IVec3::new(2, 6, 8));
}

#[test]
fn region_contains_is_half_open() {
	let region = NonZeroRegion::new(IVec3::ZERO, IVec3::splat(4)).unwrap();
	assert!(region.contains(IVec3::new(0, 0, 0)));
	assert!(region.contains(IVec3::new(3, 3, 3)));
	assert!(!region.contains(IVec3::new(4, 3, 3)));
	assert!(!region.contains(IVec3::new(-1, 0, 0)));
}

#[test]
fn region_intersection_and_translation() {
	let a = NonZeroRegion::new(IVec3::ZERO, IVec3::splat(8)).unwrap();
	let b = NonZeroRegion::new(IVec3::new(4, -1, 2), IVec3::splat(4)).unwrap();
	let overlap = a.intersection(b).unwrap();
	assert_eq!(overlap, NonZeroRegion::from_min_end(IVec3::new(4, 0, 2), IVec3::new(8, 3, 6)).unwrap());
	assert_eq!(overlap.translated(IVec3::new(1, 2, 3)), NonZeroRegion::from_min_end(IVec3::new(5, 2, 5), IVec3::new(9, 5, 9)).unwrap());
}
