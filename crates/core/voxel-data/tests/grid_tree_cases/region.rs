use bevy::math::{IVec3, UVec3};
use voxel_trees::grid_tree::NonZeroVoxelRegion;

#[test]
fn region_rejects_empty_or_negative_sizes() {
	assert!(NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::ZERO).is_none());
	assert!(NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::new(1, 0, 1)).is_none());
	assert!(NonZeroVoxelRegion::from_min_end(IVec3::splat(2), IVec3::splat(1)).is_none());
}

#[test]
fn region_reports_size_and_inclusive_max() {
	let region = NonZeroVoxelRegion::from_min_size(IVec3::new(-1, 2, 3), IVec3::new(4, 5, 6)).unwrap();
	assert_eq!(region.size(), UVec3::new(4, 5, 6));
	assert_eq!(region.max(), IVec3::new(2, 6, 8));
}

#[test]
fn region_contains_is_half_open() {
	let region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::splat(4)).unwrap();
	assert!(region.contains(IVec3::new(0, 0, 0)));
	assert!(region.contains(IVec3::new(3, 3, 3)));
	assert!(!region.contains(IVec3::new(4, 3, 3)));
	assert!(!region.contains(IVec3::new(-1, 0, 0)));
}

#[test]
fn region_intersection_and_translation() {
	let a = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::splat(8)).unwrap();
	let b = NonZeroVoxelRegion::from_min_size(IVec3::new(4, -1, 2), IVec3::splat(4)).unwrap();
	let overlap = a.intersection(b).unwrap();
	assert_eq!(overlap, NonZeroVoxelRegion::from_min_end(IVec3::new(4, 0, 2), IVec3::new(8, 3, 6)).unwrap());
	assert_eq!(overlap.translated(IVec3::new(1, 2, 3)), NonZeroVoxelRegion::from_min_end(IVec3::new(5, 2, 5), IVec3::new(9, 5, 9)).unwrap());
}
