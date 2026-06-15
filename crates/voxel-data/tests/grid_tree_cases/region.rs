use bevy::math::IVec3;
use voxel_data::grid_tree::GridRegion;

#[test]
fn region_rejects_empty_or_negative_sizes() {
	assert!(GridRegion::from_min_size(IVec3::ZERO, IVec3::ZERO).is_none());
	assert!(GridRegion::from_min_size(IVec3::ZERO, IVec3::new(1, 0, 1)).is_none());
	assert!(GridRegion::new(IVec3::splat(2), IVec3::splat(1)).is_none());
}

#[test]
fn region_reports_size_and_inclusive_max() {
	let region = GridRegion::from_min_size(IVec3::new(-1, 2, 3), IVec3::new(4, 5, 6)).unwrap();
	assert_eq!(region.size(), IVec3::new(4, 5, 6));
	assert_eq!(region.max_inclusive(), IVec3::new(2, 6, 8));
}

#[test]
fn region_contains_is_half_open() {
	let region = GridRegion::from_min_size(IVec3::ZERO, IVec3::splat(4)).unwrap();
	assert!(region.contains(IVec3::new(0, 0, 0)));
	assert!(region.contains(IVec3::new(3, 3, 3)));
	assert!(!region.contains(IVec3::new(4, 3, 3)));
	assert!(!region.contains(IVec3::new(-1, 0, 0)));
}

#[test]
fn region_intersection_and_translation() {
	let a = GridRegion::from_min_size(IVec3::ZERO, IVec3::splat(8)).unwrap();
	let b = GridRegion::from_min_size(IVec3::new(4, -1, 2), IVec3::splat(4)).unwrap();
	let overlap = a.intersection(b).unwrap();
	assert_eq!(overlap, GridRegion { min: IVec3::new(4, 0, 2), end: IVec3::new(8, 3, 6) });
	assert_eq!(overlap.translated(IVec3::new(1, 2, 3)), GridRegion { min: IVec3::new(5, 2, 5), end: IVec3::new(9, 5, 9) });
}
