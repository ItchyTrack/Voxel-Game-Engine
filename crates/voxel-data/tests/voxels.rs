use bevy::math::{IVec3, U16Vec3};
use voxel_data::{
	grid_tree::GridRegion,
	voxels::{Voxel, Voxels},
};

fn vox(c: u8) -> Voxel {
	Voxel { color: [c, c, c, 255], mass: 1 }
}

#[test]
fn bounding_box_rebuild_matches_remaining_occupancy() {
	let mut voxels = Voxels::new();
	voxels.add_area(U16Vec3::new(2, 3, 4), U16Vec3::new(6, 6, 6), vox(1));
	voxels.add_area(U16Vec3::new(18, 19, 20), U16Vec3::new(2, 2, 2), vox(2));
	voxels.remove_area(U16Vec3::new(2, 3, 4), U16Vec3::new(6, 6, 6));

	assert_eq!(voxels.bounding_box(), Some((U16Vec3::new(18, 19, 20), U16Vec3::new(19, 20, 21))));
}

#[test]
fn merge_region_from_updates_bounds_from_exact_occupied_region() {
	let mut source = Voxels::new();
	source.add_voxel(U16Vec3::new(1, 1, 1), vox(1));
	source.add_voxel(U16Vec3::new(10, 10, 10), vox(2));
	let region = GridRegion::from_min_size(IVec3::new(9, 9, 9), IVec3::splat(4)).unwrap();

	let mut dest = Voxels::new();
	dest.merge_region_from(&source, Some(region), IVec3::new(5, 0, 0));

	assert_eq!(dest.bounding_box(), Some((U16Vec3::new(15, 10, 10), U16Vec3::new(15, 10, 10))));
	assert_eq!(dest.voxel(&U16Vec3::new(15, 10, 10)), Some(&vox(2)));
	assert_eq!(dest.voxel(&U16Vec3::new(6, 1, 1)), None);
}
