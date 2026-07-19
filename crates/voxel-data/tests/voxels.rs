use bevy::math::{IVec3, U16Vec3};
use voxel_data::{
	grid_tree::GridRegion,
	voxels::{Voxel, VoxelTypeInfo, Voxels},
};

fn test_type_info() -> VoxelTypeInfo {
	VoxelTypeInfo { id: voxel_data::voxels::VoxelTypeId(1), size_bytes: 8 }
}

fn vox(c: u8) -> Voxel {
	Voxel::new(test_type_info().id, [c, c, c, 255, 1, 0, 0, 0])
}

#[test]
fn bounding_box_rebuild_matches_remaining_occupancy() {
	let mut voxels = Voxels::new_with_type(test_type_info());
	voxels.add_area(U16Vec3::new(2, 3, 4), U16Vec3::new(6, 6, 6), vox(1).get_ref());
	voxels.add_area(U16Vec3::new(18, 19, 20), U16Vec3::new(2, 2, 2), vox(2).get_ref());
	voxels.remove_area(U16Vec3::new(2, 3, 4), U16Vec3::new(6, 6, 6));

	assert_eq!(voxels.bounding_box(), Some((U16Vec3::new(18, 19, 20), U16Vec3::new(19, 20, 21))));
}

#[test]
fn merge_region_from_updates_bounds_from_exact_occupied_region() {
	let mut source = Voxels::new_with_type(test_type_info());
	source.add_voxel(U16Vec3::new(1, 1, 1), vox(1).get_ref());
	source.add_voxel(U16Vec3::new(10, 10, 10), vox(2).get_ref());
	let region = GridRegion::from_min_size(IVec3::new(9, 9, 9), IVec3::splat(4)).unwrap();

	let mut dest = Voxels::new_with_type(test_type_info());
	dest.merge_region_from(&source, Some(region), IVec3::new(5, 0, 0));

	assert_eq!(dest.bounding_box(), Some((U16Vec3::new(15, 10, 10), U16Vec3::new(15, 10, 10))));
	assert_eq!(dest.voxel(&U16Vec3::new(15, 10, 10)), Some(vox(2).get_ref()));
	assert_eq!(dest.voxel(&U16Vec3::new(6, 1, 1)), None);
}
