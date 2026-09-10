use super::*;
use bevy::math::UVec3;
use voxel_data::voxels::{Voxel, VoxelTypeInfo};

fn data() -> (Voxels, VoxelMassReaders, Voxel) {
	let info = VoxelTypeInfo { id: VoxelTypeId(1), size_bytes: 1 };
	let mut readers = VoxelMassReaders::default();
	readers.readers.insert(info.id, |_| 1);
	(Voxels::new_with_type(info), readers, Voxel::new(info.id, [1]))
}

#[test]
fn leaf_centers_keep_the_full_first_moment() {
	let (mut voxels, readers, voxel) = data();
	let positions = [0u32, 1, 3, 9, 10, 17, 31, 63, 64];
	for x in positions { voxels.add_voxel(UVec3::new(x, 0, 0), voxel.get_ref()); }
	let origin = IVec3::splat(i32::MAX);
	let result = mass_properties_of_voxels(&readers, &voxels, origin).unwrap();
	let center = Fixed::from_num(positions.into_iter().sum::<u32>()) / Fixed::from_num(positions.len()) + Fixed::from_num(0.5);
	assert_eq!(result.mass, Mass(positions.len() as u64));
	assert_eq!(result.center_of_mass.0.x, Fixed::from_num(i32::MAX) + center);
}

#[test]
#[should_panic(expected = "voxel mass overflow")]
fn leaf_mass_overflow_is_checked_in_release() {
	let (mut voxels, mut readers, voxel) = data();
	readers.readers.insert(voxel.type_id(), |_| u64::MAX);
	voxels.add_area(UVec3::ZERO, UVec3::splat(16), voxel.get_ref());
	mass_properties_of_voxels(&readers, &voxels, IVec3::ZERO);
}
