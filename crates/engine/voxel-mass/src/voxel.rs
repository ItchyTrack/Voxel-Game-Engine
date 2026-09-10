use bevy::math::IVec3;
use voxel_math::{Fixed, FixedVec3};
use bevy::prelude::{App, Resource};
use rustc_hash::FxHashMap;
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId, Voxels};

use crate::{CenterOfMass, InertiaTensor, Mass, MassProperties, RotationalInertia};

#[cfg(test)]
#[path = "voxel_tests.rs"]
mod tests;

pub trait VoxelMassValue: VoxelType {
	fn voxel_mass(&self) -> u64;
}

type VoxelMassReader = for<'a> fn(&VoxelRef<'a>) -> u64;

#[derive(Resource, Default, Clone)]
pub struct VoxelMassReaders {
	readers: FxHashMap<VoxelTypeId, VoxelMassReader>,
}

impl VoxelMassReaders {
	pub fn register<T: VoxelMassValue>(&mut self) {
		self.readers.insert(T::TYPE_ID, |voxel: &VoxelRef<'_>| { T::from_voxel_ref(voxel).voxel_mass() });
	}

	pub fn contains(&self, voxel_type: VoxelTypeId) -> bool {
		self.readers.contains_key(&voxel_type)
	}

	pub fn mass(&self, voxel: &VoxelRef<'_>) -> Option<u64> {
		self.readers.get(&voxel.type_id()).map(|reader| reader(voxel))
	}
}

pub trait VoxelMassAppExt {
	fn register_voxel_mass<T: VoxelMassValue>(&mut self) -> &mut Self;
}

impl VoxelMassAppExt for App {
	fn register_voxel_mass<T: VoxelMassValue>(&mut self) -> &mut Self {
		self.init_resource::<VoxelMassReaders>();
		self.world_mut().resource_mut::<VoxelMassReaders>().register::<T>();
		self
	}
}

/// Computes properties directly from constant-data tree leaves.
pub fn mass_properties_of_voxels(
	readers: &VoxelMassReaders,
	voxels: &Voxels,
	grid_voxel_origin: IVec3,
) -> Option<MassProperties> {
	let reader = *readers.readers.get(&voxels.voxel_type_id())?;
	let mut properties = MassProperties::ZERO;
	let mut first_moment = [0i128; 3];
	for (leaf_origin, leaf_size, voxel) in voxels.grid_tree() {
		let voxel_mass = reader(&voxel);
		if voxel_mass == 0 { continue; }
		let size = u64::from(leaf_size);
		let count = size.checked_pow(3).expect("voxel count overflow");
		let mass = voxel_mass.checked_mul(count).expect("voxel mass overflow");
		properties = properties.replaced_tracking([(MassProperties::ZERO, MassProperties {
			mass: Mass(mass),
			center_of_mass: CenterOfMass(FixedVec3::from(grid_voxel_origin) + FixedVec3::from(leaf_origin) + FixedVec3::splat(Fixed::from_num(size) / Fixed::from_num(2))),
			rotational_inertia: RotationalInertia(InertiaTensor::get_inertia_tensor_for_cube(mass as f64, size as f64)),
		})], &mut first_moment);
	}
	(properties.mass.0 != 0).then_some(properties)
}
