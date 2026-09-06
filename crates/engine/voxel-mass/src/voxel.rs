use bevy::math::{DVec3, IVec3};
use bevy::prelude::{App, Resource};
use rustc_hash::FxHashMap;
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId, Voxels};

use crate::{CenterOfMass, InertiaTensor, Mass, MassProperties, RotationalInertia};

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
	for (leaf_origin, leaf_size, voxel) in voxels.grid_tree() {
		let voxel_mass = reader(&voxel);
		if voxel_mass == 0 { continue; }
		let size = u64::from(leaf_size);
		let count = size * size * size;
		let mass = voxel_mass * count;
		properties = properties.add(MassProperties {
			mass: Mass(mass),
			center_of_mass: CenterOfMass(grid_voxel_origin.as_dvec3() + leaf_origin.as_dvec3() + DVec3::splat(size as f64 * 0.5)),
			rotational_inertia: RotationalInertia(InertiaTensor::get_inertia_tensor_for_cube(mass as f64, size as f64)),
		});
	}
	(properties.mass.0 != 0).then_some(properties)
}
