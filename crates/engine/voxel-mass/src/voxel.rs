use bevy::math::{DMat3, IVec3};
use bevy::prelude::{App, Resource};
use rustc_hash::FxHashMap;
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId, Voxels};

use crate::MassProperties;

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
	let mut mass = 0u64;
	let mut first_moment = [0i64; 3];
	let mut inertia_at_origin = DMat3::ZERO;

	for (leaf_origin, leaf_size, voxel) in voxels.grid_tree() {
		let voxel_mass = reader(&voxel);
		if voxel_mass == 0 { continue; }

		let size = u64::from(leaf_size);
		let count = size.checked_mul(size).and_then(|n| n.checked_mul(size)).expect("voxel count overflow");
		mass = mass.checked_add(voxel_mass.checked_mul(count).expect("mass overflow")).expect("mass overflow");

		let q0 = [
			i64::from(grid_voxel_origin.x) + i64::from(leaf_origin.x),
			i64::from(grid_voxel_origin.y) + i64::from(leaf_origin.y),
			i64::from(grid_voxel_origin.z) + i64::from(leaf_origin.z),
		];
		let line_index_sum = size.checked_mul(size.checked_sub(1).unwrap()).expect("coordinate sum overflow") / 2;
		let plane_count = size.checked_mul(size).expect("voxel count overflow");
		for axis in 0..3 {
			let base_sum = i128::from(q0[axis]).checked_mul(i128::from(count)).expect("coordinate sum overflow");
			let offset_sum = i128::from(line_index_sum).checked_mul(i128::from(plane_count)).expect("coordinate sum overflow");
			let coordinate_sum = base_sum.checked_add(offset_sum).expect("coordinate sum overflow");
			let contribution = coordinate_sum.checked_mul(i128::from(voxel_mass)).expect("first-moment overflow");
			let contribution = i64::try_from(contribution).expect("first-moment overflow");
			first_moment[axis] = first_moment[axis].checked_add(contribution).expect("first-moment overflow");
		}

		inertia_at_origin += cube_run_inertia(q0, leaf_size, voxel_mass);
	}

	(mass != 0).then(|| MassProperties::new(mass, first_moment, inertia_at_origin))
}

fn cube_run_inertia(q0: [i64; 3], size: u32, voxel_mass: u64) -> DMat3 {
	let n = f64::from(size);
	let count = n * n * n;
	let index_sum = n * (n - 1.0) * 0.5;
	let index_square_sum = n * (n - 1.0) * (2.0 * n - 1.0) / 6.0;
	let mut line_sum = [0.0; 3];
	let mut square_sum = [0.0; 3];

	for axis in 0..3 {
		let start = q0[axis] as f64 + 0.5;
		line_sum[axis] = n * start + index_sum;
		let line_square_sum = n * start * start + 2.0 * start * index_sum + index_square_sum;
		square_sum[axis] = line_square_sum * n * n;
	}

	let xy = line_sum[0] * line_sum[1] * n;
	let xz = line_sum[0] * line_sum[2] * n;
	let yz = line_sum[1] * line_sum[2] * n;
	let m = voxel_mass as f64;
	let cube_center_inertia = count / 6.0;
	let xx = m * (square_sum[1] + square_sum[2] + cube_center_inertia);
	let yy = m * (square_sum[0] + square_sum[2] + cube_center_inertia);
	let zz = m * (square_sum[0] + square_sum[1] + cube_center_inertia);

	DMat3::from_cols_array(&[
		xx, -m * xy, -m * xz,
		-m * xy, yy, -m * yz,
		-m * xz, -m * yz, zz,
	])
}
