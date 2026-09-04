use std::{any::TypeId, sync::Arc};

use bevy::prelude::{App, Resource};
use rustc_hash::FxHashMap;
use voxel_sources::edit::GridEdit;
use voxel_trees::region::NonZeroVoxelRegion;

use crate::{MassError, MassProperties, VoxelMassReaders};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MassRange {
	min: u64,
	max: u64,
}

impl MassRange {
	pub const fn new(min: u64, max: u64) -> Self {
		assert!(min <= max, "mass range minimum exceeds maximum");
		Self { min, max }
	}

	pub const fn min(&self) -> u64 { self.min }
	pub const fn max(&self) -> u64 { self.max }
}

pub fn edit_reservation_error(
	current: MassProperties,
	current_error: MassError,
	new_mass: MassRange,
	region: NonZeroVoxelRegion,
) -> MassError {
	let old_mass_max = current.mass().checked_add(current_error.mass_plus()).expect("mass bound overflow");
	let mass_delta_min = i128::from(new_mass.min()) - i128::from(old_mass_max);
	let mass_delta_max = i128::from(new_mass.max());

	let mut moment_minus = [0; 3];
	let mut moment_plus = [0; 3];
	let min = region.min();
	let max = region.end() - bevy::math::IVec3::ONE;
	for axis in 0..3 {
		let (new_min, new_max) = moment_bounds(new_mass, min[axis], max[axis]);
		let (old_min, old_max) = moment_bounds(MassRange::new(0, old_mass_max), min[axis], max[axis]);
		moment_minus[axis] = negative_magnitude(new_min - old_max);
		moment_plus[axis] = positive_magnitude(new_max - old_min);
	}

	MassError::new(
		negative_magnitude(mass_delta_min),
		positive_magnitude(mass_delta_max),
		moment_minus,
		moment_plus,
	)
}

fn moment_bounds(mass: MassRange, coordinate_min: i32, coordinate_max: i32) -> (i128, i128) {
	let coordinate_min = i128::from(coordinate_min);
	let coordinate_max = i128::from(coordinate_max);
	let products = [
		i128::from(mass.min()) * coordinate_min,
		i128::from(mass.min()) * coordinate_max,
		i128::from(mass.max()) * coordinate_min,
		i128::from(mass.max()) * coordinate_max,
	];
	(*products.iter().min().unwrap(), *products.iter().max().unwrap())
}

fn negative_magnitude(value: i128) -> u64 {
	if value >= 0 { 0 } else { u64::try_from(value.checked_neg().expect("bound overflow")).expect("bound overflow") }
}

fn positive_magnitude(value: i128) -> u64 {
	if value <= 0 { 0 } else { u64::try_from(value).expect("bound overflow") }
}

type GridEditMassReader =
	Arc<dyn Fn(&dyn GridEdit, NonZeroVoxelRegion, &VoxelMassReaders) -> MassRange + Send + Sync>;

/// Type-erased bounds for the mass left in an edit's affected region.
#[derive(Resource, Default, Clone)]
pub struct GridEditMassReaders {
	readers: FxHashMap<TypeId, GridEditMassReader>,
}

impl GridEditMassReaders {
	pub fn register<T, F>(&mut self, reader: F)
	where
		T: GridEdit,
		F: Fn(&T, NonZeroVoxelRegion, &VoxelMassReaders) -> MassRange + Send + Sync + 'static,
	{
		self.readers.insert(TypeId::of::<T>(), Arc::new(move |edit, region, mass_readers| {
			reader(
				edit.as_any().downcast_ref::<T>().expect("grid-edit mass reader selected for the wrong edit type"),
				region,
				mass_readers,
			)
		}));
	}

	/// Returns `None` when no bound reader is registered for the concrete edit type.
	pub fn read(
		&self,
		edit: &dyn GridEdit,
		region: NonZeroVoxelRegion,
		mass_readers: &VoxelMassReaders,
	) -> Option<MassRange> {
		self.readers.get(&edit.as_any().type_id()).map(|reader| reader(edit, region, mass_readers))
	}

	pub fn read_typed<T: GridEdit>(
		&self,
		edit: &T,
		region: NonZeroVoxelRegion,
		mass_readers: &VoxelMassReaders,
	) -> Option<MassRange> {
		self.read(edit, region, mass_readers)
	}
}

pub trait GridEditMassAppExt {
	fn register_grid_edit_mass<T, F>(&mut self, reader: F) -> &mut Self
	where
		T: GridEdit,
		F: Fn(&T, NonZeroVoxelRegion, &VoxelMassReaders) -> MassRange + Send + Sync + 'static;
}

impl GridEditMassAppExt for App {
	fn register_grid_edit_mass<T, F>(&mut self, reader: F) -> &mut Self
	where
		T: GridEdit,
		F: Fn(&T, NonZeroVoxelRegion, &VoxelMassReaders) -> MassRange + Send + Sync + 'static,
	{
		self.init_resource::<GridEditMassReaders>();
		self.world_mut().resource_mut::<GridEditMassReaders>().register::<T, F>(reader);
		self
	}
}
