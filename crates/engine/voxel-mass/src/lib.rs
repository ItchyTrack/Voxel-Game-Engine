mod components;
mod edit;
mod grid;
mod inertia_tensor;
mod marker;
mod properties;
mod source;
mod voxel;

pub use components::{CenterOfMass, Mass, RotationalInertia, VoxelMass};
pub use edit::{GridEditMassAppExt, GridEditMassReaders, MassRange, edit_reservation_error};
pub use grid::{GridMassProperties, SourceMassChange, apply_source_mass_changes};
pub use inertia_tensor::InertiaTensor;
pub use marker::MarkerGridType;
pub use properties::{BodyMassError, MassError, MassProperties};
pub use source::SourceMassState;
pub use voxel::{VoxelMassAppExt, VoxelMassReaders, VoxelMassValue, mass_properties_of_voxels};

use bevy::prelude::*;
use voxel_data::grid::Grid;

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VoxelMassSet {
	SourceDrain,
	ApplySourceChanges,
	BodyAggregation,
}

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoxelMassAuthority(pub bool);

impl Default for VoxelMassAuthority {
	fn default() -> Self { Self(true) }
}

pub struct VoxelMassPlugin {
	pub authoritative: bool,
}

impl Default for VoxelMassPlugin {
	fn default() -> Self { Self { authoritative: true } }
}

impl Plugin for VoxelMassPlugin {
	fn build(&self, app: &mut App) {
		app.insert_resource(VoxelMassAuthority(self.authoritative))
			.init_resource::<VoxelMassReaders>()
			.init_resource::<GridEditMassReaders>()
			.add_message::<SourceMassChange>()
			.configure_sets(
				FixedUpdate,
				(VoxelMassSet::SourceDrain, VoxelMassSet::ApplySourceChanges, VoxelMassSet::BodyAggregation).chain(),
			)
			.add_systems(FixedUpdate, apply_source_mass_changes.in_set(VoxelMassSet::ApplySourceChanges))
			.add_systems(Update, add_grid_mass_properties);
	}
}

fn add_grid_mass_properties(
	mut commands: Commands,
	grids: Query<Entity, (Added<Grid>, Without<GridMassProperties>)>,
) {
	for grid in &grids {
		commands.entity(grid).insert(GridMassProperties::default());
	}
}
