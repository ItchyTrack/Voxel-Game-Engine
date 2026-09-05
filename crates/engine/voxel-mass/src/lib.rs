mod body;
mod components;
mod edit;
mod grid;
mod inertia_tensor;
mod marker;
mod properties;
mod source;
mod voxel;

pub use components::{BodyMassInitialized, CenterOfMass, Mass, RotationalInertia, VoxelMass};
pub use edit::{GridEditMassAppExt, GridEditMassReaders, MassRange, edit_reservation_error};
pub use grid::{GridMassProperties, SourceMassChange, apply_source_mass_changes};
pub use inertia_tensor::InertiaTensor;
pub use marker::MarkerGridType;
pub use properties::{BodyMassError, MassError, MassProperties};
pub use source::SourceMassState;
pub use voxel::{VoxelMassAppExt, VoxelMassReaders, VoxelMassValue, mass_properties_of_voxels};

use bevy::prelude::*;
use voxel_data::{body::Body, grid::Grid};

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
			.register_required_components::<Body, Mass>()
			.register_required_components::<Body, CenterOfMass>()
			.register_required_components::<Body, RotationalInertia>()
			.register_required_components::<Body, BodyMassError>()
			.register_required_components::<Body, BodyMassInitialized>()
			.register_required_components::<Grid, GridMassProperties>()
			.init_resource::<VoxelMassReaders>()
			.init_resource::<GridEditMassReaders>()
			.add_message::<SourceMassChange>()
			.configure_sets(
				FixedUpdate,
				(VoxelMassSet::SourceDrain, VoxelMassSet::ApplySourceChanges, VoxelMassSet::BodyAggregation).chain(),
			)
			.add_systems(FixedUpdate, apply_source_mass_changes.in_set(VoxelMassSet::ApplySourceChanges))
			.add_systems(FixedUpdate, body::aggregate_body_mass_properties.in_set(VoxelMassSet::BodyAggregation));
	}
}
