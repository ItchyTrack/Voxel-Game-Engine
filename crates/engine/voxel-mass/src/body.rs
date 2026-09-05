use bevy::prelude::*;
use voxel_data::{body::Body, grid::Grid};

use crate::{BodyMassError, BodyMassInitialized, CenterOfMass, GridMassProperties, Mass, MassError, MassProperties, RotationalInertia, VoxelMass, VoxelMassAuthority};

pub fn aggregate_body_mass_properties(
	authority: Res<VoxelMassAuthority>,
	mut bodies: Query<(
		Option<&Children>,
		&mut Mass,
		&mut CenterOfMass,
		&mut RotationalInertia,
		&mut BodyMassError,
		&mut BodyMassInitialized,
	), With<Body>>,
	grids: Query<(&Transform, &GridMassProperties, Has<VoxelMass>), With<Grid>>,
) {
	if !authority.0 { return; }

	for (children, mut body_mass, mut body_center, mut body_inertia, mut body_error, mut body_initialized) in &mut bodies {
		let parts = children.into_iter().flatten().filter_map(|child| grids.get(*child).ok());
		let mut error = MassError::ZERO;
		let mut initialized = true;
		for (transform, properties, requires_mass) in parts.clone() {
			initialized &= !requires_mass || properties.is_initialized();
			error = error.checked_add(properties.aggregate_error().get_transformed(transform));
		}
		let properties = MassProperties::checked_sum(
			parts.map(|(transform, properties, _)| properties.nominal().get_transformed(transform)),
		);

		body_mass.set_if_neq(properties.mass);
		body_center.set_if_neq(properties.center_of_mass);
		body_inertia.set_if_neq(properties.rotational_inertia);
		body_error.set_if_neq(BodyMassError(error));
		body_initialized.set_if_neq(BodyMassInitialized(initialized));
	}
}
