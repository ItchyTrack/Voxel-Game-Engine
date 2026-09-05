use bevy::prelude::*;
use voxel_mass::{BodyMassError, CenterOfMass, GridMassProperties, Mass, MassError, MassProperties, RotationalInertia, VoxelMass, VoxelMassAuthority};

use crate::{IsStatic, RigidBody};

pub fn aggregate_body_mass_properties(
	authority: Res<VoxelMassAuthority>,
	mut ready: ResMut<crate::PhysicsMassReady>,
	mut bodies: Query<(
		Option<&Children>,
		&mut Mass,
		&mut CenterOfMass,
		&mut RotationalInertia,
		&mut BodyMassError,
		Has<IsStatic>,
	), With<RigidBody>>,
	grids: Query<(&Transform, &GridMassProperties, Has<VoxelMass>)>,
) {
	let mut all_ready = true;
	if !authority.0 {
		for (_, body_mass, _, _, body_error, is_static) in &mut bodies {
			if !is_static {
				all_ready &= error_is_ready(*body_mass, body_error.0);
			}
		}
		ready.0 = all_ready;
		return;
	}

	for (children, mut body_mass, mut body_center, mut body_inertia, mut body_error, is_static) in &mut bodies {
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

		if !is_static {
			all_ready &= initialized && error_is_ready(properties.mass, error);
		}
	}

	ready.0 = all_ready;
}

fn error_is_ready(mass: Mass, error: MassError) -> bool {
	let maximum_error = error.mass_minus().max(error.mass_plus());
	u128::from(maximum_error) * 100 <= u128::from(mass.0)
}
