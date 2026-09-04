use bevy::{math::{DMat3, DVec3}, prelude::*};
use voxel_mass::{BodyMassError, GridMassProperties, MassError, MassProperties, VoxelMassAuthority};

use crate::{CenterOfMass, IsStatic, Mass, RigidBody, RotationalInertia, components::VoxelMass, inertia_tensor::InertiaTensor};

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
				all_ready &= error_is_ready(body_mass.0 as u64, body_error.0);
			}
		}
		ready.0 = all_ready;
		return;
	}

	for (children, mut body_mass, mut body_center, mut body_inertia, mut body_error, is_static) in &mut bodies {
		let mut mass = 0u64;
		let mut weighted_center = DVec3::ZERO;
		let mut error = MassError::ZERO;
		let mut initialized = true;

		for child in children.into_iter().flatten() {
			let Ok((transform, properties, requires_mass)) = grids.get(*child) else { continue };
			initialized &= !requires_mass || properties.is_initialized();
			assert_eq!(transform.scale, Vec3::ONE, "mass grid transform must have unit scale");

			let nominal = properties.nominal();
			mass = mass.checked_add(nominal.mass()).expect("body mass overflow");
			if let Some(grid_center) = grid_center(nominal) {
				weighted_center += transform_point(transform, grid_center) * nominal.mass() as f64;
			}
			error = error.checked_add(transform_error(properties.aggregate_error(), transform));
		}

		let center = if mass == 0 { DVec3::ZERO } else { weighted_center / mass as f64 };
		let mut inertia = InertiaTensor::ZERO;
		if mass != 0 {
			for child in children.into_iter().flatten() {
				let Ok((transform, properties, _)) = grids.get(*child) else { continue };
				let nominal = properties.nominal();
				let grid_mass = nominal.mass();
				let Some(grid_center) = grid_center(nominal) else { continue };
				let body_grid_center = transform_point(transform, grid_center);
				let central_inertia = InertiaTensor::from_mat3(nominal.inertia_at_origin())
					.move_to_center_of_mass(&grid_center, grid_mass as f64)
					.get_rotated(transform.rotation.as_dquat());
				inertia += central_inertia.move_from_center_of_mass(&(body_grid_center - center), grid_mass as f64);
			}
		}

		body_mass.set_if_neq(Mass(mass as f32));
		body_center.set_if_neq(CenterOfMass(center.as_vec3()));
		body_inertia.set_if_neq(RotationalInertia(inertia));
		body_error.set_if_neq(BodyMassError(error));

		if !is_static {
			all_ready &= initialized && error_is_ready(mass, error);
		}
	}

	ready.0 = all_ready;
}

fn error_is_ready(mass: u64, error: MassError) -> bool {
	let maximum_error = error.mass_minus().max(error.mass_plus());
	u128::from(maximum_error) * 100 <= u128::from(mass)
}

fn grid_center(properties: &MassProperties) -> Option<DVec3> {
	(properties.mass() != 0).then(|| {
		let moment = properties.first_moment();
		DVec3::new(moment[0] as f64, moment[1] as f64, moment[2] as f64) / properties.mass() as f64
			+ DVec3::splat(0.5)
	})
}

fn transform_point(transform: &Transform, point: DVec3) -> DVec3 {
	transform.rotation.as_dquat() * point + transform.translation.as_dvec3()
}

fn transform_error(error: MassError, transform: &Transform) -> MassError {
	let rotation = DMat3::from_quat(transform.rotation.as_dquat());
	let mass_coefficient = transform.translation.as_dvec3() + rotation * DVec3::splat(0.5);
	let moment_minus = error.first_moment_minus();
	let moment_plus = error.first_moment_plus();
	let mut transformed_minus = [0; 3];
	let mut transformed_plus = [0; 3];

	for output_axis in 0..3 {
		let mut minus = 0.0;
		let mut plus = 0.0;
		for input_axis in 0..3 {
			let coefficient = rotation.col(input_axis)[output_axis];
			if coefficient >= 0.0 {
				minus = add_up(minus, multiply_up(coefficient, moment_minus[input_axis]));
				plus = add_up(plus, multiply_up(coefficient, moment_plus[input_axis]));
			} else {
				minus = add_up(minus, multiply_up(-coefficient, moment_plus[input_axis]));
				plus = add_up(plus, multiply_up(-coefficient, moment_minus[input_axis]));
			}
		}

		let coefficient = mass_coefficient[output_axis];
		if coefficient >= 0.0 {
			minus = add_up(minus, multiply_up(coefficient, error.mass_minus()));
			plus = add_up(plus, multiply_up(coefficient, error.mass_plus()));
		} else {
			minus = add_up(minus, multiply_up(-coefficient, error.mass_plus()));
			plus = add_up(plus, multiply_up(-coefficient, error.mass_minus()));
		}

		transformed_minus[output_axis] = outward_rounded_error(minus);
		transformed_plus[output_axis] = outward_rounded_error(plus);
	}

	MassError::new(
		error.mass_minus(),
		error.mass_plus(),
		transformed_minus,
		transformed_plus,
	)
}

fn multiply_up(coefficient: f64, value: u64) -> f64 {
	if coefficient == 0.0 || value == 0 { return 0.0 }
	let value = value as f64;
	(coefficient * value.next_up()).next_up()
}

fn add_up(a: f64, b: f64) -> f64 {
	if b == 0.0 { a } else { (a + b).next_up() }
}

fn outward_rounded_error(value: f64) -> u64 {
	const U64_UPPER_BOUND: f64 = 18_446_744_073_709_551_616.0;
	let rounded = value.ceil();
	assert!(rounded.is_finite() && rounded >= 0.0 && rounded < U64_UPPER_BOUND, "body mass error overflow");
	rounded as u64
}
