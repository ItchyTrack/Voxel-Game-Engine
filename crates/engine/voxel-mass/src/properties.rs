use bevy::{math::{DMat3, DVec3}, prelude::{Component, Transform, Vec3}};
use serde::{Deserialize, Serialize};

use crate::{CenterOfMass, InertiaTensor, Mass, RotationalInertia};

/// Mass, center of mass, and inertia about that center, all in the same local space.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MassProperties {
	pub mass: Mass,
	pub center_of_mass: CenterOfMass,
	pub rotational_inertia: RotationalInertia,
}

impl MassProperties {
	pub const ZERO: Self = Self {
		mass: Mass(0),
		center_of_mass: CenterOfMass(DVec3::ZERO),
		rotational_inertia: RotationalInertia(InertiaTensor::ZERO),
	};

	pub fn get_transformed(self, transform: &Transform) -> Self {
		assert_eq!(transform.scale, Vec3::ONE, "mass grid transform must have unit scale");
		Self {
			center_of_mass: self.center_of_mass.get_transformed(transform),
			rotational_inertia: RotationalInertia(self.rotational_inertia.0.get_rotated(transform.rotation.as_dquat())),
			..self
		}
	}

	pub fn add(self, other: Self) -> Self {
		Self::sum([self, other])
	}

	pub fn sum(parts: impl IntoIterator<Item = Self, IntoIter: Clone>) -> Self {
		Self::combine(parts.into_iter().map(|part| (part, 1)))
	}

	/// Applies replacements together so source handoffs cannot cause intermediate mass underflow.
	pub fn replaced(self, updates: impl IntoIterator<Item = (Self, Self), IntoIter: Clone>) -> Self {
		Self::combine(std::iter::once((self, 1)).chain(
			updates.into_iter().flat_map(|(before, after)| [(before, -1), (after, 1)]),
		))
	}

	fn combine(parts: impl Iterator<Item = (Self, i128)> + Clone) -> Self {
		let reference = parts.clone().find(|(part, _)| part.mass.0 != 0)
			.map_or(DVec3::ZERO, |(part, _)| part.center_of_mass.0);
		let mut mass = 0i128;
		let mut weighted_offset = DVec3::ZERO;
		for (part, sign) in parts.clone() {
			let signed_mass = i128::from(part.mass.0) * sign;
			mass = mass + signed_mass;
			weighted_offset += (part.center_of_mass.0 - reference) * signed_mass as f64;
		}
		let mass = u64::try_from(mass).expect("mass underflow or overflow");
		if mass == 0 { return Self::ZERO; }
		let center = reference + weighted_offset / mass as f64;
		let mut inertia = InertiaTensor::ZERO;
		for (part, sign) in parts {
			if part.mass.0 == 0 { continue; }
			let shifted = part.rotational_inertia.0.move_from_center_of_mass(
				&(part.center_of_mass.0 - center), part.mass.0 as f64,
			);
			if sign > 0 { inertia += shifted; } else { inertia -= shifted; }
		}
		Self { mass: Mass(mass), center_of_mass: CenterOfMass(center), rotational_inertia: RotationalInertia(inertia) }
	}
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct MassError {
	mass_minus: u64,
	mass_plus: u64,
	first_moment_minus: [u64; 3],
	first_moment_plus: [u64; 3],
}

impl MassError {
	pub const ZERO: Self = Self {
		mass_minus: 0,
		mass_plus: 0,
		first_moment_minus: [0; 3],
		first_moment_plus: [0; 3],
	};

	pub const fn new(
		mass_minus: u64,
		mass_plus: u64,
		first_moment_minus: [u64; 3],
		first_moment_plus: [u64; 3],
	) -> Self {
		Self { mass_minus, mass_plus, first_moment_minus, first_moment_plus }
	}

	pub const fn mass_minus(&self) -> u64 { self.mass_minus }
	pub const fn mass_plus(&self) -> u64 { self.mass_plus }
	pub const fn first_moment_minus(&self) -> [u64; 3] { self.first_moment_minus }
	pub const fn first_moment_plus(&self) -> [u64; 3] { self.first_moment_plus }

	/// Transforms voxel-index first-moment error bounds into parent-space bounds.
	pub fn get_transformed(self, transform: &Transform) -> Self {
		let rotation = DMat3::from_quat(transform.rotation.as_dquat());
		let mass_coefficient = transform.translation.as_dvec3() + rotation * DVec3::splat(0.5);
		let moment_minus = self.first_moment_minus();
		let moment_plus = self.first_moment_plus();
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
				minus = add_up(minus, multiply_up(coefficient, self.mass_minus()));
				plus = add_up(plus, multiply_up(coefficient, self.mass_plus()));
			} else {
				minus = add_up(minus, multiply_up(-coefficient, self.mass_plus()));
				plus = add_up(plus, multiply_up(-coefficient, self.mass_minus()));
			}

			transformed_minus[output_axis] = outward_rounded_error(minus);
			transformed_plus[output_axis] = outward_rounded_error(plus);
		}

		MassError::new(
			self.mass_minus(),
			self.mass_plus(),
			transformed_minus,
			transformed_plus,
		)
	}

	pub fn add(self, other: Self) -> Self {
		Self {
			mass_minus: self.mass_minus + other.mass_minus,
			mass_plus: self.mass_plus + other.mass_plus,
			first_moment_minus: array_add(self.first_moment_minus, other.first_moment_minus),
			first_moment_plus: array_add(self.first_moment_plus, other.first_moment_plus),
		}
	}

	pub fn expand(&mut self, amount: Self) {
		*self = self.add(amount);
	}

	pub fn reduce(&mut self, amount: Self) {
		*self = Self {
			mass_minus: self.mass_minus - amount.mass_minus,
			mass_plus: self.mass_plus - amount.mass_plus,
			first_moment_minus: array_sub(self.first_moment_minus, amount.first_moment_minus),
			first_moment_plus: array_sub(self.first_moment_plus, amount.first_moment_plus),
		};
	}

	pub fn expanded(mut self, amount: Self) -> Self {
		self.expand(amount);
		self
	}

	pub fn reduced(mut self, amount: Self) -> Self {
		self.reduce(amount);
		self
	}
}

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyMassError(pub MassError);

fn array_add(a: [u64; 3], b: [u64; 3]) -> [u64; 3] {
	std::array::from_fn(|axis| a[axis] - b[axis])
}

fn array_sub(a: [u64; 3], b: [u64; 3]) -> [u64; 3] {
	std::array::from_fn(|axis| a[axis] - b[axis])
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
