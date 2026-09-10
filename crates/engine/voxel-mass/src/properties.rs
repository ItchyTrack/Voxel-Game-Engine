use bevy::prelude::Component;
use voxel_math::{Fixed, FixedVec3};
use voxel_transform::{Scale, Transform};
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
		center_of_mass: CenterOfMass(FixedVec3::ZERO),
		rotational_inertia: RotationalInertia(InertiaTensor::ZERO),
	};

	pub fn get_transformed(self, transform: &Transform) -> Self {
		assert_eq!(transform.scale, Scale::ONE, "mass grid transform must have unit scale");
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
		Self::combine(parts.into_iter().map(|part| (part, 1)), None)
	}

	/// Applies replacements together so source handoffs cannot cause intermediate mass underflow.
	pub fn replaced(self, updates: impl IntoIterator<Item = (Self, Self), IntoIter: Clone>) -> Self {
		Self::combine(std::iter::once((self, 1)).chain(
			updates.into_iter().flat_map(|(before, after)| [(before, -1), (after, 1)]),
		), None)
	}

	pub(crate) fn first_moment(self) -> [i128; 3] {
		self.center_of_mass.0.to_array().map(|value| fixed_units(value).checked_mul(i128::from(self.mass.0)).expect("first moment overflow"))
	}

	pub(crate) fn replaced_tracking(
		self,
		updates: impl IntoIterator<Item = (Self, Self), IntoIter: Clone>,
		first_moment: &mut [i128; 3],
	) -> Self {
		let updates = updates.into_iter();
		let mut next_moment = *first_moment;
		for (before, after) in updates.clone() {
			let before = before.first_moment();
			let after = after.first_moment();
			for axis in 0..3 {
				next_moment[axis] = next_moment[axis].checked_add(after[axis].checked_sub(before[axis]).expect("first moment overflow"))
					.expect("first moment overflow");
			}
		}
		let result = Self::combine(std::iter::once((self, 1)).chain(
			updates.flat_map(|(before, after)| [(before, -1), (after, 1)]),
		), Some(next_moment));
		*first_moment = if result.mass.0 == 0 { [0; 3] } else { next_moment };
		result
	}

	fn combine(parts: impl Iterator<Item = (Self, i128)> + Clone, tracked_moment: Option<[i128; 3]>) -> Self {
		let mut mass = 0i128;
		let mut first_moment = [0i128; 3];
		for (part, sign) in parts.clone() {
			let signed_mass = i128::from(part.mass.0) * sign;
			mass = mass.checked_add(signed_mass).expect("mass overflow");
			if tracked_moment.is_none() {
				for axis in 0..3 {
					let moment = fixed_units(part.center_of_mass.0[axis]).checked_mul(signed_mass).expect("first moment overflow");
					first_moment[axis] = first_moment[axis].checked_add(moment).expect("first moment overflow");
				}
			}
		}
		let mass = u64::try_from(mass).expect("mass underflow or overflow");
		if mass == 0 { return Self::ZERO; }
		let first_moment = tracked_moment.unwrap_or(first_moment);
		let center = FixedVec3::from_array(first_moment.map(|moment| center_component(moment, mass)));
		let mut inertia = InertiaTensor::ZERO;
		for (part, sign) in parts {
			if part.mass.0 == 0 { continue; }
			let shifted = part.rotational_inertia.0.move_from_center_of_mass(
				&(part.center_of_mass.0 - center).as_dvec3(), part.mass.0 as f64,
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
		assert_eq!(transform.scale, Scale::ONE, "mass transform must have unit scale");
		let rotation = [transform.rotation * FixedVec3::X, transform.rotation * FixedVec3::Y, transform.rotation * FixedVec3::Z];
		let mass_coefficient = transform.transform_point(FixedVec3::splat(Fixed::ONE / Fixed::from_num(2)));
		let moment_minus = self.first_moment_minus();
		let moment_plus = self.first_moment_plus();
		let mut transformed_minus = [0; 3];
		let mut transformed_plus = [0; 3];

		for output_axis in 0..3 {
			let mut minus = 0u128;
			let mut plus = 0u128;
			for input_axis in 0..3 {
				let coefficient = rotation[input_axis][output_axis];
				if coefficient >= Fixed::ZERO {
					minus = add_up(minus, multiply_up(coefficient, moment_minus[input_axis]));
					plus = add_up(plus, multiply_up(coefficient, moment_plus[input_axis]));
				} else {
					minus = add_up(minus, multiply_up(-coefficient, moment_plus[input_axis]));
					plus = add_up(plus, multiply_up(-coefficient, moment_minus[input_axis]));
				}
			}

			let coefficient = mass_coefficient[output_axis];
			if coefficient >= Fixed::ZERO {
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
			mass_minus: self.mass_minus.checked_add(other.mass_minus).expect("mass error overflow"),
			mass_plus: self.mass_plus.checked_add(other.mass_plus).expect("mass error overflow"),
			first_moment_minus: array_add(self.first_moment_minus, other.first_moment_minus),
			first_moment_plus: array_add(self.first_moment_plus, other.first_moment_plus),
		}
	}

	pub fn expand(&mut self, amount: Self) {
		*self = self.add(amount);
	}

	pub fn reduce(&mut self, amount: Self) {
		*self = Self {
			mass_minus: self.mass_minus.checked_sub(amount.mass_minus).expect("mass error underflow"),
			mass_plus: self.mass_plus.checked_sub(amount.mass_plus).expect("mass error underflow"),
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
	std::array::from_fn(|axis| a[axis].checked_add(b[axis]).expect("mass error overflow"))
}

fn array_sub(a: [u64; 3], b: [u64; 3]) -> [u64; 3] {
	std::array::from_fn(|axis| a[axis].checked_sub(b[axis]).expect("mass error underflow"))
}

#[cfg(test)]
#[path = "properties_tests.rs"]
mod tests;

const UNITS_PER_ONE: i128 = 1 << Fixed::FRAC_BITS;

fn fixed_units(value: Fixed) -> i128 {
	i128::from(value.to_bits())
}

fn center_component(moment: i128, mass: u64) -> Fixed {
	let divisor = i128::from(mass);
	let quotient = moment / divisor;
	let remainder = (moment % divisor).unsigned_abs();
	let round_up = remainder * 2 > mass as u128 || (remainder * 2 == mass as u128 && quotient & 1 != 0);
	let rounded = quotient + if round_up { moment.signum() } else { 0 };
	Fixed::from_bits(i64::try_from(rounded).expect("center of mass overflow"))
}

fn multiply_up(coefficient: Fixed, value: u64) -> u128 {
	u128::try_from(fixed_units(coefficient)).expect("negative error coefficient")
		.checked_mul(u128::from(value)).expect("body mass error overflow")
}

fn add_up(a: u128, b: u128) -> u128 {
	a.checked_add(b).expect("body mass error overflow")
}

fn outward_rounded_error(value: u128) -> u64 {
	u64::try_from(value.div_ceil(UNITS_PER_ONE as u128)).expect("body mass error overflow")
}
