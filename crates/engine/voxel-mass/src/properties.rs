use bevy::{math::{DMat3, Vec3}, prelude::Component};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AddRemove<T> {
	Add(T),
	Remove(T),
}

impl AddRemove<u64> {
	pub const fn amount(self) -> u64 {
		match self {
			Self::Add(amount) | Self::Remove(amount) => amount,
		}
	}

	pub fn checked_add(self, other: Self) -> Self {
		match (self, other) {
			(Self::Add(a), Self::Add(b)) => Self::Add(a.checked_add(b).expect("mass delta overflow")),
			(Self::Remove(a), Self::Remove(b)) => Self::Remove(a.checked_add(b).expect("mass delta overflow")),
			(Self::Add(a), Self::Remove(b)) if a >= b => Self::Add(a - b),
			(Self::Add(a), Self::Remove(b)) => Self::Remove(b - a),
			(Self::Remove(a), Self::Add(b)) if a >= b => Self::Remove(a - b),
			(Self::Remove(a), Self::Add(b)) => Self::Add(b - a),
		}
	}
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
	mass: u64,
	first_moment: [i64; 3],
	inertia_at_origin: DMat3,
}

impl Default for MassProperties {
	fn default() -> Self { Self::ZERO }
}

impl MassProperties {
	pub const ZERO: Self = Self {
		mass: 0,
		first_moment: [0; 3],
		inertia_at_origin: DMat3::ZERO,
	};

	pub const fn new(mass: u64, first_moment: [i64; 3], inertia_at_origin: DMat3) -> Self {
		Self { mass, first_moment, inertia_at_origin }
	}

	pub const fn mass(&self) -> u64 { self.mass }
	pub const fn first_moment(&self) -> [i64; 3] { self.first_moment }
	pub const fn inertia_at_origin(&self) -> DMat3 { self.inertia_at_origin }

	/// Returns `None` for a massless value.
	pub fn center_of_mass(&self) -> Option<Vec3> {
		(self.mass != 0).then(|| {
			let inverse_mass = 1.0 / self.mass as f32;
			Vec3::new(
				self.first_moment[0] as f32 * inverse_mass,
				self.first_moment[1] as f32 * inverse_mass,
				self.first_moment[2] as f32 * inverse_mass,
			) + Vec3::splat(0.5)
		})
	}

	pub fn checked_apply(&mut self, delta: MassDelta) {
		let mass = match delta.mass {
			AddRemove::Add(amount) => self.mass.checked_add(amount).expect("mass overflow"),
			AddRemove::Remove(amount) => self.mass.checked_sub(amount).expect("mass underflow"),
		};
		let first_moment = std::array::from_fn(|axis| {
			self.first_moment[axis].checked_add(delta.first_moment[axis]).expect("first-moment overflow")
		});
		self.mass = mass;
		self.first_moment = first_moment;
		self.inertia_at_origin += delta.inertia_at_origin;
	}

	pub fn checked_applied(mut self, delta: MassDelta) -> Self {
		self.checked_apply(delta);
		self
	}

	pub fn checked_difference(&self, previous: &Self) -> MassDelta {
		MassDelta::checked_difference(*self, *previous)
	}
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassDelta {
	mass: AddRemove<u64>,
	first_moment: [i64; 3],
	inertia_at_origin: DMat3,
}

impl Default for MassDelta {
	fn default() -> Self { Self::ZERO }
}

impl MassDelta {
	pub const ZERO: Self = Self {
		mass: AddRemove::Add(0),
		first_moment: [0; 3],
		inertia_at_origin: DMat3::ZERO,
	};

	pub const fn new(mass: AddRemove<u64>, first_moment: [i64; 3], inertia_at_origin: DMat3) -> Self {
		Self { mass, first_moment, inertia_at_origin }
	}

	pub const fn mass(&self) -> AddRemove<u64> { self.mass }
	pub const fn first_moment(&self) -> [i64; 3] { self.first_moment }
	pub const fn inertia_at_origin(&self) -> DMat3 { self.inertia_at_origin }

	pub fn checked_difference(new: MassProperties, old: MassProperties) -> Self {
		let mass = if new.mass >= old.mass {
			AddRemove::Add(new.mass - old.mass)
		} else {
			AddRemove::Remove(old.mass - new.mass)
		};
		let first_moment = std::array::from_fn(|axis| {
			new.first_moment[axis].checked_sub(old.first_moment[axis]).expect("first-moment delta overflow")
		});
		Self {
			mass,
			first_moment,
			inertia_at_origin: new.inertia_at_origin - old.inertia_at_origin,
		}
	}

	pub fn checked_add(self, other: Self) -> Self {
		Self::checked_sum([self, other])
	}

	pub fn checked_sum(deltas: impl IntoIterator<Item = Self>) -> Self {
		let mut added_mass = 0u128;
		let mut removed_mass = 0u128;
		let mut first_moment = [0i128; 3];
		let mut inertia_at_origin = DMat3::ZERO;
		for delta in deltas {
			match delta.mass {
				AddRemove::Add(value) => added_mass = added_mass.checked_add(u128::from(value)).expect("mass delta overflow"),
				AddRemove::Remove(value) => removed_mass = removed_mass.checked_add(u128::from(value)).expect("mass delta overflow"),
			}
			for axis in 0..3 {
				first_moment[axis] = first_moment[axis]
					.checked_add(i128::from(delta.first_moment[axis]))
					.expect("first-moment delta overflow");
			}
			inertia_at_origin += delta.inertia_at_origin;
		}
		let mass = if added_mass >= removed_mass {
			AddRemove::Add(u64::try_from(added_mass - removed_mass).expect("mass delta overflow"))
		} else {
			AddRemove::Remove(u64::try_from(removed_mass - added_mass).expect("mass delta overflow"))
		};
		Self {
			mass,
			first_moment: first_moment.map(|value| i64::try_from(value).expect("first-moment delta overflow")),
			inertia_at_origin,
		}
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

	pub fn checked_add(self, other: Self) -> Self {
		Self {
			mass_minus: self.mass_minus.checked_add(other.mass_minus).expect("mass error overflow"),
			mass_plus: self.mass_plus.checked_add(other.mass_plus).expect("mass error overflow"),
			first_moment_minus: checked_array_add(self.first_moment_minus, other.first_moment_minus),
			first_moment_plus: checked_array_add(self.first_moment_plus, other.first_moment_plus),
		}
	}

	pub fn checked_expand(&mut self, amount: Self) {
		*self = self.checked_add(amount);
	}

	pub fn checked_reduce(&mut self, amount: Self) {
		*self = Self {
			mass_minus: self.mass_minus.checked_sub(amount.mass_minus).expect("mass error underflow"),
			mass_plus: self.mass_plus.checked_sub(amount.mass_plus).expect("mass error underflow"),
			first_moment_minus: checked_array_sub(self.first_moment_minus, amount.first_moment_minus),
			first_moment_plus: checked_array_sub(self.first_moment_plus, amount.first_moment_plus),
		};
	}

	pub fn checked_expanded(mut self, amount: Self) -> Self {
		self.checked_expand(amount);
		self
	}

	pub fn checked_reduced(mut self, amount: Self) -> Self {
		self.checked_reduce(amount);
		self
	}
}

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BodyMassError(pub MassError);

fn checked_array_add(a: [u64; 3], b: [u64; 3]) -> [u64; 3] {
	std::array::from_fn(|axis| a[axis].checked_add(b[axis]).expect("first-moment error overflow"))
}

fn checked_array_sub(a: [u64; 3], b: [u64; 3]) -> [u64; 3] {
	std::array::from_fn(|axis| a[axis].checked_sub(b[axis]).expect("first-moment error underflow"))
}
