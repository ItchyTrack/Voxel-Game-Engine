use std::ops::*;

use glam::{DMat3, DQuat, DVec3};

#[derive(Copy, Clone, PartialEq)]
pub struct InertiaTensor {
	pub mat: DMat3
}

fn self_outer_product(vec: &DVec3) -> DMat3 {
	DMat3::from_cols(
		vec * vec.x,
		vec * vec.y,
		vec * vec.z
	)
}

impl InertiaTensor {
	pub const ZERO: Self = Self::from_mat3(DMat3::ZERO);

	pub const fn from_mat3(mat: DMat3) -> Self {
		Self {
			mat: mat
		}
	}

	pub fn get_inertia_tensor_for_cube(mass: f64, size: f64) -> InertiaTensor {
		InertiaTensor {
			mat: DMat3::IDENTITY * mass * size * size / 6.0
		}
	}

	pub fn get_inertia_tensor_for_cube_at_pos(mass: f64, size: f64, cube_pos: &DVec3) -> InertiaTensor {
		Self::get_inertia_tensor_for_cube(mass, size).move_from_center_of_mass(cube_pos, mass)
	}

	pub fn move_from_center_of_mass(&self, center_of_mass: &DVec3, mass: f64) -> InertiaTensor {
		InertiaTensor {
			mat: self.mat + (DMat3::IDENTITY * center_of_mass.length_squared() - self_outer_product(center_of_mass)) * mass,
		}
	}

	pub fn move_to_center_of_mass(&self, center_of_mass: &DVec3, mass: f64) -> InertiaTensor {
		InertiaTensor {
			mat: self.mat - (DMat3::IDENTITY * (center_of_mass).length_squared() - self_outer_product(center_of_mass)) * mass,
		}
	}

	pub fn move_between_points(&self, center_of_mass_1: &DVec3, center_of_mass_2: &DVec3, mass: f64) -> InertiaTensor {
		self.move_to_center_of_mass(center_of_mass_1, mass).move_from_center_of_mass(center_of_mass_2, mass)
	}

	pub fn get_rotated(&self, rotation :DQuat) -> InertiaTensor {
		InertiaTensor {
			mat: DMat3::from_quat(rotation) * self.mat * DMat3::from_quat(rotation.inverse())
		}
	}
}

impl Add<Self> for InertiaTensor {
	type Output = InertiaTensor;
	#[inline]
	fn add(self, rhs: Self) -> Self::Output {
		InertiaTensor { mat: self.mat + rhs.mat }
	}
}

impl Add<&Self> for InertiaTensor {
	type Output = Self;
	#[inline]
	fn add(self, rhs: &Self) -> Self::Output {
		self.add(*rhs)
	}
}

impl Add<Self> for &InertiaTensor {
	type Output = InertiaTensor;
	#[inline]
	fn add(self, rhs: Self) -> Self::Output {
		(*self).add(rhs)
	}
}

impl Add<&Self> for &InertiaTensor {
	type Output = InertiaTensor;
	#[inline]
	fn add(self, rhs: &Self) -> Self::Output {
		(*self).add(*rhs)
	}
}

impl AddAssign<Self> for InertiaTensor {
	#[inline]
	fn add_assign(&mut self, rhs: Self) {
		self.mat.add_assign(rhs.mat);
	}
}

impl AddAssign<&Self> for InertiaTensor {
	#[inline]
	fn add_assign(&mut self, rhs: &Self) {
		self.add_assign(*rhs);
	}
}

impl Sub<Self> for InertiaTensor {
	type Output = InertiaTensor;
	#[inline]
	fn sub(self, rhs: Self) -> Self::Output {
		InertiaTensor { mat: self.mat - rhs.mat }
	}
}

impl Sub<&Self> for InertiaTensor {
	type Output = Self;
	#[inline]
	fn sub(self, rhs: &Self) -> Self::Output {
		self.sub(*rhs)
	}
}

impl Sub<Self> for &InertiaTensor {
	type Output = InertiaTensor;
	#[inline]
	fn sub(self, rhs: Self) -> Self::Output {
		(*self).sub(rhs)
	}
}

impl Sub<&Self> for &InertiaTensor {
	type Output = InertiaTensor;
	#[inline]
	fn sub(self, rhs: &Self) -> Self::Output {
		(*self).sub(*rhs)
	}
}

impl SubAssign<Self> for InertiaTensor {
	#[inline]
	fn sub_assign(&mut self, rhs: Self) {
		self.mat.sub_assign(rhs.mat);
	}
}

impl SubAssign<&Self> for InertiaTensor {
	#[inline]
	fn sub_assign(&mut self, rhs: &Self) {
		self.sub_assign(*rhs);
	}
}
