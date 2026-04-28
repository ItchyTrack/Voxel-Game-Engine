use std::ops::*;

use glam::{DMat3, DQuat, DVec3, Vec3, Vec4};

use crate::{debug_draw};

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

	// pos is relative to the center of mass
	pub fn move_to_center_of_mass(&self, pos: &DVec3, mass: f64) -> InertiaTensor {
		InertiaTensor {
			mat: self.mat - (DMat3::IDENTITY * (pos).length_squared() - self_outer_product(pos)) * mass,
		}
	}

	// pos is relative to the center of mass
	pub fn move_between_points(&self, start_pos_1: &DVec3, end_pos_2: &DVec3, mass: f64) -> InertiaTensor {
		self.move_to_center_of_mass(start_pos_1, mass).move_from_center_of_mass(end_pos_2, mass)
	}

	pub fn get_rotated(&self, rotation :DQuat) -> InertiaTensor {
		InertiaTensor {
			mat: DMat3::from_quat(rotation) * self.mat * DMat3::from_quat(rotation.inverse())
		}
	}

	// The inertia tensor should already be at that pos.
	pub fn render_debug_box(&self, mass: f32, pos: Vec3) {
		use nalgebra;
		let maxtrix = nalgebra::Matrix3::new(
			self.mat.x_axis.x as f32, self.mat.x_axis.y as f32, self.mat.x_axis.z as f32,
			self.mat.y_axis.x as f32, self.mat.y_axis.y as f32, self.mat.y_axis.z as f32,
			self.mat.z_axis.x as f32, self.mat.z_axis.y as f32, self.mat.z_axis.z as f32
		);
		let eigen = nalgebra::SymmetricEigen::new(maxtrix);
		let eigenvalues = (eigen.eigenvalues.x, eigen.eigenvalues.y, eigen.eigenvalues.z);
		let eigenvectors = (
			Vec3::new(eigen.eigenvectors[(0,0)], eigen.eigenvectors[(1,0)], eigen.eigenvectors[(2,0)]),
			Vec3::new(eigen.eigenvectors[(0,1)], eigen.eigenvectors[(1,1)], eigen.eigenvectors[(2,1)]),
			Vec3::new(eigen.eigenvectors[(0,2)], eigen.eigenvectors[(1,2)], eigen.eigenvectors[(2,2)])
		);
		let box_size_vec = (
			eigenvectors.0.normalize() * ((6.0 / mass) * (eigenvalues.1 + eigenvalues.2 - eigenvalues.0)).sqrt(),
			eigenvectors.1.normalize() * ((6.0 / mass) * (eigenvalues.0 + eigenvalues.2 - eigenvalues.1)).sqrt(),
			eigenvectors.2.normalize() * ((6.0 / mass) * (eigenvalues.0 + eigenvalues.1 - eigenvalues.2)).sqrt()
		);
		debug_draw::rectangular_prism_from_vec(
			&(pos + (box_size_vec.0 + box_size_vec.1 + box_size_vec.2) / -2.0),
			&box_size_vec,
			&Vec4::new(1.0, 0.2, 0.2, 0.2),
			true
		);
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
