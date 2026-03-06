use std::{ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign}};
use core::fmt;
use glam::{Mat3, Quat, Vec3};

pub fn add_vec_to_quat(q: &Quat, dx: &Vec3) -> Quat { (q + (Quat::from_xyzw(dx.x, dx.y, dx.z, 0.0) * q) * 0.5).normalize() }
pub fn sub_quat(q1: &Quat, q2: &Quat) -> Vec3 { (q1 * q2.inverse()).xyz() * 2.0 } //    return (a * inverse(b)).vec() * 2.0f;

#[derive(Copy, Clone, PartialEq)]
pub struct Vec6 {
	data: [f32; 6],
}

impl Vec6 {
	pub const ZERO: Self = Self::splat(0.0);
	pub const ONE: Self = Self::splat(1.0);
	pub const NEG_ONE: Self = Self::splat(-1.0);
	pub const X0: Self = Self::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	pub const X1: Self = Self::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
	pub const X2: Self = Self::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
	pub const X3: Self = Self::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
	pub const X4: Self = Self::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	pub const X5: Self = Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
	pub const NEG_X0: Self = Self::new(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	pub const NEG_X1: Self = Self::new(0.0, -1.0, 0.0, 0.0, 0.0, 0.0);
	pub const NEG_X2: Self = Self::new(0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
	pub const NEG_X3: Self = Self::new(0.0, 0.0, 0.0, -1.0, 0.0, 0.0);
	pub const NEG_X4: Self = Self::new(0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
	pub const NEG_X5: Self = Self::new(0.0, 0.0, 0.0, 0.0, 0.0, -1.0);
	pub const AXES: [Self; 6] = [Self::X0, Self::X1, Self::X2, Self::X3, Self::X4, Self::X5];

	#[inline(always)]
	#[must_use]
	pub const fn new(x0: f32, x1: f32, x2: f32, x3: f32, x4: f32, x5: f32,) -> Self {
	   Self::from_array([x0, x1, x2, x3, x4, x5])
	}

	#[inline]
	#[must_use]
	pub const fn splat(v: f32) -> Self  {
		Self::new(v, v, v, v, v, v)
	}

	#[inline]
	#[must_use]
	pub const fn from_array(a: [f32; 6]) -> Self {
	   Self { data: a }
	}

	#[inline]
	#[must_use]
	pub const fn to_array(&self) -> [f32; 6] {
	   self.data
	}

	#[inline]
	#[must_use]
	pub const fn from_vec3(v1: Vec3, v2: Vec3) -> Self {
		Self::new(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z)
	}

	#[inline]
	#[must_use]
	pub fn dot(&self, rhs: &Self) -> f32 {
		self.data.iter().zip(rhs.data).map(|(a, b)| a * b).sum()
	}

	#[inline]
	#[must_use]
	pub fn element_sum(self) -> f32 {
		self.data.iter().sum()
	}

	#[inline]
	#[must_use]
	pub fn element_product(&self) -> f32 {
		self.data.iter().product()
	}

	#[inline]
	#[must_use]
	pub fn abs(&self) -> Self {
		Self::from_array(std::array::from_fn(|i| self.data[i].abs()))
	}

	#[inline]
	#[must_use]
	pub fn length(&self) -> f32 {
		f32::sqrt(self.dot(self))
	}

	#[inline]
	#[must_use]
	pub fn length_squared(&self) -> f32 {
		self.dot(self)
	}

	#[inline]
	#[must_use]
	pub const fn get(&self, index: usize) -> f32 {
		self.data[index]
	}

	#[inline]
	#[must_use]
	pub fn get_mut(&mut self, index: usize) -> &mut f32 {
		&mut self.data[index]
	}

	#[inline]
	#[must_use]
	pub fn upper_vec3(&self) -> Vec3 {
		Vec3::new(self.get(0), self.get(1), self.get(2))
	}

	#[inline]
	#[must_use]
	pub fn lower_vec3(&self) -> Vec3 {
		Vec3::new(self.get(3), self.get(4), self.get(5))
	}
}

impl Add<Self> for Vec6 {
	type Output = Self;
	#[inline]
	fn add(self, rhs: Self) -> Self {
		Self {data: std::array::from_fn(|i| self.data[i].add(rhs.data[i])) }
	}
}

impl Add<&Self> for Vec6 {
	type Output = Self;
	#[inline]
	fn add(self, rhs: &Self) -> Self {
		self.add(*rhs)
	}
}

impl Add<Self> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn add(self, rhs: Self) -> Vec6 {
		(*self).add(rhs)
	}
}

impl Add<&Self> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn add(self, rhs: &Self) -> Vec6 {
		(*self).add(*rhs)
	}
}

impl AddAssign<Self> for Vec6 {
	#[inline]
	fn add_assign(&mut self, rhs: Self) {
		for i in 0..6 {
			self.data[i].add_assign(rhs.data[i]);
		}
	}
}

impl AddAssign<&Self> for Vec6 {
	#[inline]
	fn add_assign(&mut self, rhs: &Self) {
		self.add_assign(*rhs);
	}
}

impl Sub<Self> for Vec6 {
	type Output = Self;
	#[inline]
	fn sub(self, rhs: Self) -> Self {
		Self {data: std::array::from_fn(|i| self.data[i].sub(rhs.data[i])) }
	}
}

impl Sub<&Self> for Vec6 {
	type Output = Self;
	#[inline]
	fn sub(self, rhs: &Self) -> Self {
		self.sub(*rhs)
	}
}

impl Sub<Self> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn sub(self, rhs: Self) -> Vec6 {
		(*self).sub(rhs)
	}
}

impl Sub<&Self> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn sub(self, rhs: &Self) -> Vec6 {
		(*self).sub(*rhs)
	}
}

impl SubAssign<Self> for Vec6 {
	#[inline]
	fn sub_assign(&mut self, rhs: Self) {
		for i in 0..6 {
			self.data[i].sub_assign(rhs.data[i]);
		}
	}
}

impl SubAssign<&Self> for Vec6 {
	#[inline]
	fn sub_assign(&mut self, rhs: &Self) {
		self.sub_assign(*rhs);
	}
}

impl Mul<f32> for Vec6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: f32) -> Vec6 {
		Vec6::from_array(std::array::from_fn(|i| rhs * self.get(i)))
	}
}

impl Mul<&f32> for Vec6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: &f32) -> Vec6 {
		self.mul(*rhs)
	}
}

impl Mul<f32> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: f32) -> Vec6 {
		(*self).mul(rhs)
	}
}
impl Mul<&f32> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: &f32) -> Vec6 {
		(*self).mul(*rhs)
	}
}

impl Mul<Vec6> for f32 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: Vec6) -> Vec6 {
		rhs.mul(self)
	}
}

impl Mul<&Vec6> for f32 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: &Vec6) -> Vec6 {
		self.mul(*rhs)
	}
}

impl Mul<Vec6> for &f32 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: Vec6) -> Vec6 {
		(*self).mul(rhs)
	}
}

impl Mul<&Vec6> for &f32 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: &Vec6) -> Vec6 {
		(*self).mul(*rhs)
	}
}

impl Div<f32> for Vec6 {
	type Output = Vec6;
	#[inline]
	fn div(self, rhs: f32) -> Vec6 {
		Vec6::from_array(std::array::from_fn(|i| self.get(i) / rhs))
	}
}

impl Div<&f32> for Vec6 {
	type Output = Vec6;
	#[inline]
	fn div(self, rhs: &f32) -> Vec6 {
		self.div(*rhs)
	}
}

impl Div<f32> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn div(self, rhs: f32) -> Vec6 {
		(*self).div(rhs)
	}
}

impl Div<&f32> for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn div(self, rhs: &f32) -> Vec6 {
		(*self).div(*rhs)
	}
}

impl Neg for Vec6 {
	type Output = Self;
	#[inline]
	fn neg(self) -> Self {
		Vec6::from_array(std::array::from_fn(|i| -self.get(i)))
	}
}

impl Neg for &Vec6 {
	type Output = Vec6;
	#[inline]
	fn neg(self) -> Vec6 {
		(*self).neg()
	}
}

impl fmt::Display for Vec6 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(p) = f.precision() {
            write!(f, "[{:.*}, {:.*}, {:.*}, {:.*}, {:.*}, {:.*}]", p, self.get(0), p, self.get(1), p, self.get(2), p, self.get(3), p, self.get(4), p, self.get(5))
        } else {
            write!(f, "[{}, {}, {}, {}, {}, {}]", self.get(0), self.get(1), self.get(2), self.get(3), self.get(4), self.get(5))
        }
    }
}

impl fmt::Debug for Vec6 {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt.debug_tuple(stringify!(Vec6))
            .field(&self.get(0))
            .field(&self.get(1))
            .field(&self.get(2))
            .field(&self.get(3))
            .field(&self.get(4))
            .field(&self.get(5))
            .finish()
    }
}

#[derive(Copy, Clone, PartialEq)]
pub struct Mat6 {
	matrix: [Vec6; 6],
}

impl Mat6 {
	pub const ZERO: Self = Self::from_cols(Vec6::ZERO, Vec6::ZERO, Vec6::ZERO, Vec6::ZERO, Vec6::ZERO, Vec6::ZERO);
	pub const IDENTITY: Self = Self::from_cols(Vec6::X0, Vec6::X1, Vec6::X2, Vec6::X3, Vec6::X4, Vec6::X5);

	#[inline(always)]
	#[must_use]
	pub const fn from_cols(
		axis0: Vec6,
		axis1: Vec6,
		axis2: Vec6,
		axis3: Vec6,
		axis4: Vec6,
		axis5: Vec6
	) -> Self {
		Self { matrix: [axis0, axis1, axis2, axis3, axis4, axis5] }
	}

	pub const fn from_mat3(m00: Mat3, m01: Mat3, m10: Mat3, m11: Mat3) -> Self {
		Self::from_cols(
			Vec6::from_vec3(m00.x_axis, m10.x_axis),
			Vec6::from_vec3(m00.y_axis, m10.y_axis),
			Vec6::from_vec3(m00.z_axis, m10.z_axis),
			Vec6::from_vec3(m01.x_axis, m11.x_axis),
			Vec6::from_vec3(m01.y_axis, m11.y_axis),
			Vec6::from_vec3(m01.z_axis, m11.z_axis)
		)
	}

	pub const fn from_array(cols: [Vec6; 6]) -> Self {
		Self::from_cols(cols[0], cols[1], cols[2], cols[3], cols[4], cols[5])
	}

	pub fn from_mat3_quat(m: Mat3, q: Quat) -> Self {
		Self::from_mat3(m, Mat3::ZERO, Mat3::ZERO, Mat3::from_quat(q))
	}

	pub const fn from_diagonal(diagonal: Vec6) -> Self {
        Self::from_array([
            Vec6::new(diagonal.get(0), 0.0, 0.0, 0.0, 0.0, 0.0),
            Vec6::new(0.0, diagonal.get(1), 0.0, 0.0, 0.0, 0.0),
            Vec6::new(0.0, 0.0, diagonal.get(2), 0.0, 0.0, 0.0),
            Vec6::new(0.0, 0.0, 0.0, diagonal.get(3), 0.0, 0.0),
            Vec6::new(0.0, 0.0, 0.0, 0.0, diagonal.get(4), 0.0),
            Vec6::new(0.0, 0.0, 0.0, 0.0, 0.0, diagonal.get(5))
        ])
    }

	pub fn to_mat3(&self) -> [Mat3; 4] {
		[
			Mat3::from_cols(self.col(0).upper_vec3(), self.col(1).upper_vec3(), self.col(2).upper_vec3()),
			Mat3::from_cols(self.col(3).upper_vec3(), self.col(4).upper_vec3(), self.col(5).upper_vec3()),
			Mat3::from_cols(self.col(0).lower_vec3(), self.col(1).lower_vec3(), self.col(2).lower_vec3()),
			Mat3::from_cols(self.col(3).lower_vec3(), self.col(4).lower_vec3(), self.col(5).lower_vec3())
		]
	}

	// #[inline(always)]
	// #[must_use]
	// pub fn inverse_checked<const CHECKED: bool>(&self) -> (Self, bool) {
	// 	let mut a = *self; // copy of self
	// 	let mut inv = Mat6::IDENTITY;
	// 	for i in 0..6 {
	// 		// Find pivot
	// 		let mut pivot_val = a.matrix[i].get(i);
	// 		let mut pivot_row = i;
	// 		for j in i + 1..6 {
	// 			if a.matrix[j].get(i).abs() > pivot_val.abs() {
	// 				pivot_val = a.matrix[j].get(i);
	// 				pivot_row = j;
	// 			}
	// 		}
	// 		if CHECKED && pivot_val.abs() < 1e-6 {
	// 			return (Mat6::ZERO, false);
	// 		} else if !CHECKED {
	// 			assert!(pivot_val.abs() > 1e-6);
	// 		}
	// 		// Swap rows if needed
	// 		if pivot_row != i {
	// 			a.matrix.swap(i, pivot_row);
	// 			inv.matrix.swap(i, pivot_row);
	// 		}
	// 		// Normalize pivot row
	// 		let inv_pivot = 1.0 / a.matrix[i].get(i);
	// 		for k in 0..6 {
	// 			*a.matrix[i].get_mut(k) *= inv_pivot;
	// 			*inv.matrix[i].get_mut(k) *= inv_pivot;
	// 		}
	// 		// Eliminate other rows
	// 		for j in 0..6 {
	// 			if j == i { continue; }
	// 			let factor = a.matrix[j].get(i);
	// 			for k in 0..6 {
	// 				*a.matrix[j].get_mut(k) -= factor * a.matrix[i].get(k);
	// 				*inv.matrix[j].get_mut(k) -= factor * inv.matrix[i].get(k);
	// 			}
	// 		}
	// 	}
	// 	(inv, true)
	// }

	// #[inline]
	// #[must_use]
	// pub fn inverse(&self) -> Self {
	// 	self.inverse_checked::<false>().0
	// }

	#[inline]
	#[must_use]
	pub fn col(&self, index: usize) -> &Vec6 {
		&self.matrix[index]
	}

	#[inline]
	#[must_use]
	pub fn col_mut(&mut self, index: usize) -> &mut Vec6 {
		&mut self.matrix[index]
	}

	#[inline]
	#[must_use]
	pub fn row(&self, index: usize) -> Vec6 {
		Vec6::from_array(std::array::from_fn(|i| self.matrix[i].get(index)))
	}
}

impl Add<Self> for Mat6 {
	type Output = Self;
	#[inline]
	fn add(self, rhs: Self) -> Self {
		Self::from_array(std::array::from_fn(|i| self.col(i) + rhs.col(i)))
	}
}

impl Add<&Self> for Mat6 {
	type Output = Self;
	#[inline]
	fn add(self, rhs: &Self) -> Self {
		self.add(*rhs)
	}
}

impl Add<Self> for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn add(self, rhs: Self) -> Mat6 {
		(*self).add(rhs)
	}
}

impl Add<&Self> for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn add(self, rhs: &Self) -> Mat6 {
		(*self).add(*rhs)
	}
}

impl AddAssign<Self> for Mat6 {
	#[inline]
	fn add_assign(&mut self, rhs: Self) {
		for i in 0..6 {
			self.col_mut(i).add_assign(rhs.col(i));
		}
	}
}

impl AddAssign<&Self> for Mat6 {
	#[inline]
	fn add_assign(&mut self, rhs: &Self) {
		self.add_assign(*rhs);
	}
}

impl Mul<Vec6> for Mat6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: Vec6) -> Vec6 {
		Vec6::from_array(std::array::from_fn(|i| self.row(i).dot(&rhs)))
	}
}

impl Mul<&Vec6> for Mat6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: &Vec6) -> Vec6 {
		self.mul(*rhs)
	}
}

impl Mul<Vec6> for &Mat6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: Vec6) -> Vec6 {
		(*self).mul(rhs)
	}
}
impl Mul<&Vec6> for &Mat6 {
	type Output = Vec6;
	#[inline]
	fn mul(self, rhs: &Vec6) -> Vec6 {
		(*self).mul(*rhs)
	}
}

impl Mul<f32> for Mat6 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: f32) -> Mat6 {
		Mat6::from_array(std::array::from_fn(|i| rhs * self.matrix[i]))
	}
}

impl Mul<&f32> for Mat6 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: &f32) -> Mat6 {
		self.mul(*rhs)
	}
}

impl Mul<f32> for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: f32) -> Mat6 {
		(*self).mul(rhs)
	}
}

impl Mul<&f32> for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: &f32) -> Mat6 {
		(*self).mul(*rhs)
	}
}

impl Mul<Mat6> for f32 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: Mat6) -> Mat6 {
		rhs.mul(self)
	}
}

impl Mul<&Mat6> for f32 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: &Mat6) -> Mat6 {
		self.mul(*rhs)
	}
}

impl Mul<Mat6> for &f32 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: Mat6) -> Mat6 {
		(*self).mul(rhs)
	}
}

impl Mul<&Mat6> for &f32 {
	type Output = Mat6;
	#[inline]
	fn mul(self, rhs: &Mat6) -> Mat6 {
		(*self).mul(*rhs)
	}
}

impl Div<f32> for Mat6 {
	type Output = Mat6;
	#[inline]
	fn div(self, rhs: f32) -> Mat6 {
		Mat6::from_array(std::array::from_fn(|i| self.matrix[i] / rhs))
	}
}

impl Div<&f32> for Mat6 {
	type Output = Mat6;
	#[inline]
	fn div(self, rhs: &f32) -> Mat6 {
		self.div(*rhs)
	}
}

impl Div<f32> for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn div(self, rhs: f32) -> Mat6 {
		(*self).div(rhs)
	}
}

impl Div<&f32> for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn div(self, rhs: &f32) -> Mat6 {
		(*self).div(*rhs)
	}
}

impl Neg for Mat6 {
	type Output = Self;
	#[inline]
	fn neg(self) -> Self {
		Mat6::from_array(std::array::from_fn(|i| -self.col(i)))
	}
}

impl Neg for &Mat6 {
	type Output = Mat6;
	#[inline]
	fn neg(self) -> Mat6 {
		(*self).neg()
	}
}

impl fmt::Debug for Mat6 {
    fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt.debug_struct(stringify!(Mat6))
            .field("0_axis", &self.col(0))
            .field("1_axis", &self.col(1))
            .field("2_axis", &self.col(2))
            .field("3_axis", &self.col(3))
            .field("4_axis", &self.col(4))
            .field("5_axis", &self.col(5))
            .finish()
    }
}

impl fmt::Display for Mat6 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(p) = f.precision() {
            write!(
                f,
                "[{:.*}, {:.*}, {:.*}, {:.*}, {:.*}, {:.*}]",
                p, self.col(0), p, self.col(1), p, self.col(2), p, self.col(3), p, self.col(4), p, self.col(5)
            )
        } else {
            write!(f, "[{}, {}, {}, {}, {}, {}]", self.col(0), self.col(1), self.col(2), self.col(3), self.col(4), self.col(5))
        }
    }
}
