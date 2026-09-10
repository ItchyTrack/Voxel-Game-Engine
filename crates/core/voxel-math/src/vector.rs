use crate::{Fixed, fixed::{rounded_div, rounded_sqrt, SCALE}};
use bevy::math::{BVec3, DMat3, DVec3, I8Vec3, I16Vec3, I64Vec3, IVec3, Mat3, Quat, U8Vec3, U16Vec3, U64Vec3, UVec3, Vec3};
use serde::{Deserialize, Serialize};
use std::{iter::Sum, ops::*};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct FixedVec3 {
	pub x: Fixed,
	pub y: Fixed,
	pub z: Fixed,
}

impl FixedVec3 {
	pub const ZERO: Self = Self::splat(Fixed::ZERO);
	pub const ONE: Self = Self::splat(Fixed::ONE);
	pub const NEG_ONE: Self = Self::splat(Fixed::NEG_ONE);
	pub const MIN: Self = Self::splat(Fixed::MIN);
	pub const MAX: Self = Self::splat(Fixed::MAX);
	pub const X: Self = Self::new(Fixed::ONE, Fixed::ZERO, Fixed::ZERO);
	pub const Y: Self = Self::new(Fixed::ZERO, Fixed::ONE, Fixed::ZERO);
	pub const Z: Self = Self::new(Fixed::ZERO, Fixed::ZERO, Fixed::ONE);
	pub const NEG_X: Self = Self::new(Fixed::NEG_ONE, Fixed::ZERO, Fixed::ZERO);
	pub const NEG_Y: Self = Self::new(Fixed::ZERO, Fixed::NEG_ONE, Fixed::ZERO);
	pub const NEG_Z: Self = Self::new(Fixed::ZERO, Fixed::ZERO, Fixed::NEG_ONE);
	pub const AXES: [Self; 3] = [Self::X, Self::Y, Self::Z];

	pub const fn new(x: Fixed, y: Fixed, z: Fixed) -> Self { Self { x, y, z } }
	pub const fn splat(value: Fixed) -> Self { Self::new(value, value, value) }
	pub const fn from_array(a: [Fixed; 3]) -> Self { Self::new(a[0], a[1], a[2]) }
	pub const fn to_array(self) -> [Fixed; 3] { [self.x, self.y, self.z] }
	pub fn from_vec3(v: Vec3) -> Self { Self::new(Fixed::from_num(v.x), Fixed::from_num(v.y), Fixed::from_num(v.z)) }
	pub fn from_dvec3(v: DVec3) -> Self { Self::new(Fixed::from_num(v.x), Fixed::from_num(v.y), Fixed::from_num(v.z)) }
	/// Float output for rendering and other float-only boundaries.
	pub fn as_vec3(self) -> Vec3 { Vec3::new(self.x.to_num(), self.y.to_num(), self.z.to_num()) }
	/// Float output for rendering and other float-only boundaries.
	pub fn as_dvec3(self) -> DVec3 { DVec3::new(self.x.to_num(), self.y.to_num(), self.z.to_num()) }
	/// Integer vector casts truncate toward zero and panic on overflow.
	pub fn as_ivec3(self) -> IVec3 { IVec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }
	pub fn as_uvec3(self) -> UVec3 { UVec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }
	pub fn as_i8vec3(self) -> I8Vec3 { I8Vec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }
	pub fn as_i16vec3(self) -> I16Vec3 { I16Vec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }
	pub fn as_i64vec3(self) -> I64Vec3 { I64Vec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }
	pub fn as_u8vec3(self) -> U8Vec3 { U8Vec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }
	pub fn as_u16vec3(self) -> U16Vec3 { U16Vec3::new(self.x.trunc().to_num(), self.y.trunc().to_num(), self.z.trunc().to_num()) }

	fn wide_length_squared(self) -> u128 {
		self.to_array().into_iter().map(|v| {
			let v = v.to_bits() as i128;
			(v * v) as u128
		}).sum()
	}
	pub fn length(self) -> Fixed { Fixed::from_wide(rounded_sqrt(self.wide_length_squared()) as i128) }
	pub fn length_squared(self) -> Fixed { self.dot(self) }
	pub fn distance(self, rhs: Self) -> Fixed { (self - rhs).length() }
	pub fn distance_squared(self, rhs: Self) -> Fixed { (self - rhs).length_squared() }
	pub fn dot(self, rhs: Self) -> Fixed {
		let mut positive = 0u128;
		let mut negative = 0u128;
		for i in 0..3 {
			let product = self[i].to_bits() as i128 * rhs[i].to_bits() as i128;
			if product < 0 { negative += product.unsigned_abs(); } else { positive += product as u128; }
		}
		let (magnitude, sign) = if positive >= negative { (positive - negative, 1) } else { (negative - positive, -1) };
		let sum = i128::try_from(magnitude).expect("fixed dot product overflow") * sign;
		Fixed::from_wide(rounded_div(sum, SCALE))
	}
	pub fn cross(self, rhs: Self) -> Self {
		let component = |a: Fixed, b: Fixed, c: Fixed, d: Fixed| {
			let first = a.to_bits() as i128 * b.to_bits() as i128;
			let second = c.to_bits() as i128 * d.to_bits() as i128;
			Fixed::from_wide(rounded_div(first.checked_sub(second).expect("fixed cross product overflow"), SCALE))
		};
		Self::new(component(self.y, rhs.z, self.z, rhs.y), component(self.z, rhs.x, self.x, rhs.z), component(self.x, rhs.y, self.y, rhs.x))
	}
	pub fn normalize(self) -> Self {
		let squared = self.wide_length_squared();
		assert!(squared != 0, "cannot normalize a zero vector");
		let shift = squared.leading_zeros() / 2;
		let length = rounded_sqrt(squared << (2 * shift));
		Self::from_array(self.to_array().map(|v| Fixed::from_wide(rounded_div((v.to_bits() as i128 * SCALE) << shift, length as i128))))
	}
	pub fn normalize_or_zero(self) -> Self { if self == Self::ZERO { Self::ZERO } else { self.normalize() } }
	pub fn try_normalize(self) -> Option<Self> { (self != Self::ZERO).then(|| self.normalize()) }
	pub fn is_normalized(self) -> bool { self.length().abs_diff_eq(Fixed::ONE, Fixed::from_bits(4)) }
	pub fn recip(self) -> Self { Self::new(self.x.recip(), self.y.recip(), self.z.recip()) }
	pub fn abs(self) -> Self { Self::new(self.x.abs(), self.y.abs(), self.z.abs()) }
	pub fn signum(self) -> Self { Self::new(self.x.signum(), self.y.signum(), self.z.signum()) }
	pub fn min(self, rhs: Self) -> Self { Self::new(self.x.min(rhs.x), self.y.min(rhs.y), self.z.min(rhs.z)) }
	pub fn max(self, rhs: Self) -> Self { Self::new(self.x.max(rhs.x), self.y.max(rhs.y), self.z.max(rhs.z)) }
	pub fn clamp(self, min: Self, max: Self) -> Self { Self::new(self.x.clamp(min.x, max.x), self.y.clamp(min.y, max.y), self.z.clamp(min.z, max.z)) }
	pub fn floor(self) -> Self { Self::new(self.x.floor(), self.y.floor(), self.z.floor()) }
	pub fn ceil(self) -> Self { Self::new(self.x.ceil(), self.y.ceil(), self.z.ceil()) }
	pub fn round(self) -> Self { Self::new(self.x.round(), self.y.round(), self.z.round()) }
	pub fn max_element(self) -> Fixed { self.x.max(self.y).max(self.z) }
	pub fn min_element(self) -> Fixed { self.x.min(self.y).min(self.z) }
	pub fn max_position(self) -> usize { if self.x >= self.y && self.x >= self.z { 0 } else if self.y >= self.z { 1 } else { 2 } }
	pub fn min_position(self) -> usize { if self.x <= self.y && self.x <= self.z { 0 } else if self.y <= self.z { 1 } else { 2 } }
	pub fn cmpeq(self, rhs: Self) -> BVec3 { BVec3::new(self.x == rhs.x, self.y == rhs.y, self.z == rhs.z) }
	pub fn cmpne(self, rhs: Self) -> BVec3 { BVec3::new(self.x != rhs.x, self.y != rhs.y, self.z != rhs.z) }
	pub fn cmpge(self, rhs: Self) -> BVec3 { BVec3::new(self.x >= rhs.x, self.y >= rhs.y, self.z >= rhs.z) }
	pub fn cmpgt(self, rhs: Self) -> BVec3 { BVec3::new(self.x > rhs.x, self.y > rhs.y, self.z > rhs.z) }
	pub fn cmple(self, rhs: Self) -> BVec3 { BVec3::new(self.x <= rhs.x, self.y <= rhs.y, self.z <= rhs.z) }
	pub fn cmplt(self, rhs: Self) -> BVec3 { BVec3::new(self.x < rhs.x, self.y < rhs.y, self.z < rhs.z) }
	pub fn select(mask: BVec3, if_true: Self, if_false: Self) -> Self {
		Self::new(if mask.x { if_true.x } else { if_false.x }, if mask.y { if_true.y } else { if_false.y }, if mask.z { if_true.z } else { if_false.z })
	}
	pub fn abs_diff_eq(self, rhs: Self, tolerance: Fixed) -> bool {
		self.x.abs_diff_eq(rhs.x, tolerance) && self.y.abs_diff_eq(rhs.y, tolerance) && self.z.abs_diff_eq(rhs.z, tolerance)
	}
	pub fn lerp(self, rhs: Self, t: Fixed) -> Self { self + (rhs - self) * t }
	pub fn move_towards(self, rhs: Self, max_distance: Fixed) -> Self {
		let delta = rhs - self;
		let length = delta.length();
		if length <= max_distance || length.is_zero() { rhs } else { self + delta * (max_distance / length) }
	}
}

macro_rules! from_integer_vector {
	($($ty:ty),*) => { $(impl From<$ty> for FixedVec3 {
		fn from(v: $ty) -> Self { Self::new(Fixed::from_num(v.x), Fixed::from_num(v.y), Fixed::from_num(v.z)) }
	})* };
}
from_integer_vector!(IVec3, UVec3, I8Vec3, U8Vec3, I16Vec3, U16Vec3, I64Vec3, U64Vec3);
impl From<[Fixed; 3]> for FixedVec3 { fn from(v: [Fixed; 3]) -> Self { Self::from_array(v) } }
impl From<FixedVec3> for [Fixed; 3] { fn from(v: FixedVec3) -> Self { v.to_array() } }
impl Index<usize> for FixedVec3 {
	type Output = Fixed;
	fn index(&self, index: usize) -> &Fixed { match index { 0 => &self.x, 1 => &self.y, 2 => &self.z, _ => panic!("vector index out of bounds") } }
}
impl IndexMut<usize> for FixedVec3 {
	fn index_mut(&mut self, index: usize) -> &mut Fixed { match index { 0 => &mut self.x, 1 => &mut self.y, 2 => &mut self.z, _ => panic!("vector index out of bounds") } }
}
macro_rules! vector_op {
	($trait:ident, $method:ident, $assign:ident, $assign_method:ident, $op:tt) => {
		impl $trait for FixedVec3 {
			type Output = Self;
			fn $method(self, rhs: Self) -> Self { Self::new(self.x $op rhs.x, self.y $op rhs.y, self.z $op rhs.z) }
		}
		impl $trait<Fixed> for FixedVec3 {
			type Output = Self;
			fn $method(self, rhs: Fixed) -> Self { self $op Self::splat(rhs) }
		}
		impl $assign for FixedVec3 { fn $assign_method(&mut self, rhs: Self) { *self = *self $op rhs; } }
		impl $assign<Fixed> for FixedVec3 { fn $assign_method(&mut self, rhs: Fixed) { *self = *self $op rhs; } }
	};
}
vector_op!(Add, add, AddAssign, add_assign, +);
vector_op!(Sub, sub, SubAssign, sub_assign, -);
vector_op!(Mul, mul, MulAssign, mul_assign, *);
vector_op!(Div, div, DivAssign, div_assign, /);
vector_op!(Rem, rem, RemAssign, rem_assign, %);
impl Mul<FixedVec3> for Fixed { type Output = FixedVec3; fn mul(self, rhs: FixedVec3) -> FixedVec3 { rhs * self } }
impl Div<FixedVec3> for Fixed { type Output = FixedVec3; fn div(self, rhs: FixedVec3) -> FixedVec3 { FixedVec3::splat(self) / rhs } }
impl Neg for FixedVec3 { type Output = Self; fn neg(self) -> Self { Self::new(-self.x, -self.y, -self.z) } }
impl Sum for FixedVec3 { fn sum<I: Iterator<Item = Self>>(iter: I) -> Self { iter.fold(Self::ZERO, |a, b| a + b) } }
impl<'a> Sum<&'a Self> for FixedVec3 { fn sum<I: Iterator<Item = &'a Self>>(iter: I) -> Self { iter.copied().sum() } }

impl Mul<FixedVec3> for Quat {
	type Output = FixedVec3;
	fn mul(self, rhs: FixedVec3) -> FixedVec3 {
		let [x, y, z, w] = self.to_array().map(Fixed::from_num);
		let two = Fixed::from_num(2);
		let xx = two * x * x;
		let yy = two * y * y;
		let zz = two * z * z;
		let xy = two * x * y;
		let xz = two * x * z;
		let yz = two * y * z;
		let wx = two * w * x;
		let wy = two * w * y;
		let wz = two * w * z;
		FixedVec3::new(
			FixedVec3::new(Fixed::ONE - yy - zz, xy - wz, xz + wy).dot(rhs),
			FixedVec3::new(xy + wz, Fixed::ONE - xx - zz, yz - wx).dot(rhs),
			FixedVec3::new(xz - wy, yz + wx, Fixed::ONE - xx - yy).dot(rhs),
		)
	}
}
impl Mul<FixedVec3> for Mat3 {
	type Output = FixedVec3;
	fn mul(self, rhs: FixedVec3) -> FixedVec3 {
		FixedVec3::new(FixedVec3::from_vec3(self.row(0)).dot(rhs), FixedVec3::from_vec3(self.row(1)).dot(rhs), FixedVec3::from_vec3(self.row(2)).dot(rhs))
	}
}
impl Mul<FixedVec3> for DMat3 {
	type Output = FixedVec3;
	fn mul(self, rhs: FixedVec3) -> FixedVec3 {
		FixedVec3::new(FixedVec3::from_dvec3(self.row(0)).dot(rhs), FixedVec3::from_dvec3(self.row(1)).dot(rhs), FixedVec3::from_dvec3(self.row(2)).dot(rhs))
	}
}
