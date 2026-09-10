use std::ops::*;
use fixed::{traits::ToFixed, types::I80F48};
use voxel_math::{Fixed, FixedVec3};

/// Solver-local Q80.48. Overflow is an error in release builds too.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Wide(I80F48);

impl Wide {
	pub const ZERO: Self = Self(I80F48::ZERO);
	pub const ONE: Self = Self(I80F48::ONE);
	pub const NEG_ONE: Self = Self(I80F48::NEG_ONE);
	pub const EPSILON: Self = Self(I80F48::DELTA);
	pub const MAX: Self = Self(I80F48::MAX);
	pub const fn from_bits(bits: i128) -> Self { Self(I80F48::from_bits(bits)) }
	pub fn from_num<T: ToFixed>(value: T) -> Self {
		Self(I80F48::checked_from_num(value).expect("solver conversion overflow or non-finite input"))
	}
	pub fn to_fixed(self) -> Fixed {
		// Round to the public format, with ties to even, then check its range.
		let bits = self.0.to_bits();
		let unit = 1i128 << 24;
		let quotient = bits / unit;
		let remainder = (bits % unit).unsigned_abs();
		let rounded = quotient + if remainder * 2 > unit as u128 || (remainder * 2 == unit as u128 && quotient & 1 != 0) { bits.signum() } else { 0 };
		Fixed::from_bits(i64::try_from(rounded).expect("solver output exceeds Q40.24 range"))
	}
	pub fn abs(self) -> Self { Self(self.0.checked_abs().expect("solver absolute value overflow")) }
	pub fn sqrt(self) -> Self { Self(self.0.checked_sqrt().expect("invalid solver square root")) }
	pub fn min(self, rhs: Self) -> Self { std::cmp::min(self, rhs) }
	pub fn max(self, rhs: Self) -> Self { std::cmp::max(self, rhs) }
}

impl From<Fixed> for Wide {
	fn from(value: Fixed) -> Self { Self(I80F48::from_bits(i128::from(value.to_bits()) << 24)) }
}
impl std::fmt::Display for Wide {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { self.0.fmt(f) }
}
macro_rules! scalar_ops {
	($trait:ident, $method:ident, $checked:ident, $assign:ident, $assign_method:ident, $message:literal) => {
		impl $trait for Wide { type Output = Self; fn $method(self, rhs: Self) -> Self { Self(self.0.$checked(rhs.0).expect($message)) } }
		impl $assign for Wide { fn $assign_method(&mut self, rhs: Self) { *self = self.$method(rhs); } }
	};
}
scalar_ops!(Add, add, checked_add, AddAssign, add_assign, "solver addition overflow");
scalar_ops!(Sub, sub, checked_sub, SubAssign, sub_assign, "solver subtraction overflow");
scalar_ops!(Mul, mul, checked_mul, MulAssign, mul_assign, "solver multiplication overflow");
scalar_ops!(Div, div, checked_div, DivAssign, div_assign, "solver division overflow or zero divisor");
impl Neg for Wide { type Output = Self; fn neg(self) -> Self { Self(self.0.checked_neg().expect("solver negation overflow")) } }

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WideVec3 { pub x: Wide, pub y: Wide, pub z: Wide }

impl WideVec3 {
	pub const ZERO: Self = Self::splat(Wide::ZERO);
	pub const ONE: Self = Self::splat(Wide::ONE);
	pub const X: Self = Self::new(Wide::ONE, Wide::ZERO, Wide::ZERO);
	pub const Y: Self = Self::new(Wide::ZERO, Wide::ONE, Wide::ZERO);
	pub const Z: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ONE);
	pub const fn new(x: Wide, y: Wide, z: Wide) -> Self { Self { x, y, z } }
	pub const fn splat(v: Wide) -> Self { Self::new(v, v, v) }
	pub const fn from_array(a: [Wide; 3]) -> Self { Self::new(a[0], a[1], a[2]) }
	pub const fn to_array(self) -> [Wide; 3] { [self.x, self.y, self.z] }
	pub fn to_fixed(self) -> FixedVec3 { FixedVec3::new(self.x.to_fixed(), self.y.to_fixed(), self.z.to_fixed()) }
	pub fn dot(self, rhs: Self) -> Wide { self.x * rhs.x + self.y * rhs.y + self.z * rhs.z }
	pub fn cross(self, rhs: Self) -> Self {
		Self::new(self.y * rhs.z - self.z * rhs.y, self.z * rhs.x - self.x * rhs.z, self.x * rhs.y - self.y * rhs.x)
	}
	pub fn abs(self) -> Self { Self::from_array(self.to_array().map(Wide::abs)) }
	pub fn length(self) -> Wide { scaled_length(&self.to_array()) }
	pub fn clamp(self, min: Self, max: Self) -> Self {
		Self::new(self.x.clamp(min.x, max.x), self.y.clamp(min.y, max.y), self.z.clamp(min.z, max.z))
	}
	pub fn clamp_length_max(self, max: Wide) -> Self {
		assert!(max >= Wide::ZERO, "negative solver length limit");
		let length = self.length();
		if length > max { (self / length) * max } else { self }
	}
}

/// Scale before squaring: force magnitudes need not fit in the squared range.
pub(super) fn scaled_length(values: &[Wide]) -> Wide {
	let scale = values.iter().fold(Wide::ZERO, |a, b| a.max(b.abs()));
	if scale == Wide::ZERO { return Wide::ZERO; }
	let squared = values.iter().fold(Wide::ZERO, |sum, v| { let normalized = *v / scale; sum + normalized * normalized });
	scale * squared.sqrt()
}

impl From<FixedVec3> for WideVec3 {
	fn from(v: FixedVec3) -> Self { Self::new(v.x.into(), v.y.into(), v.z.into()) }
}
impl Index<usize> for WideVec3 {
	type Output = Wide;
	fn index(&self, i: usize) -> &Wide { match i { 0 => &self.x, 1 => &self.y, 2 => &self.z, _ => panic!("vector index out of bounds") } }
}
impl IndexMut<usize> for WideVec3 {
	fn index_mut(&mut self, i: usize) -> &mut Wide { match i { 0 => &mut self.x, 1 => &mut self.y, 2 => &mut self.z, _ => panic!("vector index out of bounds") } }
}
impl Add for WideVec3 { type Output = Self; fn add(self, rhs: Self) -> Self { Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z) } }
impl Sub for WideVec3 { type Output = Self; fn sub(self, rhs: Self) -> Self { Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z) } }
impl Neg for WideVec3 { type Output = Self; fn neg(self) -> Self { Self::new(-self.x, -self.y, -self.z) } }
impl Mul<Wide> for WideVec3 { type Output = Self; fn mul(self, rhs: Wide) -> Self { Self::new(self.x * rhs, self.y * rhs, self.z * rhs) } }
impl Div<Wide> for WideVec3 { type Output = Self; fn div(self, rhs: Wide) -> Self { Self::new(self.x / rhs, self.y / rhs, self.z / rhs) } }
impl Mul<WideVec3> for Wide { type Output = WideVec3; fn mul(self, rhs: WideVec3) -> WideVec3 { rhs * self } }
impl AddAssign for WideVec3 { fn add_assign(&mut self, rhs: Self) { *self = *self + rhs; } }
impl SubAssign for WideVec3 { fn sub_assign(&mut self, rhs: Self) { *self = *self - rhs; } }
