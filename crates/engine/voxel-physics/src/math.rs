use std::ops::*;
use bevy::math::{DMat3, Quat};
use voxel_math::{Fixed, FixedVec3};

mod wide;
pub use wide::{Wide, WideVec3};

#[cfg(test)]
#[path = "math_tests.rs"]
mod tests;

/// Fixed solver matrix. Float coefficients enter only at rotation and inertia boundaries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mat3 {
	pub x_axis: WideVec3,
	pub y_axis: WideVec3,
	pub z_axis: WideVec3,
}

impl Mat3 {
	pub const ZERO: Self = Self::from_cols(WideVec3::ZERO, WideVec3::ZERO, WideVec3::ZERO);
	pub const IDENTITY: Self = Self::from_cols(WideVec3::X, WideVec3::Y, WideVec3::Z);
	pub const fn from_cols(x_axis: WideVec3, y_axis: WideVec3, z_axis: WideVec3) -> Self { Self { x_axis, y_axis, z_axis } }
	pub fn from_cols_array_2d(cols: &[[Wide; 3]; 3]) -> Self {
		Self::from_cols(WideVec3::from_array(cols[0]), WideVec3::from_array(cols[1]), WideVec3::from_array(cols[2]))
	}
	pub fn from_diagonal(d: WideVec3) -> Self {
		Self::from_cols(WideVec3::X * d.x, WideVec3::Y * d.y, WideVec3::Z * d.z)
	}
	pub fn from_quat(q: Quat) -> Self { Self::from_cols((q * FixedVec3::X).into(), (q * FixedVec3::Y).into(), (q * FixedVec3::Z).into()) }
	pub fn from_inertia(m: DMat3) -> Self {
		let convert = |v: bevy::math::DVec3| WideVec3::new(Wide::from_num(v.x), Wide::from_num(v.y), Wide::from_num(v.z));
		Self::from_cols(convert(m.x_axis), convert(m.y_axis), convert(m.z_axis))
	}
	pub fn col(self, i: usize) -> WideVec3 { [self.x_axis, self.y_axis, self.z_axis][i] }
	pub fn row(self, i: usize) -> WideVec3 { WideVec3::new(self.x_axis[i], self.y_axis[i], self.z_axis[i]) }
	pub fn transpose(self) -> Self { Self::from_cols(self.row(0), self.row(1), self.row(2)) }
}

impl Mul<WideVec3> for Mat3 {
	type Output = WideVec3;
	fn mul(self, v: WideVec3) -> WideVec3 { WideVec3::new(self.row(0).dot(v), self.row(1).dot(v), self.row(2).dot(v)) }
}
impl Mul for Mat3 {
	type Output = Self;
	fn mul(self, m: Self) -> Self { Self::from_cols(self * m.x_axis, self * m.y_axis, self * m.z_axis) }
}
impl Mul<Wide> for Mat3 {
	type Output = Self;
	fn mul(self, s: Wide) -> Self { Self::from_cols(self.x_axis * s, self.y_axis * s, self.z_axis * s) }
}
impl Mul<Mat3> for Wide {
	type Output = Mat3;
	fn mul(self, m: Mat3) -> Mat3 { m * self }
}
impl Neg for Mat3 {
	type Output = Self;
	fn neg(self) -> Self { Self::from_cols(-self.x_axis, -self.y_axis, -self.z_axis) }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Vec6 { data: [Wide; 6] }

impl Vec6 {
	pub const ZERO: Self = Self::splat(Wide::ZERO);
	pub const ONE: Self = Self::splat(Wide::ONE);
	pub const NEG_ONE: Self = Self::splat(Wide::NEG_ONE);
	pub const X0: Self = Self::new(Wide::ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	pub const X1: Self = Self::new(Wide::ZERO, Wide::ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	pub const X2: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	pub const X3: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ONE, Wide::ZERO, Wide::ZERO);
	pub const X4: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ONE, Wide::ZERO);
	pub const X5: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ONE);
	pub const NEG_X0: Self = Self::new(Wide::NEG_ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	pub const NEG_X1: Self = Self::new(Wide::ZERO, Wide::NEG_ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	pub const NEG_X2: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::NEG_ONE, Wide::ZERO, Wide::ZERO, Wide::ZERO);
	pub const NEG_X3: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::NEG_ONE, Wide::ZERO, Wide::ZERO);
	pub const NEG_X4: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::NEG_ONE, Wide::ZERO);
	pub const NEG_X5: Self = Self::new(Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::ZERO, Wide::NEG_ONE);
	pub const AXES: [Self; 6] = [Self::X0, Self::X1, Self::X2, Self::X3, Self::X4, Self::X5];
	pub const fn new(a: Wide, b: Wide, c: Wide, d: Wide, e: Wide, f: Wide) -> Self { Self::from_array([a,b,c,d,e,f]) }
	pub const fn splat(v: Wide) -> Self { Self::from_array([v; 6]) }
	pub const fn from_array(data: [Wide; 6]) -> Self { Self { data } }
	pub const fn to_array(&self) -> [Wide; 6] { self.data }
	pub const fn from_vec3(a: WideVec3, b: WideVec3) -> Self { Self::new(a.x, a.y, a.z, b.x, b.y, b.z) }
	pub const fn get(&self, i: usize) -> Wide { self.data[i] }
	pub fn get_mut(&mut self, i: usize) -> &mut Wide { &mut self.data[i] }
	pub fn upper_vec3(&self) -> WideVec3 { WideVec3::new(self.data[0], self.data[1], self.data[2]) }
	pub fn lower_vec3(&self) -> WideVec3 { WideVec3::new(self.data[3], self.data[4], self.data[5]) }
	pub fn dot(&self, rhs: &Self) -> Wide { (0..6).fold(Wide::ZERO, |sum, i| sum + self.data[i] * rhs.data[i]) }
	pub fn element_sum(self) -> Wide { self.data.into_iter().fold(Wide::ZERO, |a, b| a + b) }
	pub fn element_product(&self) -> Wide { self.data.into_iter().fold(Wide::ONE, |a, b| a * b) }
	pub fn abs(&self) -> Self { Self::from_array(self.data.map(Wide::abs)) }
	pub fn length_squared(&self) -> Wide { self.dot(self) }
	pub fn length(&self) -> Wide { wide::scaled_length(&self.data) }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mat6 { matrix: [Vec6; 6] }

impl Mat6 {
	pub const ZERO: Self = Self::from_array([Vec6::ZERO; 6]);
	pub const IDENTITY: Self = Self::from_array(Vec6::AXES);
	pub const fn from_cols(a: Vec6, b: Vec6, c: Vec6, d: Vec6, e: Vec6, f: Vec6) -> Self { Self::from_array([a,b,c,d,e,f]) }
	pub const fn from_array(matrix: [Vec6; 6]) -> Self { Self { matrix } }
	pub const fn from_mat3(a: Mat3, b: Mat3, c: Mat3, d: Mat3) -> Self {
		Self::from_cols(Vec6::from_vec3(a.x_axis, c.x_axis), Vec6::from_vec3(a.y_axis, c.y_axis), Vec6::from_vec3(a.z_axis, c.z_axis),
			Vec6::from_vec3(b.x_axis, d.x_axis), Vec6::from_vec3(b.y_axis, d.y_axis), Vec6::from_vec3(b.z_axis, d.z_axis))
	}
	pub fn from_mat3_quat(m: Mat3, q: Quat) -> Self { Self::from_mat3(m, Mat3::ZERO, Mat3::ZERO, Mat3::from_quat(q)) }
	pub const fn from_diagonal(d: Vec6) -> Self {
		let mut cols = [Vec6::ZERO; 6];
		let mut i = 0;
		while i < 6 { cols[i].data[i] = d.data[i]; i += 1; }
		Self::from_array(cols)
	}
	pub fn col(&self, i: usize) -> &Vec6 { &self.matrix[i] }
	pub fn col_mut(&mut self, i: usize) -> &mut Vec6 { &mut self.matrix[i] }
	pub fn row(&self, i: usize) -> Vec6 { Vec6::from_array(std::array::from_fn(|j| self.matrix[j].get(i))) }
	pub fn to_mat3(&self) -> [Mat3; 4] {
		[
			Mat3::from_cols(self.col(0).upper_vec3(), self.col(1).upper_vec3(), self.col(2).upper_vec3()),
			Mat3::from_cols(self.col(3).upper_vec3(), self.col(4).upper_vec3(), self.col(5).upper_vec3()),
			Mat3::from_cols(self.col(0).lower_vec3(), self.col(1).lower_vec3(), self.col(2).lower_vec3()),
			Mat3::from_cols(self.col(3).lower_vec3(), self.col(4).lower_vec3(), self.col(5).lower_vec3()),
		]
	}
}

macro_rules! value_ops {
	($ty:ty, $field:ident) => {
		impl Add for $ty { type Output = Self; fn add(self, rhs: Self) -> Self { Self::from_array(std::array::from_fn(|i| self.$field[i] + rhs.$field[i])) } }
		impl Sub for $ty { type Output = Self; fn sub(self, rhs: Self) -> Self { Self::from_array(std::array::from_fn(|i| self.$field[i] - rhs.$field[i])) } }
		impl Neg for $ty { type Output = Self; fn neg(self) -> Self { Self::from_array(self.$field.map(|v| -v)) } }
		impl Mul<Wide> for $ty { type Output = Self; fn mul(self, rhs: Wide) -> Self { Self::from_array(self.$field.map(|v| v * rhs)) } }
		impl Div<Wide> for $ty { type Output = Self; fn div(self, rhs: Wide) -> Self { Self::from_array(self.$field.map(|v| v / rhs)) } }
		impl Mul<$ty> for Wide { type Output = $ty; fn mul(self, rhs: $ty) -> $ty { rhs * self } }
		impl AddAssign for $ty { fn add_assign(&mut self, rhs: Self) { *self = *self + rhs; } }
		impl SubAssign for $ty { fn sub_assign(&mut self, rhs: Self) { *self = *self - rhs; } }
	};
}
value_ops!(Vec6, data);
value_ops!(Mat6, matrix);
impl Mul<Vec6> for Mat6 {
	type Output = Vec6;
	fn mul(self, rhs: Vec6) -> Vec6 { Vec6::from_array(std::array::from_fn(|i| self.row(i).dot(&rhs))) }
}

macro_rules! ref_binary {
	($lhs:ty, $rhs:ty, $out:ty, $trait:ident, $method:ident, $op:tt) => {
		impl $trait<&$rhs> for $lhs { type Output = $out; fn $method(self, rhs: &$rhs) -> $out { self $op *rhs } }
		impl $trait<$rhs> for &$lhs { type Output = $out; fn $method(self, rhs: $rhs) -> $out { *self $op rhs } }
		impl $trait<&$rhs> for &$lhs { type Output = $out; fn $method(self, rhs: &$rhs) -> $out { *self $op *rhs } }
	};
}
macro_rules! ref_ops {
	($ty:ty) => {
		ref_binary!($ty, $ty, $ty, Add, add, +);
		ref_binary!($ty, $ty, $ty, Sub, sub, -);
		ref_binary!($ty, Wide, $ty, Mul, mul, *);
		ref_binary!($ty, Wide, $ty, Div, div, /);
		ref_binary!(Wide, $ty, $ty, Mul, mul, *);
		impl Neg for &$ty { type Output = $ty; fn neg(self) -> $ty { -*self } }
		impl AddAssign<&$ty> for $ty { fn add_assign(&mut self, rhs: &$ty) { *self += *rhs; } }
		impl SubAssign<&$ty> for $ty { fn sub_assign(&mut self, rhs: &$ty) { *self -= *rhs; } }
	};
}
ref_ops!(Vec6);
ref_ops!(Mat6);
ref_binary!(Mat6, Vec6, Vec6, Mul, mul, *);

impl std::fmt::Display for Vec6 {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { write!(f, "[{}, {}, {}, {}, {}, {}]", self.data[0], self.data[1], self.data[2], self.data[3], self.data[4], self.data[5]) }
}
impl std::fmt::Display for Mat6 {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { write!(f, "[{}, {}, {}, {}, {}, {}]", self.matrix[0], self.matrix[1], self.matrix[2], self.matrix[3], self.matrix[4], self.matrix[5]) }
}

/// Solve a positive semidefinite system. Unresolved null directions stay fixed.
pub(crate) fn solve_symmetric(a: Mat6, b: Vec6) -> Vec6 {
	// Equilibrate axes before LDL so mass and inertia can differ by many orders.
	let scale: [Wide; 6] = std::array::from_fn(|i| a.col(i).get(i).max(Wide::ZERO).sqrt());
	let mut l = [[Wide::ZERO; 6]; 6];
	let mut d = [Wide::ZERO; 6];
	let mut y = [Wide::ZERO; 6];
	let mut x = [Wide::ZERO; 6];
	let pivot_floor = Wide::EPSILON * Wide::from_num(256);
	for i in 0..6 {
		if scale[i] == Wide::ZERO { continue; }
		for j in 0..i {
			if d[j] == Wide::ZERO { continue; }
			let mut value = a.col(i).get(j) / scale[i] / scale[j];
			for k in 0..j { value -= (l[i][k] * d[k]) * l[j][k]; }
			l[i][j] = value / d[j];
		}
		let mut pivot = a.col(i).get(i) / scale[i] / scale[i];
		for k in 0..i { pivot -= (l[i][k] * d[k]) * l[i][k]; }
		// A flat axis (or roundoff around one) has no reliable inverse. Project it
		// out rather than divide by zero or invent an unbounded correction.
		if pivot <= pivot_floor { continue; }
		d[i] = pivot;
		y[i] = b.get(i) / scale[i];
		for j in 0..i { y[i] -= l[i][j] * y[j]; }
	}
	for i in (0..6).rev() {
		if d[i] == Wide::ZERO { continue; }
		x[i] = y[i] / d[i];
		for j in i + 1..6 { x[i] -= l[j][i] * x[j]; }
	}
	Vec6::from_array(std::array::from_fn(|i| if scale[i] == Wide::ZERO { Wide::ZERO } else { x[i] / scale[i] }))
}

pub(crate) trait FixedVec3Ext {
	fn project_onto(self, axis: Self) -> Self;
	fn any_orthonormal_pair(self) -> (Self, Self) where Self: Sized;
}

impl FixedVec3Ext for FixedVec3 {
	fn project_onto(self, axis: Self) -> Self { axis * (self.dot(axis) / axis.length_squared()) }
	fn any_orthonormal_pair(self) -> (Self, Self) {
		let axis = FixedVec3::AXES[self.abs().min_position()];
		let tangent = self.cross(axis).normalize();
		(tangent, self.cross(tangent).normalize())
	}
}

/// Convert the tick duration without passing through floating point.
pub fn fixed_duration(duration: std::time::Duration) -> Fixed {
	Fixed::from_num(duration.as_secs()) + Fixed::from_num(duration.subsec_nanos()) / Fixed::from_num(1_000_000_000u32)
}

pub(crate) fn usable_timestep(dt: Fixed) -> bool {
	// Every positive public tick has a nonzero square in Q80.48.
	dt > Fixed::ZERO
}

pub(crate) fn require_unit_scale(transform: &voxel_transform::Transform) {
	assert_eq!(transform.scale, voxel_transform::Scale::ONE, "physics requires unit scale");
}

/// Only angular offsets may cross this quaternion boundary.
pub(crate) fn rotation_step(rotation: Quat, delta: FixedVec3) -> Quat {
	(Quat::from_scaled_axis(delta.as_vec3()) * rotation).normalize()
}

pub(crate) fn rotation_delta(a: Quat, b: Quat) -> FixedVec3 {
	FixedVec3::from_vec3((a * b.inverse()).xyz()) * Fixed::from_num(2)
}

pub(crate) fn rotation_difference(a: Quat, b: Quat) -> FixedVec3 {
	FixedVec3::from_vec3((a * b.inverse()).normalize().to_scaled_axis())
}

pub(crate) fn rotation_correction(rotation: Quat, delta: FixedVec3) -> Quat {
	let half = (delta / Fixed::from_num(2)).as_vec3();
	(rotation + Quat::from_xyzw(half.x, half.y, half.z, 0.0) * rotation).normalize()
}
