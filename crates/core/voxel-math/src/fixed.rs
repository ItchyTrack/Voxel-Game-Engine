use fixed::{traits::{FromFixed, ToFixed}, types::I40F24};
use serde::{Deserialize, Serialize};
use std::{fmt, iter::{Product, Sum}, ops::*};

/// Signed Q40.24. Arithmetic and conversions panic on overflow in every build.
#[derive(Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
#[repr(transparent)]
pub struct Fixed(i64);

pub(crate) const SCALE: i128 = 1 << 24;

/// Divide with nearest rounding; halfway values go to the even integer.
pub(crate) fn rounded_div(n: i128, d: i128) -> i128 {
	let q = n.checked_div(d).expect("fixed division overflow or zero divisor");
	let r = n % d;
	let twice = r.unsigned_abs() * 2;
	if twice > d.unsigned_abs() || (twice == d.unsigned_abs() && q & 1 != 0) {
		q.checked_add(if (n < 0) == (d < 0) { 1 } else { -1 }).expect("fixed rounding overflow")
	} else { q }
}

pub(crate) fn rounded_sqrt(n: u128) -> u128 {
	let root = n.isqrt();
	let remainder = n - root * root;
	if remainder > root { root + 1 } else { root }
}

impl Fixed {
	pub const ZERO: Self = Self(0);
	pub const ONE: Self = Self(1 << 24);
	pub const NEG_ONE: Self = Self(-(1 << 24));
	pub const MIN: Self = Self(i64::MIN);
	pub const MAX: Self = Self(i64::MAX);
	pub const EPSILON: Self = Self(1);
	pub const FRAC_BITS: u32 = 24;

	pub fn from_num<T: ToFixed>(value: T) -> Self {
		Self(I40F24::checked_from_num(value).expect("fixed conversion overflow or non-finite input").to_bits())
	}
	pub fn to_num<T: FromFixed>(self) -> T {
		I40F24::from_bits(self.0).checked_to_num().expect("fixed output conversion overflow")
	}
	pub const fn from_bits(bits: i64) -> Self { Self(bits) }
	pub const fn to_bits(self) -> i64 { self.0 }
	pub(crate) fn from_wide(bits: i128) -> Self { Self(i64::try_from(bits).expect("fixed arithmetic overflow")) }
	pub fn checked_add(self, rhs: Self) -> Option<Self> { self.0.checked_add(rhs.0).map(Self) }
	pub fn checked_sub(self, rhs: Self) -> Option<Self> { self.0.checked_sub(rhs.0).map(Self) }
	pub fn checked_mul(self, rhs: Self) -> Option<Self> {
		i64::try_from(rounded_div(self.0 as i128 * rhs.0 as i128, SCALE)).ok().map(Self)
	}
	pub fn checked_div(self, rhs: Self) -> Option<Self> {
		if rhs.is_zero() { return None; }
		i64::try_from(rounded_div(self.0 as i128 * SCALE, rhs.0 as i128)).ok().map(Self)
	}
	pub fn abs(self) -> Self { Self(self.0.checked_abs().expect("fixed absolute value overflow")) }
	pub fn min(self, rhs: Self) -> Self { std::cmp::min(self, rhs) }
	pub fn max(self, rhs: Self) -> Self { std::cmp::max(self, rhs) }
	pub fn clamp(self, min: Self, max: Self) -> Self { Ord::clamp(self, min, max) }
	pub fn floor(self) -> Self { Self(self.0 & !((1 << 24) - 1)) }
	pub fn ceil(self) -> Self {
		if self == self.floor() { self } else { self.floor() + Self::ONE }
	}
	pub fn round(self) -> Self { Self::from_wide(rounded_div(self.0 as i128, SCALE) * SCALE) }
	pub fn trunc(self) -> Self { Self(self.0 / (1 << 24) * (1 << 24)) }
	pub fn fract(self) -> Self { self - self.trunc() }
	pub fn sqrt(self) -> Self {
		assert!(self >= Self::ZERO, "fixed square root of a negative value");
		Self::from_wide(rounded_sqrt(self.0 as u128 * SCALE as u128) as i128)
	}
	pub fn recip(self) -> Self { Self::ONE / self }
	pub fn signum(self) -> Self { Self(self.0.signum() * (1 << 24)) }
	pub const fn is_zero(self) -> bool { self.0 == 0 }
	pub fn abs_diff_eq(self, rhs: Self, tolerance: Self) -> bool {
		tolerance >= Self::ZERO && (self.0 as i128 - rhs.0 as i128).unsigned_abs() <= tolerance.0 as u128
	}
}

impl fmt::Display for Fixed {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { fmt::Display::fmt(&I40F24::from_bits(self.0), f) }
}
impl fmt::Debug for Fixed {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result { fmt::Display::fmt(self, f) }
}
impl Add for Fixed { type Output = Self; fn add(self, rhs: Self) -> Self { self.checked_add(rhs).expect("fixed addition overflow") } }
impl Sub for Fixed { type Output = Self; fn sub(self, rhs: Self) -> Self { self.checked_sub(rhs).expect("fixed subtraction overflow") } }
impl Mul for Fixed { type Output = Self; fn mul(self, rhs: Self) -> Self { self.checked_mul(rhs).expect("fixed multiplication overflow") } }
impl Div for Fixed { type Output = Self; fn div(self, rhs: Self) -> Self { self.checked_div(rhs).expect("fixed division overflow or zero divisor") } }
impl Rem for Fixed { type Output = Self; fn rem(self, rhs: Self) -> Self { Self::from_wide((self.0 as i128).checked_rem(rhs.0 as i128).expect("fixed remainder with zero divisor")) } }
impl Neg for Fixed { type Output = Self; fn neg(self) -> Self { Self(self.0.checked_neg().expect("fixed negation overflow")) } }
macro_rules! assign {
	($trait:ident, $method:ident, $op:tt) => {
		impl $trait for Fixed { fn $method(&mut self, rhs: Self) { *self = *self $op rhs; } }
	};
}
assign!(AddAssign, add_assign, +);
assign!(SubAssign, sub_assign, -);
assign!(MulAssign, mul_assign, *);
assign!(DivAssign, div_assign, /);
assign!(RemAssign, rem_assign, %);
impl Sum for Fixed { fn sum<I: Iterator<Item = Self>>(iter: I) -> Self { iter.fold(Self::ZERO, |a, b| a + b) } }
impl<'a> Sum<&'a Self> for Fixed { fn sum<I: Iterator<Item = &'a Self>>(iter: I) -> Self { iter.copied().sum() } }
impl Product for Fixed { fn product<I: Iterator<Item = Self>>(iter: I) -> Self { iter.fold(Self::ONE, |a, b| a * b) } }
