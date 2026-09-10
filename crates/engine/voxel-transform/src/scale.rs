use std::{num::NonZeroU32, ops::Mul};
use fixed::traits::ToFixed;
use serde::{Deserialize, Serialize};
use voxel_math::Fixed;

/// Positive uniform scale in unsigned Q16.16.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Scale(NonZeroU32);

fn rounded_div(n: u64, d: u64) -> u64 {
	let q = n / d;
	let r = n % d;
	q + u64::from(r * 2 > d || (r * 2 == d && q & 1 != 0))
}

impl Scale {
	pub const ONE: Self = Self(NonZeroU32::new(1 << 16).unwrap());

	pub const fn from_bits(bits: u32) -> Option<Self> {
		match NonZeroU32::new(bits) { Some(bits) => Some(Self(bits)), None => None }
	}
	pub const fn to_bits(self) -> u32 { self.0.get() }
	pub const fn to_fixed(self) -> Fixed { Fixed::from_bits((self.to_bits() as i64) << 8) }

	pub fn from_num<T: ToFixed>(value: T) -> Self { Self::from_fixed(Fixed::from_num(value)) }
	pub fn from_fixed(value: Fixed) -> Self {
		Self::checked_from_fixed(value).expect("scale must round to a positive Q16.16 value without overflow")
	}
	/// Round to nearest, choosing the even value at a tie.
	pub fn checked_from_fixed(value: Fixed) -> Option<Self> {
		let bits = u64::try_from(value.to_bits()).ok()?;
		Self::from_bits(u32::try_from(rounded_div(bits, 1 << 8)).ok()?)
	}
	pub fn checked_mul(self, rhs: Self) -> Option<Self> {
		let bits = rounded_div(self.to_bits() as u64 * rhs.to_bits() as u64, 1 << 16);
		Self::from_bits(u32::try_from(bits).ok()?)
	}
	pub fn checked_recip(self) -> Option<Self> {
		Self::from_bits(u32::try_from(rounded_div(1 << 32, self.to_bits() as u64)).ok()?)
	}
	pub fn recip(self) -> Self { self.checked_recip().expect("inverse scale is outside Q16.16 range") }
}

impl Default for Scale { fn default() -> Self { Self::ONE } }
impl From<Fixed> for Scale { fn from(value: Fixed) -> Self { Self::from_fixed(value) } }
impl From<Scale> for Fixed { fn from(value: Scale) -> Self { value.to_fixed() } }
impl Mul for Scale {
	type Output = Self;
	fn mul(self, rhs: Self) -> Self { self.checked_mul(rhs).expect("scale composition underflow or overflow") }
}
