use std::{fmt::Debug, hash::Hash};

use bevy::math::{IVec3, UVec3};

/// A three-dimensional integer region with a signed minimum and unsigned size.
pub trait Region: Clone + Copy + Debug + PartialEq + Eq + Hash {
	/// The possibly zero-sized form of this region's coordinate unit.
	type Zero: Region<Zero = Self::Zero, NonZero = Self::NonZero> + From<Self::NonZero>;
	/// The form of this region's coordinate unit that is nonzero on every axis.
	type NonZero: NonZeroRegion<Zero = Self::Zero, NonZero = Self::NonZero> + TryFrom<Self::Zero>;

	fn min(&self) -> IVec3;
	fn size(&self) -> UVec3;
	fn contains(&self, position: IVec3) -> bool;
	fn contains_region<R: Region<Zero = Self::Zero, NonZero = Self::NonZero>>(&self, other: R) -> bool;
	fn intersects<R: Region<Zero = Self::Zero, NonZero = Self::NonZero>>(&self, other: R) -> bool;
	fn intersection<R: Region<Zero = Self::Zero, NonZero = Self::NonZero>>(&self, other: R) -> Option<Self::NonZero>;
	fn translated(&self, offset: IVec3) -> Self;
}

/// A region whose size is nonzero on every axis.
pub trait NonZeroRegion: Region<NonZero = Self> {}

/// Returned when a nonzero region is constructed with a zero-sized axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ZeroSizedRegionError;

impl std::fmt::Display for ZeroSizedRegionError {
	fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		formatter.write_str("region size must be nonzero on every axis")
	}
}

impl std::error::Error for ZeroSizedRegionError {}

#[doc(hidden)]
pub fn region_end(region: &impl Region) -> IVec3 {
	region.min() + region.size().as_ivec3()
}

#[macro_export]
macro_rules! define_region_types {
	($region:ident, $nonzero_region:ident) => {
		#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, ::serde::Serialize, ::serde::Deserialize)]
		pub struct $region {
			min: ::bevy::math::IVec3,
			size: ::bevy::math::UVec3,
		}

		impl $region {
			pub const fn new(min: ::bevy::math::IVec3, size: ::bevy::math::UVec3) -> Self {
				Self { min, size }
			}

			pub fn from_min_end(min: ::bevy::math::IVec3, end: ::bevy::math::IVec3) -> Option<Self> {
				min.cmple(end).all().then(|| Self { min, size: (end - min).as_uvec3() })
			}

			pub fn from_min_max_inclusive(min: ::bevy::math::IVec3, max: ::bevy::math::IVec3) -> Option<Self> {
				Self::from_min_end(min, max + ::bevy::math::IVec3::ONE)
			}

			pub const fn min(self) -> ::bevy::math::IVec3 {
				self.min
			}

			pub const fn size(self) -> ::bevy::math::UVec3 {
				self.size
			}

			pub fn end(self) -> ::bevy::math::IVec3 {
				self.min + self.size.as_ivec3()
			}

			pub const fn is_empty(self) -> bool {
				self.size.x == 0 || self.size.y == 0 || self.size.z == 0
			}

			pub fn contains(self, position: ::bevy::math::IVec3) -> bool {
				<$region as $crate::region::Region>::contains(&self, position)
			}

			pub fn contains_region<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(self, other: R) -> bool {
				<$region as $crate::region::Region>::contains_region(&self, other)
			}

			pub fn intersects<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(self, other: R) -> bool {
				<$region as $crate::region::Region>::intersects(&self, other)
			}

			pub fn intersection<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(self, other: R) -> Option<$nonzero_region> {
				<$region as $crate::region::Region>::intersection(&self, other)
			}

			pub fn translated(self, offset: ::bevy::math::IVec3) -> Self {
				<$region as $crate::region::Region>::translated(&self, offset)
			}
		}

		impl $crate::region::Region for $region {
			type Zero = $region;
			type NonZero = $nonzero_region;

			fn min(&self) -> ::bevy::math::IVec3 {
				self.min
			}

			fn size(&self) -> ::bevy::math::UVec3 {
				self.size
			}

			fn contains(&self, position: ::bevy::math::IVec3) -> bool {
				position.cmpge(self.min).all() && position.cmplt($crate::region::region_end(self)).all()
			}

			fn contains_region<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(&self, other: R) -> bool {
				other.min().cmpge(self.min).all() && $crate::region::region_end(&other).cmple($crate::region::region_end(self)).all()
			}

			fn intersects<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(&self, other: R) -> bool {
				self.min.cmplt($crate::region::region_end(&other)).all() && other.min().cmplt($crate::region::region_end(self)).all()
			}

			fn intersection<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(&self, other: R) -> Option<$nonzero_region> {
				let min = self.min.max(other.min());
				let end = $crate::region::region_end(self).min($crate::region::region_end(&other));
				$nonzero_region::from_min_end(min, end)
			}

			fn translated(&self, offset: ::bevy::math::IVec3) -> Self {
				Self { min: self.min + offset, size: self.size }
			}
		}

		#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, ::serde::Serialize)]
		pub struct $nonzero_region {
			min: ::bevy::math::IVec3,
			size: ::bevy::math::UVec3,
		}

		impl $nonzero_region {
			pub fn new(min: ::bevy::math::IVec3, size: ::bevy::math::UVec3) -> Option<Self> {
				(size.x != 0 && size.y != 0 && size.z != 0).then_some(Self { min, size })
			}

			pub fn from_min_size(min: ::bevy::math::IVec3, size: ::bevy::math::IVec3) -> Option<Self> {
				(size.cmpgt(::bevy::math::IVec3::ZERO).all()).then(|| Self { min, size: size.as_uvec3() })
			}

			pub fn from_min_end(min: ::bevy::math::IVec3, end: ::bevy::math::IVec3) -> Option<Self> {
				min.cmplt(end).all().then(|| Self { min, size: (end - min).as_uvec3() })
			}

			pub fn from_min_max_inclusive(min: ::bevy::math::IVec3, max: ::bevy::math::IVec3) -> Option<Self> {
				Self::from_min_end(min, max + ::bevy::math::IVec3::ONE)
			}

			pub const fn min(self) -> ::bevy::math::IVec3 {
				self.min
			}

			pub const fn size(self) -> ::bevy::math::UVec3 {
				self.size
			}

			pub fn end(self) -> ::bevy::math::IVec3 {
				self.min + self.size.as_ivec3()
			}

			pub fn max_inclusive(self) -> ::bevy::math::IVec3 {
				self.end() - ::bevy::math::IVec3::ONE
			}

			pub fn contains(self, position: ::bevy::math::IVec3) -> bool {
				<$nonzero_region as $crate::region::Region>::contains(&self, position)
			}

			pub fn contains_region<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(self, other: R) -> bool {
				<$nonzero_region as $crate::region::Region>::contains_region(&self, other)
			}

			pub fn intersects<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(self, other: R) -> bool {
				<$nonzero_region as $crate::region::Region>::intersects(&self, other)
			}

			pub fn intersection<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(self, other: R) -> Option<$nonzero_region> {
				<$nonzero_region as $crate::region::Region>::intersection(&self, other)
			}

			pub fn translated(self, offset: ::bevy::math::IVec3) -> Self {
				<$nonzero_region as $crate::region::Region>::translated(&self, offset)
			}
		}

		impl $crate::region::Region for $nonzero_region {
			type Zero = $region;
			type NonZero = $nonzero_region;

			fn min(&self) -> ::bevy::math::IVec3 {
				self.min
			}

			fn size(&self) -> ::bevy::math::UVec3 {
				self.size
			}

			fn contains(&self, position: ::bevy::math::IVec3) -> bool {
				position.cmpge(self.min).all() && position.cmplt($crate::region::region_end(self)).all()
			}

			fn contains_region<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(&self, other: R) -> bool {
				other.min().cmpge(self.min).all() && $crate::region::region_end(&other).cmple($crate::region::region_end(self)).all()
			}

			fn intersects<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(&self, other: R) -> bool {
				self.min.cmplt($crate::region::region_end(&other)).all() && other.min().cmplt($crate::region::region_end(self)).all()
			}

			fn intersection<R: $crate::region::Region<Zero = $region, NonZero = $nonzero_region>>(&self, other: R) -> Option<$nonzero_region> {
				let min = self.min.max(other.min());
				let end = $crate::region::region_end(self).min($crate::region::region_end(&other));
				$nonzero_region::from_min_end(min, end)
			}

			fn translated(&self, offset: ::bevy::math::IVec3) -> Self {
				Self { min: self.min + offset, size: self.size }
			}
		}

		impl $crate::region::NonZeroRegion for $nonzero_region {}

		impl<'de> ::serde::Deserialize<'de> for $nonzero_region {
			fn deserialize<D: ::serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
				let region = <$region as ::serde::Deserialize>::deserialize(deserializer)?;
				Self::try_from(region).map_err(::serde::de::Error::custom)
			}
		}

		impl ::core::convert::TryFrom<$region> for $nonzero_region {
			type Error = $crate::region::ZeroSizedRegionError;

			fn try_from(region: $region) -> Result<Self, Self::Error> {
				Self::new(region.min, region.size).ok_or($crate::region::ZeroSizedRegionError)
			}
		}

		impl From<$nonzero_region> for $region {
			fn from(region: $nonzero_region) -> Self {
				Self { min: region.min, size: region.size }
			}
		}
	};
}

define_region_types!(VoxelRegion, NonZeroVoxelRegion);
