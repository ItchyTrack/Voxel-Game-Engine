use bevy::math::{IVec3, UVec3};

use crate::{NonZeroChunkRegion, TileClassId};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileKey {
	pub region: NonZeroChunkRegion,
	pub lod: u8,
	pub class: TileClassId,
}

impl TileKey {
	pub fn new(min: IVec3, size: UVec3, lod: u8, class: TileClassId) -> Option<Self> {
		Some(Self { region: NonZeroChunkRegion::new(min, size)?, lod, class })
	}

	pub const fn min(self) -> IVec3 { self.region.min() }
	pub const fn size(self) -> UVec3 { self.region.size() }
}
