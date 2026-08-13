use bevy::math::IVec3;
use tile_data::{NonZeroChunkRegion, TileIndexKey};
use voxel_data::grid::GridId;
use voxel_streaming::TileClassId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileKey {
	pub(crate) grid: GridId,
	pub(crate) class: TileClassId,
	pub(crate) lod: u8,
	pub(crate) region: NonZeroChunkRegion,
}

impl TileKey {
	pub(crate) fn new(grid: GridId, class: TileClassId, lod: u8, min: IVec3) -> Self {
		let size = 1u32 << lod;
		Self { grid, class, lod, region: NonZeroChunkRegion::new(min, bevy::math::UVec3::splat(size)).unwrap() }
	}

	pub(crate) fn min(self) -> IVec3 { self.region.min() }
	pub(crate) fn size(self) -> IVec3 { self.region.size().as_ivec3() }
	pub(crate) fn streaming_key(self) -> voxel_streaming::TileKey {
		voxel_streaming::TileKey { region: self.region, lod: self.lod, class: self.class }
	}
}

impl TileIndexKey for TileKey {
	fn min(self) -> IVec3 { self.region.min() }
	fn size(self) -> IVec3 { self.region.size().as_ivec3() }
}
