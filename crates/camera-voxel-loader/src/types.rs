use bevy::math::IVec3;
use tile_data::TileIndexKey;
use voxel_data::grid::GridId;
use voxel_streaming::TileClassId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileKey { pub(crate) grid: GridId, pub(crate) class: TileClassId, pub(crate) lod: u8, pub(crate) min: IVec3 }

impl TileKey {
	pub(crate) fn size(self) -> IVec3 { IVec3::splat(1i32 << self.lod) }
	pub(crate) fn streaming_key(self) -> voxel_streaming::TileKey {
		voxel_streaming::TileKey { min: self.min, size: self.size(), lod: self.lod, class: self.class }
	}
}

impl TileIndexKey for TileKey {
	fn lod(self) -> u8 { self.lod }
	fn min(self) -> IVec3 { self.min }
	fn size(self) -> IVec3 { self.size() }
}
