use bevy::math::IVec3;
use tile_data::{NonZeroChunkRegion, TileIndexKey, TileKey};
use voxel_data::grid::GridId;
use tile_data::TileClassId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct GridTileKey {
	pub(crate) grid: GridId,
	pub(crate) tile_key: TileKey,
}

impl GridTileKey {
	pub(crate) fn new(grid: GridId, class: TileClassId, lod: u8, min: IVec3) -> Self {
		let size = 1u32 << lod;
		Self {
			grid,
			tile_key: TileKey {
				class,
				lod,
				region: NonZeroChunkRegion::new(min, bevy::math::UVec3::splat(size)).unwrap()
			}
		}
	}
}

impl TileIndexKey for GridTileKey {
	fn region(self) -> NonZeroChunkRegion { self.tile_key.region }
}
