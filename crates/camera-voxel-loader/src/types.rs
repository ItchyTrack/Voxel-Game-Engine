use bevy::{ecs::entity::Entity, math::IVec3};
use voxel_data::grid::GridId;
use voxel_streaming::TileIndexKey;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileKey { pub(crate) grid: GridId, pub(crate) lod: u8, pub(crate) min: IVec3 }

impl TileKey {
	pub(crate) fn chunk(grid: GridId, chunk: IVec3) -> Self { Self { grid, lod: 0, min: chunk } }
	pub(crate) fn size(self) -> IVec3 { IVec3::splat(1i32 << self.lod) }
	pub(crate) fn is_chunk(self) -> bool { self.lod == 0 }
}

impl TileIndexKey for TileKey {
	fn lod(self) -> u8 { self.lod }
	fn min(self) -> IVec3 { self.min }
	fn size(self) -> IVec3 { self.size() }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TileRecord {
	pub(crate) status: TileStatus,
	pub(crate) entity: Option<Entity>,
}

impl TileRecord {
	pub(crate) fn queued() -> Self { Self { status: TileStatus::Queued, entity: None } }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TileStatus { Queued, Loading, Ready, Retiring }
