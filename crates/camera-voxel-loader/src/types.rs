use bevy::{ecs::entity::Entity, math::IVec3};
use voxel_data::grid::GridId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct ChunkKey {
	pub(crate) grid: GridId,
	pub(crate) chunk: IVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PolicyDebugBoxKind {
	NearChunks,
	LodOuter(u8),
	LodInner(u8),
	LodNearExclusion(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PolicyDebugBox {
	pub(crate) grid: GridId,
	pub(crate) min: IVec3,
	pub(crate) max: IVec3,
	pub(crate) entering: bool,
	pub(crate) kind: PolicyDebugBoxKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct TileKey {
	pub(crate) grid: GridId,
	pub(crate) lod: u8,
	pub(crate) min: IVec3,
}

impl TileKey {
	pub(crate) fn size(self) -> IVec3 {
		IVec3::splat(1i32 << self.lod)
	}
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TileRecord {
	pub(crate) status: TileStatus,
	pub(crate) entity: Option<Entity>,
	pub(crate) generation: u64,
	pub(crate) stale_entity: Option<Entity>,
}

impl TileRecord {
	pub(crate) fn queued() -> Self {
		Self { status: TileStatus::Queued, entity: None, generation: 0, stale_entity: None }
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum TileStatus {
	Queued,
	Loading,
	LoadedWaitingGpu,
	Ready,
	Retiring,
}
