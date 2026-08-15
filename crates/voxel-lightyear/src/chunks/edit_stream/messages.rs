use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::math::IVec3;
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::grid::GridId;
use voxel_data::region::NonZeroVoxelRegion;
use voxel_data::voxels::Voxel;
use voxel_edit::GridEdit;
use tile_data::ChunkRegion;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) enum WireGridEdit {
	Add { voxel_pos: IVec3, voxel: Voxel },
	Remove { voxel_pos: IVec3 },
	AddArea { region: NonZeroVoxelRegion, voxel: Voxel },
	RemoveArea { region: NonZeroVoxelRegion },
}

impl WireGridEdit {
	pub(super) fn from_edit(edit: &GridEdit) -> Option<Self> {
		match edit {
			GridEdit::Add { voxel_pos, voxel } => Some(Self::Add { voxel_pos: *voxel_pos, voxel: voxel.clone() }),
			GridEdit::Remove { voxel_pos } => Some(Self::Remove { voxel_pos: *voxel_pos }),
			GridEdit::AddArea { region, voxel } => Some(Self::AddArea { region: *region, voxel: voxel.clone() }),
			GridEdit::RemoveArea { region } => Some(Self::RemoveArea { region: *region }),
			GridEdit::ApplySdf { .. } | GridEdit::ClearSdf { .. } => None,
		}
	}

	pub(super) fn into_edit(self) -> GridEdit {
		match self {
			Self::Add { voxel_pos, voxel } => GridEdit::Add { voxel_pos, voxel },
			Self::Remove { voxel_pos } => GridEdit::Remove { voxel_pos },
			Self::AddArea { region, voxel } => GridEdit::AddArea { region, voxel },
			Self::RemoveArea { region } => GridEdit::RemoveArea { region },
		}
	}
}

#[derive(Event, Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct EditInterest {
	pub grid: GridId,
	pub region: ChunkRegion,
	pub version: u64,
	pub interested: bool,
}

impl MapEntities for EditInterest {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}

#[derive(Event, Clone, Copy, Debug, Serialize, Deserialize)]
pub(super) struct EditStreamStart {
	pub grid: GridId,
	pub first_stream_sequence: u64,
}

impl MapEntities for EditStreamStart {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}

#[derive(Event, Clone, Debug, Serialize, Deserialize)]
pub(super) struct RemoteGridEdit {
	pub grid: GridId,
	pub region: ChunkRegion,
	pub stream_sequence: u64,
	pub generation: u64,
	pub edit: WireGridEdit,
}

impl MapEntities for RemoteGridEdit {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}
