use bevy::ecs::entity::{EntityMapper, MapEntities};
use bevy::prelude::Event;
use serde::{Deserialize, Serialize};
use voxel_data::grid::GridId;
use voxel_data::region::NonZeroVoxelRegion;
use voxel_data::voxels::Voxel;
use tile_data::{ChunkRegion, NonZeroChunkRegion};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) enum WireGridEdit {
	AddArea { region: NonZeroVoxelRegion, voxel: Voxel },
	RemoveArea { region: NonZeroVoxelRegion },
}

impl WireGridEdit {
	pub(super) fn from_edit(edit: &ResolvedGridEdit) -> Option<Self> {
		match edit {
			ResolvedGridEdit::AddArea { region, voxel } => Some(Self::AddArea { region: *region, voxel: voxel.clone() }),
			ResolvedGridEdit::RemoveArea { region } => Some(Self::RemoveArea { region: *region }),
		}
	}

	pub(super) fn into_edit(self) -> ResolvedGridEdit {
		match self {
			Self::AddArea { region, voxel } => ResolvedGridEdit::AddArea { region, voxel },
			Self::RemoveArea { region } => ResolvedGridEdit::RemoveArea { region },
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
	pub region: NonZeroChunkRegion,
	pub stream_sequence: u64,
	pub generation: u64,
	pub edit: WireGridEdit,
}

impl MapEntities for RemoteGridEdit {
	fn map_entities<M: EntityMapper>(&mut self, mapper: &mut M) { self.grid = mapper.get_mapped(self.grid); }
}
