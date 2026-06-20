use bevy::math::IVec3;
use crossbeam_channel::Sender;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

use crate::source::SourceId;

pub enum SourceEvent {
	Available { grid: GridId, chunk: IVec3 },
	AvailableArea { grid: GridId, min: IVec3, size: IVec3 },
	Unavailable { grid: GridId, chunk: IVec3 },
	Edited { source: SourceId, grid: GridId, chunk: IVec3 },
}

pub struct SourceResult {
	pub grid: GridId,
	pub chunk: IVec3,
	pub voxels: Option<Voxels>,
}

pub struct SourceLodResult {
	pub source: SourceId,
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub lod: f32,
	pub voxels: Option<Voxels>,
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(crate) id: SourceId,
	pub(crate) events: Sender<SourceEvent>,
	pub(crate) results: Sender<SourceResult>,
	pub(crate) lod_results: Sender<SourceLodResult>,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId {
		self.id
	}

	/// Finished a load. `None` voxels = confirmed empty.
	pub fn loaded(&self, grid: GridId, chunk: IVec3, voxels: Option<Voxels>) {
		let _ = self.results.send(SourceResult { grid, chunk, voxels });
	}

	pub fn loaded_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, voxels: Option<Voxels>) {
		let _ = self.lod_results.send(SourceLodResult { source: self.id, grid, min, size, lod, voxels });
	}

	pub fn available(&self, grid: GridId, chunk: IVec3) {
		let _ = self.events.send(SourceEvent::Available { grid, chunk });
	}

	pub fn available_area(&self, grid: GridId, min: IVec3, size: IVec3) {
		let _ = self.events.send(SourceEvent::AvailableArea { grid, min, size });
	}

	pub fn unavailable(&self, grid: GridId, chunk: IVec3) {
		let _ = self.events.send(SourceEvent::Unavailable { grid, chunk });
	}

	pub fn edited(&self, grid: GridId, chunk: IVec3) {
		let _ = self.events.send(SourceEvent::Edited { source: self.id, grid, chunk });
	}
}
