use bevy::math::IVec3;
use crossbeam_channel::Sender;

use voxel_data::voxels::Voxels;

use crate::source::{GridKey, SourceId};

pub(crate) enum SourceEvent {
	Available { grid: GridKey, chunk: IVec3 },
	AvailableArea { grid: GridKey, min: IVec3, size: IVec3 },
	Unavailable { grid: GridKey, chunk: IVec3 },
	Edited { source: SourceId, grid: GridKey, chunk: IVec3 },
}

pub(crate) struct SourceResult {
	pub grid: GridKey,
	pub chunk: IVec3,
	pub voxels: Option<Voxels>,
}

#[derive(Clone)]
pub struct SourceHandle {
	pub(crate) id: SourceId,
	pub(crate) events: Sender<SourceEvent>,
	pub(crate) results: Sender<SourceResult>,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId {
		self.id
	}

	/// Finished a load. `None` voxels = confirmed empty.
	pub fn loaded(&self, grid: GridKey, chunk: IVec3, voxels: Option<Voxels>) {
		let _ = self.results.send(SourceResult { grid, chunk, voxels });
	}

	pub fn available(&self, grid: GridKey, chunk: IVec3) {
		let _ = self.events.send(SourceEvent::Available { grid, chunk });
	}

	pub fn available_area(&self, grid: GridKey, min: IVec3, size: IVec3) {
		let _ = self.events.send(SourceEvent::AvailableArea { grid, min, size });
	}

	pub fn unavailable(&self, grid: GridKey, chunk: IVec3) {
		let _ = self.events.send(SourceEvent::Unavailable { grid, chunk });
	}

	pub fn edited(&self, grid: GridKey, chunk: IVec3) {
		let _ = self.events.send(SourceEvent::Edited { source: self.id, grid, chunk });
	}
}
