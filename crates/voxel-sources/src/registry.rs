use std::collections::HashMap;

use bevy::math::IVec3;
use bevy::ecs::resource::Resource;
use crossbeam_channel::{unbounded, Receiver, Sender};

use voxel_data::grid::GridId;

use crate::handle::{SourceEvent, SourceResult};
use crate::source::{ChunkSource, GridKey, SourceId};

#[derive(Resource)]
pub struct SourceRegistry {
	pub(crate) sources: Vec<Box<dyn ChunkSource>>,
	pub(crate) keys: HashMap<GridKey, GridId>,
	pub(crate) event_tx: Sender<SourceEvent>,
	pub(crate) event_rx: Receiver<SourceEvent>,
	pub(crate) result_tx: Sender<SourceResult>,
	pub(crate) result_rx: Receiver<SourceResult>,
}

impl Default for SourceRegistry {
	fn default() -> Self {
		let (event_tx, event_rx) = unbounded();
		let (result_tx, result_rx) = unbounded();
		Self { sources: Vec::new(), keys: HashMap::new(), event_tx, event_rx, result_tx, result_rx }
	}
}

impl SourceRegistry {
	pub(crate) fn push(&mut self, source: Box<dyn ChunkSource>) {
		self.sources.push(source);
	}

	pub(crate) fn entity(&self, grid: GridKey) -> Option<GridId> {
		self.keys.get(&grid).copied()
	}

	pub(crate) fn cheapest(&self, grid: GridKey, chunk: IVec3) -> Option<SourceId> {
		self.sources
			.iter()
			.enumerate()
			.filter_map(|(i, s)| s.cost(grid, chunk).map(|c| (c, SourceId(i))))
			.min_by_key(|(c, _)| *c)
			.map(|(_, id)| id)
	}

	pub(crate) fn forget_others(&mut self, keep: SourceId, grid: GridKey, chunk: IVec3) {
		for (i, source) in self.sources.iter_mut().enumerate() {
			if i != keep.0 {
				source.forget(grid, chunk);
			}
		}
	}
}
