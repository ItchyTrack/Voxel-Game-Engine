use std::collections::{HashMap, HashSet};
use std::mem;

use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

use crate::loader::{ChunkLoadRequest, ChunkLoaderChannel};
use crate::presence::ChunkPresence;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ChunkState {
	InFlight,
	Loaded,
	Empty,
}

#[derive(Component, Default)]
pub struct GridStreaming {
	presence: ChunkPresence,
	states: HashMap<IVec3, ChunkState>,
	staged: Vec<(IVec3, Voxels)>,
}

impl GridStreaming {
	pub fn presence(&self) -> &ChunkPresence { &self.presence }
	pub fn presence_mut(&mut self) -> &mut ChunkPresence { &mut self.presence }

	pub fn state(&self, chunk: IVec3) -> Option<ChunkState> {
		self.states.get(&chunk).copied()
	}

	pub fn is_loaded(&self, chunk: IVec3) -> bool {
		matches!(self.states.get(&chunk), Some(ChunkState::Loaded | ChunkState::Empty))
	}

	/// Drains freshly-loaded chunk voxels awaiting handoff into the grid's sub-grids.
	pub fn take_staged(&mut self) -> Vec<(IVec3, Voxels)> {
		mem::take(&mut self.staged)
	}

	fn try_begin_load(&mut self, chunk: IVec3) -> bool {
		if self.states.contains_key(&chunk) || !self.presence.is_present(chunk) {
			return false;
		}
		self.states.insert(chunk, ChunkState::InFlight);
		true
	}

	fn apply_result(&mut self, chunk: IVec3, voxels: Option<Voxels>) {
		match voxels {
			Some(v) => {
				self.presence.mark_present(chunk);
				self.staged.push((chunk, v));
				self.states.insert(chunk, ChunkState::Loaded);
			}
			None => {
				self.presence.clear_present(chunk);
				self.states.insert(chunk, ChunkState::Empty);
			}
		}
	}
}

#[derive(Resource, Default)]
pub struct ChunkRequests {
	needed: HashMap<GridId, HashSet<IVec3>>,
	prefetch: HashMap<GridId, HashSet<IVec3>>,
}

impl ChunkRequests {
	pub fn request_needed(&mut self, grid: GridId, chunk: IVec3) {
		self.needed.entry(grid).or_default().insert(chunk);
	}

	pub fn request_prefetch(&mut self, grid: GridId, chunk: IVec3) {
		self.prefetch.entry(grid).or_default().insert(chunk);
	}

	fn clear(&mut self) {
		self.needed.clear();
		self.prefetch.clear();
	}
}

#[derive(Resource, Default)]
pub struct ChunkReadiness {
	ready: bool,
}

impl ChunkReadiness {
	pub fn ready(&self) -> bool { self.ready }
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamingSet {
	Receive,
	Clear,
	Collect,
	Emit,
}

pub fn chunks_ready(readiness: Res<ChunkReadiness>) -> bool {
	readiness.ready()
}

pub(crate) fn receive_results(channel: Res<ChunkLoaderChannel>, mut grids: Query<&mut GridStreaming>) {
	while let Some(result) = channel.try_recv() {
		if let Ok(mut streaming) = grids.get_mut(result.grid) {
			streaming.apply_result(result.chunk, result.voxels);
		}
	}
}

pub(crate) fn clear_requests(mut requests: ResMut<ChunkRequests>) {
	requests.clear();
}

pub(crate) fn emit_requests(
	requests: Res<ChunkRequests>,
	mut grids: Query<&mut GridStreaming>,
	mut writer: MessageWriter<ChunkLoadRequest>,
	mut readiness: ResMut<ChunkReadiness>,
) {
	for (grid, chunks) in requests.needed.iter().chain(requests.prefetch.iter()) {
		let Ok(mut streaming) = grids.get_mut(*grid) else { continue };
		for &chunk in chunks {
			if streaming.try_begin_load(chunk) {
				writer.write(ChunkLoadRequest { grid: *grid, chunk });
			}
		}
	}

	let mut ready = true;
	'outer: for (grid, chunks) in requests.needed.iter() {
		let Ok(streaming) = grids.get(*grid) else { ready = false; break };
		for &chunk in chunks {
			if streaming.presence().is_present(chunk) && !streaming.is_loaded(chunk) {
				ready = false;
				break 'outer;
			}
		}
	}
	readiness.ready = ready;
}
