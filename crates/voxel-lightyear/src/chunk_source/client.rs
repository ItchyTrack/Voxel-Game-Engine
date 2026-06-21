use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_sources::{ChunkSource, LodKey, SourceHandle};

use crate::chunk_source::{ChunkPresenceAabb, ChunkRequest, ChunkResponse, LodRequest, LodResponse, PresenceRequest};

const REMOTE_COST: u32 = 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingChunk {
	grid: GridId,
	chunk: IVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingLod {
	grid: GridId,
	key: LodKey,
}

#[derive(Default)]
struct ClientChunkSourceState {
	handle: OnceLock<SourceHandle>,
	presence_requests: Mutex<VecDeque<PresenceRequest>>,
	chunk_requests: Mutex<VecDeque<ChunkRequest>>,
	lod_requests: Mutex<VecDeque<LodRequest>>,
	pending_chunks: Mutex<HashSet<PendingChunk>>,
	pending_lods: Mutex<HashSet<PendingLod>>,
}

#[derive(Clone, Default, Resource)]
pub struct ClientChunkSource {
	state: Arc<ClientChunkSourceState>,
}

impl ChunkSource for ClientChunkSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.state.handle.set(handle);
	}

	fn request_available_area(&self, grid: GridId) {
		self.state.presence_requests.lock().unwrap().push_back(PresenceRequest { grid });
	}

	fn cost(&self, _grid: GridId, _chunk: IVec3) -> Option<u32> {
		Some(REMOTE_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3) {
		let pending = PendingChunk { grid, chunk };
		self.state.pending_chunks.lock().unwrap().insert(pending);
		self.state.chunk_requests.lock().unwrap().push_back(ChunkRequest { grid, chunk });
	}

	fn cost_lod(&self, _grid: GridId, _min: IVec3, _size: IVec3, _lod: f32) -> Option<u32> {
		Some(REMOTE_COST)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32) {
		let key = LodKey { min, size, lod: lod.max(0.0).floor() as u8 };
		let pending = PendingLod { grid, key };
		self.state.pending_lods.lock().unwrap().insert(pending);
		self.state.lod_requests.lock().unwrap().push_back(LodRequest { grid, key, priority: 0.0 });
	}
}

pub(crate) fn flush_remote_presence_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<PresenceRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.presence_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<super::PresenceRequestChannel>(request);
	}
}

pub(crate) fn flush_remote_chunk_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<ChunkRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.chunk_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<super::ChunkRequestChannel>(request);
	}
}

pub(crate) fn flush_remote_lod_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<LodRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.lod_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<super::LodRequestChannel>(request);
	}
}

pub(crate) fn receive_remote_chunk_response(
	mut trigger: On<RemoteEvent<ChunkResponse>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let from = trigger.event().from;
	let response = &mut trigger.event_mut().trigger;
	let pending = PendingChunk { grid: response.grid, chunk: response.chunk };
	if !source.state.pending_chunks.lock().unwrap().remove(&pending) {
		warn!(grid=?response.grid, chunk=?response.chunk, ?from, "ignoring unexpected remote chunk response");
		return;
	}
	let voxels = match response.voxels.take() {
		Some(voxels) => match voxels.decompress() {
			Ok(voxels) => Some(voxels),
			Err(err) => {
				warn!(grid=?response.grid, chunk=?response.chunk, ?from, error=%err, "failed to decompress remote chunk response");
				return;
			}
		},
		None => None,
	};
	handle.loaded(response.grid, response.chunk, voxels);
}

pub(crate) fn receive_remote_lod_response(
	mut trigger: On<RemoteEvent<LodResponse>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let from = trigger.event().from;
	let response = &mut trigger.event_mut().trigger;
	let pending = PendingLod { grid: response.grid, key: response.key };
	if !source.state.pending_lods.lock().unwrap().remove(&pending) {
		warn!(grid=?response.grid, min=?response.key.min, size=?response.key.size, lod=response.key.lod, ?from, "ignoring unexpected remote lod response");
		return;
	}
	let voxels = match response.voxels.take() {
		Some(voxels) => match voxels.decompress() {
			Ok(voxels) => Some(voxels),
			Err(err) => {
				warn!(grid=?response.grid, min=?response.key.min, size=?response.key.size, lod=response.key.lod, ?from, error=%err, "failed to decompress remote lod response");
				return;
			}
		},
		None => None,
	};
	handle.loaded_lod(response.grid, response.key.min, response.key.size, response.key.lod as f32, voxels);
}

pub(crate) fn receive_chunk_presence_aabb(
	trigger: On<RemoteEvent<ChunkPresenceAabb>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event().trigger;
	handle.available_area(event.grid, event.min, event.size);
}
