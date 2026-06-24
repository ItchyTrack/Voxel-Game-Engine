use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_sources::{ChunkSource, LodKey, SourceHandle};

use super::remote_generations::RemoteGenerations;
use crate::chunks::{ChunkRequest, LodRequest, PresenceRequest};

const REMOTE_COST: u32 = 100;

#[derive(Default)]
pub(super) struct ClientChunkSourceState {
	pub handle: OnceLock<SourceHandle>,
	pub presence_requests: Mutex<VecDeque<PresenceRequest>>,
	pub chunk_requests: Mutex<VecDeque<ChunkRequest>>,
	pub lod_requests: Mutex<VecDeque<LodRequest>>,
	pub remote_generations: Mutex<RemoteGenerations>,
	pub remote_grids: Mutex<HashSet<GridId>>,
}

#[derive(Clone, Default, Resource)]
pub struct ClientChunkSource {
	pub(super) state: Arc<ClientChunkSourceState>,
}

impl ChunkSource for ClientChunkSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.state.handle.set(handle);
	}

	fn request_available_area(&self, grid: GridId) {
		self.state.presence_requests.lock().unwrap().push_back(PresenceRequest { grid });
	}

	fn cost(&self, grid: GridId, _chunk: IVec3) -> Option<u32> {
		self.state.remote_grids.lock().unwrap().contains(&grid).then_some(REMOTE_COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64) {
		let mut remote_generations = self.state.remote_generations.lock().unwrap();
		let mut requests = self.state.chunk_requests.lock().unwrap();
		remote_generations.request_chunk(&mut requests, grid, chunk, generation);
	}

	fn cost_lod(&self, grid: GridId, _min: IVec3, _size: IVec3, _lod: f32) -> Option<u32> {
		self.state.remote_grids.lock().unwrap().contains(&grid).then_some(REMOTE_COST)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64) {
		let key = LodKey { min, size, lod: lod.max(0.0).floor() as u8 };
		let mut remote_generations = self.state.remote_generations.lock().unwrap();
		let mut requests = self.state.lod_requests.lock().unwrap();
		remote_generations.request_lod(&mut requests, grid, key, generation);
	}
}
