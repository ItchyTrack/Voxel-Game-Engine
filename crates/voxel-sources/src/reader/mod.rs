mod loader;
mod request_handle;
pub(crate) mod systems;

use std::collections::{HashMap, HashSet};

use bevy::ecs::resource::Resource;
use crossbeam_channel::{Receiver, Sender, unbounded};

use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_tasks::CancellationToken;

use crate::source_manager::SourceManager;

pub use loader::{
	ChunkLoadRequest, ChunkSaveChannel, ChunkSaveRequest, PresenceLoadRequest,
	VoxelAreaCancellation, VoxelAreaKey, VoxelAreaMessageRequest, VoxelAreaLoadEvent,
	VoxelAreaLoadRequest, VoxelAreaLoadResult,
};
pub use request_handle::{VoxelSourcesRequestHandle, VoxelSourcesRequestHandleGetter};

use loader::{PendingVoxelAreaRequest, ReaderRequest};
use request_handle::VoxelSourceRequesterId;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VoxelReaderKey {
	grid: GridId,
	key: VoxelAreaKey,
	voxel_type: VoxelTypeId,
}

struct ActiveChunkLoad {
	requester: VoxelSourceRequesterId,
	required_edit_index: u64,
	cancellation: CancellationToken,
}

struct PendingVoxelLoad {
	requests: Vec<PendingVoxelAreaRequest>,
	required_edit_index: u64,
	source_cancellation: CancellationToken,
}

#[derive(Resource)]
pub(crate) struct VoxelReader {
	request_tx: Sender<ReaderRequest>,
	request_rx: Receiver<ReaderRequest>,
	pending_presence: HashMap<GridId, HashSet<VoxelSourceRequesterId>>,
	active_chunks: HashMap<(GridId, bevy::math::IVec3), ActiveChunkLoad>,
	pending_voxels: HashMap<VoxelReaderKey, PendingVoxelLoad>,
}

impl Default for VoxelReader {
	fn default() -> Self {
		let (request_tx, request_rx) = unbounded();
		Self {
			request_tx,
			request_rx,
			pending_presence: HashMap::new(),
			active_chunks: HashMap::new(),
			pending_voxels: HashMap::new(),
		}
	}
}

impl VoxelReader {
	pub(crate) fn request_sender(&self) -> Sender<ReaderRequest> {
		self.request_tx.clone()
	}

	pub(crate) fn process_requests(&mut self, sources: &SourceManager) {
		while let Ok(request) = self.request_rx.try_recv() {
			match request {
				ReaderRequest::Presence { requester, request } => {
					let start_request = match self.pending_presence.entry(request.grid) {
						std::collections::hash_map::Entry::Occupied(mut entry) => {
							entry.get_mut().insert(requester);
							false
						}
						std::collections::hash_map::Entry::Vacant(entry) => {
							entry.insert(HashSet::from([requester]));
							true
						}
					};
					if start_request {
						sources.request_presence(request.grid);
					}
				}
				ReaderRequest::Chunk { requester, request, cancellation } => {
					let edit_index = sources.request_chunk(request.grid, request.chunk, cancellation.clone());
					if let Some(previous) = self.active_chunks.insert(
						(request.grid, request.chunk),
						ActiveChunkLoad { requester, required_edit_index: edit_index, cancellation },
					) {
						previous.cancellation.cancel();
					}
				}
				ReaderRequest::VoxelArea { requester, request, cancellation, target } => {
					self.request_voxels(sources, requester, request, cancellation, target);
				}
				ReaderRequest::CancelVoxelArea { requester, grid, key, voxel_type } => {
					self.cancel_voxels(requester, VoxelReaderKey { grid, key, voxel_type });
				}
				ReaderRequest::ReleaseRequester { requester } => self.release_requester(requester),
			}
		}
	}

	fn request_voxels(
		&mut self,
		sources: &SourceManager,
		requester: VoxelSourceRequesterId,
		request: VoxelAreaLoadRequest,
		cancellation: CancellationToken,
		target: loader::VoxelCompletionTarget,
	) {
		let source_cancellation = CancellationToken::new();
		let edit_index = sources.request_voxels(
			request.grid,
			request.key.min(),
			request.key.size().as_ivec3(),
			request.key.lod as f32,
			request.voxel_type,
			request.priority,
			source_cancellation.clone(),
		);
		let key = VoxelReaderKey { grid: request.grid, key: request.key, voxel_type: request.voxel_type };
		let pending_request = PendingVoxelAreaRequest { requester, request, cancellation, target };
		match self.pending_voxels.get_mut(&key) {
			Some(load) if edit_index <= load.required_edit_index => {
				source_cancellation.cancel();
				load.requests.push(pending_request);
			}
			Some(load) => {
				load.source_cancellation.cancel();
				load.required_edit_index = edit_index;
				load.source_cancellation = source_cancellation;
				load.requests.push(pending_request);
			}
			None => {
				self.pending_voxels.insert(key, PendingVoxelLoad {
					requests: vec![pending_request],
					required_edit_index: edit_index,
					source_cancellation,
				});
			}
		}
	}

	fn cancel_voxels(&mut self, requester: VoxelSourceRequesterId, key: VoxelReaderKey) {
		let remove = if let Some(load) = self.pending_voxels.get_mut(&key) {
			load.requests.retain(|request| request.requester != requester && !request.cancellation.is_cancelled());
			if load.requests.is_empty() {
				load.source_cancellation.cancel();
				true
			} else {
				false
			}
		} else {
			false
		};
		if remove {
			self.pending_voxels.remove(&key);
		}
	}

	fn release_requester(&mut self, requester: VoxelSourceRequesterId) {
		for requesters in self.pending_presence.values_mut() {
			requesters.remove(&requester);
		}

		self.active_chunks.retain(|_, load| {
			if load.requester == requester {
				load.cancellation.cancel();
				false
			} else {
				true
			}
		});

		self.pending_voxels.retain(|_, load| {
			load.requests.retain(|request| request.requester != requester && !request.cancellation.is_cancelled());
			if load.requests.is_empty() {
				load.source_cancellation.cancel();
				false
			} else {
				true
			}
		});
	}

	pub(crate) fn complete_presence(&mut self, grid: GridId) -> bool {
		self.pending_presence.remove(&grid).is_some_and(|requesters| !requesters.is_empty())
	}

	pub(crate) fn complete_chunk(&mut self, grid: GridId, chunk: bevy::math::IVec3, edit_index: u64) -> bool {
		let key = (grid, chunk);
		if !self.active_chunks.get(&key).is_some_and(|load| {
			!load.cancellation.is_cancelled() && edit_index >= load.required_edit_index
		}) {
			return false;
		}
		self.active_chunks.remove(&key);
		true
	}

	pub(crate) fn complete_voxels(
		&mut self,
		grid: GridId,
		min: bevy::math::IVec3,
		size: bevy::math::IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		edit_index: u64,
	) -> Vec<PendingVoxelAreaRequest> {
		let key = VoxelReaderKey {
			grid,
			key: VoxelAreaKey::new(min, size.as_uvec3(), lod.max(0.0).floor() as u8),
			voxel_type,
		};
		let Some(load) = self.pending_voxels.get(&key) else { return Vec::new() };
		if edit_index < load.required_edit_index { return Vec::new(); }
		let mut load = self.pending_voxels.remove(&key).unwrap();
		load.requests.retain(|request| !request.cancellation.is_cancelled());
		load.requests
	}
}
