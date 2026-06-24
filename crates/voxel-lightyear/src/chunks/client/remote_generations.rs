use std::collections::{HashMap, VecDeque};

use bevy::log::warn;
use bevy::math::IVec3;
use voxel_data::grid::GridId;
use voxel_sources::{LodKey, SourceHandle};

use crate::chunks::messages::RemoteChunkChangeKind;
use crate::chunks::{ChunkRequest, ChunkResponse, LodRequest, LodResponse, RemoteChunkChanged};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingChunkKey {
	grid: GridId,
	chunk: IVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PendingChunk {
	request_generation: u64,
	deferred_generation: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PendingLodKey {
	grid: GridId,
	key: LodKey,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PendingLod {
	request_generation: u64,
	deferred_generation: Option<u64>,
}

#[derive(Default)]
pub(super) struct RemoteGenerations {
	chunks: HashMap<PendingChunkKey, PendingChunk>,
	lods: HashMap<PendingLodKey, PendingLod>,
}

impl RemoteGenerations {
	pub fn request_chunk(&mut self, requests: &mut VecDeque<ChunkRequest>, grid: GridId, chunk: IVec3, request_generation: u64) {
		self.chunks.entry(PendingChunkKey { grid, chunk }).or_insert(PendingChunk {
			request_generation,
			deferred_generation: None,
		});
		requests.push_back(ChunkRequest { grid, chunk });
	}

	pub fn request_lod(&mut self, requests: &mut VecDeque<LodRequest>, grid: GridId, key: LodKey, request_generation: u64) {
		self.lods.entry(PendingLodKey { grid, key }).or_insert(PendingLod {
			request_generation,
			deferred_generation: None,
		});
		requests.push_back(LodRequest { grid, key, priority: 0.0 });
	}

	pub fn receive_chunk_changed(&mut self, handle: &SourceHandle, event: RemoteChunkChanged) {
		let generation = match event.kind {
			RemoteChunkChangeKind::Changed { generation } | RemoteChunkChangeKind::Removed { generation } => generation,
		};
		for x in event.min.x..event.min.x + event.size.x {
			for y in event.min.y..event.min.y + event.size.y {
				for z in event.min.z..event.min.z + event.size.z {
					let chunk = IVec3::new(x, y, z);
					let pending = PendingChunkKey { grid: event.grid, chunk };
					match event.kind {
						RemoteChunkChangeKind::Changed { .. } => {
							if let Some(entry) = self.chunks.get_mut(&pending) {
								entry.deferred_generation = Some(entry.deferred_generation.map_or(generation, |current| current.max(generation)));
							} else {
								handle.claim(event.grid, chunk, IVec3::ONE);
							}
						}
						RemoteChunkChangeKind::Removed { .. } => {
							self.chunks.remove(&pending);
							handle.unavailable(event.grid, chunk, IVec3::ONE);
						}
					}
				}
			}
		}

		for (pending, deferred) in &mut self.lods {
			if pending.grid != event.grid || !lod_overlaps_area(pending.key, event.min, event.size) {
				continue;
			}
			deferred.deferred_generation = Some(deferred.deferred_generation.map_or(generation, |current| current.max(generation)));
		}
	}

	pub fn receive_chunk_response(&mut self, handle: &SourceHandle, from: impl std::fmt::Debug, response: &mut ChunkResponse) {
		let Some(pending_chunk) = self.chunks.remove(&PendingChunkKey { grid: response.grid, chunk: response.chunk }) else {
			return;
		};
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
		if pending_chunk.deferred_generation.is_some_and(|dirty_generation| response.generation < dirty_generation) {
			handle.claim(response.grid, response.chunk, IVec3::ONE);
		}
		handle.loaded(response.grid, response.chunk, pending_chunk.request_generation, voxels);
	}

	pub fn receive_lod_response(&mut self, handle: &SourceHandle, from: impl std::fmt::Debug, response: &mut LodResponse) {
		let Some(pending_lod) = self.lods.remove(&PendingLodKey { grid: response.grid, key: response.key }) else {
			warn!(grid=?response.grid, min=?response.key.min, size=?response.key.size, lod=response.key.lod, ?from, "ignoring unexpected remote lod response");
			return;
		};
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
		if pending_lod.deferred_generation.is_some_and(|generation| response.generation < generation) {
			// Preserve the stale generation; voxel-streaming will reject it.
		}
		handle.loaded_lod(response.grid, response.key.min, response.key.size, response.key.lod as f32, pending_lod.request_generation, voxels);
	}
}

fn lod_overlaps_area(key: LodKey, min: IVec3, size: IVec3) -> bool {
	let max = min + size;
	let key_max = key.min + key.size;
	key.min.cmplt(max).all() && min.cmplt(key_max).all()
}
