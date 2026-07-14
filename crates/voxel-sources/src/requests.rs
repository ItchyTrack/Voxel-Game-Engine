use bevy::ecs::system::SystemParam;
use bevy::math::IVec3;
use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;
use voxel_tasks::CancellationToken;

use crate::loader::{ChunkLoadRequest, LodCancellation, LodLoadRequest, PresenceLoadRequest};
use crate::registry::SourceRegistry;

pub trait VoxelSourceRequestApi {
	fn request_presence(&self, request: PresenceLoadRequest);
	fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken;
	fn request_lod(&self, request: LodLoadRequest) -> LodCancellation;
	fn chunk_requests_sent(&self) -> u64;
	fn lod_requests_sent(&self) -> u64;
}

impl VoxelSourceRequestApi for SourceRegistry {
	fn request_presence(&self, request: PresenceLoadRequest) {
		self.request_presence(request);
	}

	fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken {
		self.request_chunk(request)
	}

	fn request_lod(&self, request: LodLoadRequest) -> LodCancellation {
		self.request_lod(request)
	}

	fn chunk_requests_sent(&self) -> u64 {
		SourceRegistry::chunk_requests_sent(self)
	}

	fn lod_requests_sent(&self) -> u64 {
		SourceRegistry::lod_requests_sent(self)
	}
}

#[derive(SystemParam)]
pub struct VoxelSourceRequests<'w> {
	registry: Res<'w, SourceRegistry>,
}

#[derive(SystemParam)]
pub struct VoxelSources<'w> {
	registry: Res<'w, SourceRegistry>,
}

impl VoxelSources<'_> {
	pub fn route_save(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) {
		self.registry.route_save(grid, chunk, voxels);
	}

	pub fn forget_others(&self, keep: crate::SourceId, grid: GridId, min: IVec3, size: IVec3) {
		self.registry.forget_others(keep, grid, min, size);
	}
}

impl VoxelSourceRequestApi for VoxelSourceRequests<'_> {
	fn request_presence(&self, request: PresenceLoadRequest) {
		self.registry.request_presence(request);
	}

	fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken {
		self.registry.request_chunk(request)
	}

	fn request_lod(&self, request: LodLoadRequest) -> LodCancellation {
		self.registry.request_lod(request)
	}

	fn chunk_requests_sent(&self) -> u64 {
		self.registry.chunk_requests_sent()
	}

	fn lod_requests_sent(&self) -> u64 {
		self.registry.lod_requests_sent()
	}
}

impl VoxelSourceRequestApi for VoxelSources<'_> {
	fn request_presence(&self, request: PresenceLoadRequest) {
		self.registry.request_presence(request);
	}

	fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken {
		self.registry.request_chunk(request)
	}

	fn request_lod(&self, request: LodLoadRequest) -> LodCancellation {
		self.registry.request_lod(request)
	}

	fn chunk_requests_sent(&self) -> u64 {
		self.registry.chunk_requests_sent()
	}

	fn lod_requests_sent(&self) -> u64 {
		self.registry.lod_requests_sent()
	}
}
