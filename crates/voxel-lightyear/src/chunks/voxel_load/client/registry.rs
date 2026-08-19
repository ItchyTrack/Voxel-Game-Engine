use std::collections::{HashMap, VecDeque};

use bevy::log::warn;
use bevy::math::IVec3;
use tile_data::NonZeroChunkRegion;
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{CancellationToken, SourceHandle, RequestId};

use super::super::{
	VoxelLoadFinished,
	VoxelLoadOutcome,
	VoxelLoadRequest,
	VoxelLoadRequestKind,
	VoxelLoadResponse,
};

#[derive(Clone, Debug)]
struct PendingVoxels {
	grid: GridId,
	region: NonZeroChunkRegion,
	required_lod: u8,
	cancellation: CancellationToken,
}

#[derive(Default)]
pub(crate) struct ClientLoadRegistry {
	pending: HashMap<RequestId, PendingVoxels>,
	requests: VecDeque<VoxelLoadRequest>,
	finished: VecDeque<VoxelLoadFinished>,
}

impl ClientLoadRegistry {
	pub fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		priority: f32,
	) {
		self.pending.insert(request_id, PendingVoxelLoad::VoxelArea(PendingVoxelArea {
			grid,
			region,
			required_lod,
			cancellation,
		}));
		self.requests.push_back(VoxelLoadRequest { request_id, grid, region, required_lod, voxel_type, priority });
	}

	pub fn drain_cancelled(&mut self) {
		let cancelled: Vec<_> = self.pending.iter().filter_map(|(&id, pending)| pending.is_cancelled().then_some(id)).collect();
		for id in cancelled {
			self.finish(id, VoxelLoadOutcome::Cancelled);
		}
	}

	pub fn receive_voxels(&mut self, handle: &SourceHandle, from: impl std::fmt::Debug, response: &mut VoxelLoadResponse) {
		let Some(pending) = self.pending.get(&response.request_id) else { return };
		if pending.cancellation.is_cancelled() {
			self.finish(id, VoxelLoadOutcome::Cancelled);
			return;
		}
		if generation < pending.required_generation { return; }

		let voxels = match decompress(compressed) {
			Ok(voxels) => voxels,
			Err(err) => {
				warn!(?id, ?grid, min=?key.min(), size=?key.size(), lod=key.lod, ?from, error=%err, "failed to decompress remote lod response");
				return;
			}
		};
		self.finish(id, VoxelLoadOutcome::Received);
		handle.voxels(request_id, grid, key.region(), key.lod, generation, voxels);
	}

	pub(crate) fn pop_request(&mut self) -> Option<VoxelLoadRequest> { self.requests.pop_front() }
	pub(crate) fn pop_finished(&mut self) -> Option<VoxelLoadFinished> { self.finished.pop_front() }

	// fn receive_chunk_response(
	// 	&mut self,
	// 	handle: &SourceHandle,
	// 	from: impl std::fmt::Debug,
	// 	id: RequestId,
	// 	grid: GridId,
	// 	chunk: IVec3,
	// 	generation: u64,
	// 	compressed: &mut Option<CompressedVoxels>,
	// ) {
	// 	let Some(pending) = self.pending.get(&id) else { return };
	// 	let PendingVoxelLoad::Chunk(pending) = pending else {
	// 		warn!(?id, ?from, "ignoring chunk response for a non-chunk voxel load");
	// 		return;
	// 	};
	// 	if pending.cancellation.is_cancelled() {
	// 		self.finish(id, VoxelLoadOutcome::Cancelled);
	// 		return;
	// 	}
	// 	if pending.key.grid != grid || pending.key.chunk != chunk {
	// 		warn!(?id, ?grid, ?chunk, ?from, "ignoring mismatched remote chunk response");
	// 		return;
	// 	}
	// 	if generation < pending.request_generation { return; }

	// 	let voxels = match decompress(compressed) {
	// 		Ok(voxels) => voxels,
	// 		Err(err) => {
	// 			warn!(?id, ?grid, ?chunk, ?from, error=%err, "failed to decompress remote chunk response");
	// 			return;
	// 		}
	// 	};
	// 	let Some(PendingVoxelLoad::Chunk(_)) = self.finish(id, VoxelLoadOutcome::Received) else {
	// 		unreachable!("checked chunk voxel load disappeared")
	// 	};
	// 	handle.loaded(grid, chunk, generation, voxels);
	// }

	fn receive_voxels_area_response(
		&mut self,
		handle: &SourceHandle,
		request_id: RequestId,
		grid: GridId,
		region: NonZeroChunkRegion,
		generation: u64,
		voxel_type: VoxelTypeId,
		compressed: &mut CompressedVoxels,
	) {

	}

	fn finish(&mut self, id: RequestId, outcome: VoxelLoadOutcome) {
		assert!(self.pending.remove(&id).is_some());
		self.finished.push_back(VoxelLoadFinished { request_id, outcome });
	}
}

#[cfg(test)]
mod tests {
	use bevy::{math::UVec3, prelude::Entity};
use tile_data::NonZeroChunkRegion;

	use super::*;


	#[test]
	fn cancelled_chunk_has_one_terminal_outcome() {
		let cancellation = CancellationToken::new();
		let mut loads = ClientLoadRegistry::default();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 7, None, cancellation.clone());
		let request = loads.pop_request().unwrap();

		cancellation.cancel();
		loads.drain_cancelled();

		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { request_id: request.request_id, outcome: VoxelLoadOutcome::Cancelled }));
		assert_eq!(loads.pop_finished(), None);
		assert!(!loads.pending.contains_key(&request.id));
	}

	#[test]
	fn cancelled_voxel_area_has_one_terminal_outcome() {
		let cancellation = CancellationToken::new();
		let key = VoxelAreaKey {
			region: NonZeroChunkRegion::from_single(IVec3::ZERO),
			lod: 1,
		};
		let mut loads = ClientLoadRegistry::default();
		loads.request_voxel_area(
			Entity::PLACEHOLDER,
			key,
			VoxelTypeId(1),
			0.0,
			7,
			cancellation.clone(),
		);
		let request = loads.pop_request().unwrap();

		cancellation.cancel();
		loads.drain_cancelled();

		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { request_id: request.request_id, outcome: VoxelLoadOutcome::Cancelled }));
		assert_eq!(loads.pop_finished(), None);
		assert!(!loads.pending.contains_key(&request.id));
	}

	#[test]
	fn received_voxel_load_cannot_be_cancelled_afterward() {
		let mut loads = ClientLoadRegistry::default();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 1, None, CancellationToken::new());
		let request = loads.pop_request().unwrap();

		loads.finish(request.id, VoxelLoadOutcome::Received).is_some();
		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { request_id: request.request_id, outcome: VoxelLoadOutcome::Received }));
		assert_eq!(loads.pop_finished(), None);
	}

	#[test]
	fn replacement_cancels_previous_chunk_voxel_load() {
		let mut loads = ClientLoadRegistry::default();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 1, None, CancellationToken::new());
		let first = loads.pop_request().unwrap();
		loads.request_chunk(Entity::PLACEHOLDER, IVec3::ZERO, 2, None, CancellationToken::new());
		let second = loads.pop_request().unwrap();

		assert_ne!(first.id, second.id);
		assert_eq!(loads.pop_finished(), Some(VoxelLoadFinished { request_id: first.request_id, outcome: VoxelLoadOutcome::Cancelled }));
		assert!(!loads.pending.contains_key(&first.id));
		assert!(loads.pending.contains_key(&second.id));
	}
}
