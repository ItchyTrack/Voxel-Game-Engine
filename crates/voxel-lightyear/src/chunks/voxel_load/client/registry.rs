use std::collections::{HashMap, HashSet, VecDeque};

use bevy::{log::warn, math::IVec3};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion};
use voxel_data::{grid::GridId, region::NonZeroVoxelRegion, voxels::{VoxelTypeId, Voxels}};
use voxel_sources::{RequestId, SourceHandle, edit::GridGeneration};
use voxel_tasks::CancellationToken;

use super::super::{VoxelLoadCancel, VoxelLoadComplete, VoxelLoadPayload, VoxelLoadRequest, VoxelRequestKey};
use crate::chunks::request_id::{NetworkRequestId, NetworkRequestIdAllocator};

#[derive(Clone, Debug)]
struct PendingVoxels {
	request_id: RequestId,
	key: VoxelRequestKey,
	cancellation: CancellationToken,
	received_payload_count: u32,
	accepted_chunks: HashSet<IVec3>,
}

#[derive(Default)]
pub(crate) struct ClientLoadRegistry {
	ids: NetworkRequestIdAllocator,
	pending: HashMap<NetworkRequestId, PendingVoxels>,
	requests: VecDeque<VoxelLoadRequest>,
	cancellations: VecDeque<VoxelLoadCancel>,
}

fn filter_accepted_chunks(
	accepted_chunks: &HashSet<IVec3>,
	region: NonZeroChunkRegion,
	lod: u8,
	voxels: Voxels,
) -> Option<Voxels> {
	let chunk_extent = (CHUNK_SIZE >> lod) as i32;
	let mut filtered = Voxels::new_with_type(voxels.voxel_type_info());
	for &chunk in accepted_chunks {
		if !region.contains(chunk) { continue; }
		let local_min = (chunk - region.min()) * chunk_extent;
		let local_region = NonZeroVoxelRegion::from_min_size(local_min, bevy::math::UVec3::splat(chunk_extent as u32)).unwrap();
		filtered.merge_region_from(&voxels, Some(local_region), IVec3::ZERO);
	}
	(!filtered.is_empty()).then_some(filtered)
}

impl ClientLoadRegistry {
	pub(crate) fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
		accepted_chunks: HashSet<IVec3>,
	) {
		let id = self.ids.allocate();
		let key = VoxelRequestKey { grid, region, lod, voxel_type, generation };
		self.pending.insert(id, PendingVoxels {
			request_id,
			key,
			cancellation: cancellation.clone(),
			received_payload_count: 0,
			accepted_chunks,
		});
		self.requests.push_back(VoxelLoadRequest { id, key });
	}

	pub(crate) fn drain_cancelled(&mut self) {
		let cancelled: Vec<_> = self.pending.iter()
			.filter_map(|(&id, pending)| pending.cancellation.is_cancelled().then_some(id))
			.collect();
		for id in cancelled {
			self.cancel(id);
		}
	}

	pub(crate) fn receive_payload(
		&mut self,
		handle: &SourceHandle,
		from: impl std::fmt::Debug,
		payload: &VoxelLoadPayload,
	) {
		let Some(pending) = self.pending.get_mut(&payload.id) else { return };
		if pending.cancellation.is_cancelled() {
			self.cancel(payload.id);
			return;
		}
		if pending.key.grid != payload.grid
			|| !pending.key.region.contains_region(payload.region)
			|| pending.key.lod != payload.lod
			|| pending.key.generation != payload.generation
		{
			warn!(id=?payload.id, grid=?payload.grid, region=?payload.region, lod=payload.lod, generation=?payload.generation, ?from, "ignoring mismatched remote voxel payload");
			return;
		}
		let voxels = match payload.voxels.decompress() {
			Ok(voxels) => voxels,
			Err(err) => {
				warn!(id=?payload.id, grid=?payload.grid, region=?payload.region, lod=payload.lod, ?from, error=%err, "failed to decompress remote voxel payload");
				return;
			}
		};
		let Some(received_payload_count) = pending.received_payload_count.checked_add(1) else {
			warn!(id=?payload.id, ?from, "remote voxel payload count overflowed");
			return;
		};
		pending.received_payload_count = received_payload_count;
		if let Some(voxels) = filter_accepted_chunks(&pending.accepted_chunks, payload.region, payload.lod, voxels) {
			handle.voxels(
				pending.request_id,
				payload.grid,
				payload.region,
				payload.lod,
				pending.key.generation,
				voxels,
			);
		}
	}

	pub(crate) fn receive_complete(
		&mut self,
		handle: &SourceHandle,
		from: impl std::fmt::Debug,
		complete: VoxelLoadComplete,
	) {
		let Some(pending) = self.pending.get(&complete.id) else { return };
		if pending.cancellation.is_cancelled() {
			self.cancel(complete.id);
			return;
		}
		if pending.received_payload_count != complete.sent_payload_count {
			warn!(id=?complete.id, expected=complete.sent_payload_count, received=pending.received_payload_count, ?from, "ignoring remote voxel completion with a mismatched payload count");
			return;
		}
		let request_id = pending.request_id;
		let generation = pending.key.generation;
		self.pending.remove(&complete.id);
		handle.voxels_loaded(request_id, generation);
	}

	pub(crate) fn pop_request(&mut self) -> Option<VoxelLoadRequest> { self.requests.pop_front() }
	pub(crate) fn pop_cancellation(&mut self) -> Option<VoxelLoadCancel> { self.cancellations.pop_front() }

	fn cancel(&mut self, id: NetworkRequestId) {
		if self.pending.remove(&id).is_some() {
			self.cancellations.push_back(VoxelLoadCancel { id });
		}
	}
}
