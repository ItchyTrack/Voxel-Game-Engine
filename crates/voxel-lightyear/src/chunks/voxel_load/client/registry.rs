use std::collections::{HashMap, VecDeque};

use bevy::log::warn;
use tile_data::NonZeroChunkRegion;
use voxel_data::grid::GridId;
use voxel_data::voxels::VoxelTypeId;
use voxel_sources::{RequestId, SourceHandle};
use voxel_tasks::CancellationToken;

use super::super::{VoxelLoadCancel, VoxelLoadComplete, VoxelLoadPayload, VoxelLoadRequest, VoxelRequestKey};
use crate::chunks::request_id::{NetworkRequestId, NetworkRequestIdAllocator};

#[derive(Clone, Debug)]
struct PendingVoxels {
	request_id: RequestId,
	key: VoxelRequestKey,
	cancellation: CancellationToken,
	received_payload_count: u32,
}

#[derive(Default)]
pub(crate) struct ClientLoadRegistry {
	ids: NetworkRequestIdAllocator,
	pending: HashMap<NetworkRequestId, PendingVoxels>,
	requests: VecDeque<VoxelLoadRequest>,
	cancellations: VecDeque<VoxelLoadCancel>,
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
	) {
		let id = self.ids.allocate();
		let key = VoxelRequestKey { grid, region, lod, voxel_type };
		self.pending.insert(id, PendingVoxels {
			request_id,
			key,
			cancellation: cancellation.clone(),
			received_payload_count: 0,
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
			|| pending.key.lod != payload.lod
			|| !pending.key.region.contains_region(payload.region)
		{
			warn!(id=?payload.id, grid=?payload.grid, region=?payload.region, lod=payload.lod, ?from, "ignoring mismatched remote voxel payload");
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
		handle.voxels(
			pending.request_id,
			payload.grid,
			payload.region,
			payload.lod,
			payload.generation,
			voxels,
		);
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
		self.pending.remove(&complete.id);
		handle.voxels_loaded(request_id);
	}

	pub(crate) fn pop_request(&mut self) -> Option<VoxelLoadRequest> { self.requests.pop_front() }
	pub(crate) fn pop_cancellation(&mut self) -> Option<VoxelLoadCancel> { self.cancellations.pop_front() }

	fn cancel(&mut self, id: NetworkRequestId) {
		if self.pending.remove(&id).is_some() {
			self.cancellations.push_back(VoxelLoadCancel { id });
		}
	}
}
