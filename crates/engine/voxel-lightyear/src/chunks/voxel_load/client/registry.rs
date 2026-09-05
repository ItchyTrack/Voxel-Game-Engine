use std::collections::{HashMap, HashSet, VecDeque};
use std::time::Duration;

use bevy::{log::warn, math::IVec3};
use voxel_trees::region::NonZeroVoxelRegion;
use voxel_data::{compressed_voxels::CompressedVoxels, grid::GridId, voxels::{VoxelTypeId, Voxels}};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion};
use voxel_sources::{RequestId, SourceHandle, edit::GridGeneration};
use voxel_tasks::CancellationToken;

use super::super::{
	VoxelLoadCancel,
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadReceived,
	VoxelLoadRequest,
	VoxelLoadRetry,
	VoxelPayloadIndex,
	VoxelPayloadMetadata,
	VoxelRequestKey,
};
use crate::chunks::request_id::{NetworkRequestId, NetworkRequestIdAllocator};

const RETRY_INTERVAL: Duration = Duration::from_millis(500);

#[derive(Clone, Debug)]
enum PendingPayloads {
	AwaitingManifest {
		payloads: HashMap<VoxelPayloadIndex, CompressedVoxels>,
	},
	Receiving {
		metadata: Box<[VoxelPayloadMetadata]>,
		received: Vec<bool>,
		next_retry_at: Duration,
	},
}

#[derive(Clone, Debug)]
struct PendingVoxels {
	request_id: RequestId,
	key: VoxelRequestKey,
	cancellation: CancellationToken,
	accepted_chunks: HashSet<IVec3>,
	payloads: PendingPayloads,
}

#[derive(Default)]
pub(crate) struct ClientLoadRegistry {
	ids: NetworkRequestIdAllocator,
	pending: HashMap<NetworkRequestId, PendingVoxels>,
	requests: VecDeque<VoxelLoadRequest>,
	cancellations: VecDeque<VoxelLoadCancel>,
	retries: VecDeque<VoxelLoadRetry>,
	received: VecDeque<VoxelLoadReceived>,
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
			accepted_chunks,
			payloads: PendingPayloads::AwaitingManifest { payloads: HashMap::new() },
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

	pub(crate) fn queue_due_retries(&mut self, now: Duration) {
		for (&id, pending) in &mut self.pending {
			let PendingPayloads::Receiving { received, next_retry_at, .. } = &mut pending.payloads else { continue };
			if now < *next_retry_at { continue; }
			let missing = received.iter().enumerate()
				.filter_map(|(index, received)| {
					(!received).then(|| VoxelPayloadIndex(u32::try_from(index).expect("manifest payload index exceeded protocol range")))
				})
				.collect::<Vec<_>>();
			if !missing.is_empty() {
				self.retries.push_back(VoxelLoadRetry { id, missing: missing.into_boxed_slice() });
			}
			*next_retry_at = now + RETRY_INTERVAL;
		}
	}

	pub(crate) fn receive_payload(
		&mut self,
		handle: &SourceHandle,
		from: &impl std::fmt::Debug,
		payload: &VoxelLoadPayload,
	) {
		let Some(pending) = self.pending.get_mut(&payload.id) else { return };
		if pending.cancellation.is_cancelled() {
			self.cancel(payload.id);
			return;
		}
		if let PendingPayloads::AwaitingManifest { payloads } = &mut pending.payloads {
			payloads.entry(payload.index).or_insert_with(|| payload.voxels.clone());
			return;
		}
		if self.process_payload(handle, from, payload.id, payload.index, &payload.voxels) {
			self.finish(handle, payload.id);
		}
	}

	pub(crate) fn receive_manifest(
		&mut self,
		handle: &SourceHandle,
		from: &impl std::fmt::Debug,
		manifest: &VoxelLoadManifest,
		now: Duration,
	) {
		let Some(pending) = self.pending.get_mut(&manifest.id) else { return };
		if pending.cancellation.is_cancelled() {
			self.cancel(manifest.id);
			return;
		}
		if manifest.payloads.len() > u32::MAX as usize
			|| manifest.payloads.iter().any(|metadata| {
				!pending.key.region.contains_region(metadata.region) || metadata.lod > pending.key.lod
			})
		{
			warn!(id=?manifest.id, ?from, "ignoring invalid remote voxel manifest");
			return;
		}
		let early_payloads = match std::mem::replace(
			&mut pending.payloads,
			PendingPayloads::Receiving {
				metadata: manifest.payloads.clone(),
				received: vec![false; manifest.payloads.len()],
				next_retry_at: now + RETRY_INTERVAL,
			},
		) {
			PendingPayloads::AwaitingManifest { payloads } => payloads,
			PendingPayloads::Receiving { .. } => {
				warn!(id=?manifest.id, ?from, "ignoring duplicate remote voxel manifest");
				return;
			}
		};

		for (index, voxels) in early_payloads {
			self.process_payload(handle, from, manifest.id, index, &voxels);
		}
		if self.pending.get(&manifest.id).is_some_and(|pending| {
			matches!(&pending.payloads, PendingPayloads::Receiving { received, .. } if received.iter().all(|received| *received))
		}) {
			self.finish(handle, manifest.id);
		}
	}

	pub(crate) fn pop_request(&mut self) -> Option<VoxelLoadRequest> { self.requests.pop_front() }
	pub(crate) fn pop_cancellation(&mut self) -> Option<VoxelLoadCancel> { self.cancellations.pop_front() }
	pub(crate) fn pop_retry(&mut self) -> Option<VoxelLoadRetry> { self.retries.pop_front() }
	pub(crate) fn pop_received(&mut self) -> Option<VoxelLoadReceived> { self.received.pop_front() }

	fn process_payload(
		&mut self,
		handle: &SourceHandle,
		from: &impl std::fmt::Debug,
		id: NetworkRequestId,
		index: VoxelPayloadIndex,
		compressed: &CompressedVoxels,
	) -> bool {
		let Some(pending) = self.pending.get_mut(&id) else { return false };
		let PendingPayloads::Receiving { metadata, received, .. } = &mut pending.payloads else { return false };
		let Ok(index_usize) = usize::try_from(index.0) else { return false };
		let Some((metadata, received_flag)) = metadata.get(index_usize).zip(received.get_mut(index_usize)) else {
			warn!(?id, ?index, ?from, "ignoring remote voxel payload outside its manifest");
			return false;
		};
		if *received_flag { return received.iter().all(|received| *received) }
		if u64::try_from(compressed.len()).ok() != Some(metadata.compressed_bytes) {
			warn!(?id, ?index, expected=metadata.compressed_bytes, actual=compressed.len(), ?from, "ignoring remote voxel payload with a mismatched compressed size");
			return false;
		}
		let voxels = match compressed.decompress() {
			Ok(voxels) => voxels,
			Err(err) => {
				warn!(?id, ?index, grid=?pending.key.grid, region=?metadata.region, lod=metadata.lod, ?from, error=%err, "failed to decompress remote voxel payload");
				return false;
			}
		};
		*received_flag = true;
		if let Some(voxels) = filter_accepted_chunks(&pending.accepted_chunks, metadata.region, metadata.lod, voxels) {
			handle.voxels(
				pending.request_id,
				pending.key.grid,
				metadata.region,
				metadata.lod,
				pending.key.generation,
				voxels,
			);
		}
		received.iter().all(|received| *received)
	}

	fn finish(&mut self, handle: &SourceHandle, id: NetworkRequestId) {
		let Some(pending) = self.pending.remove(&id) else { return };
		self.retries.retain(|retry| retry.id != id);
		handle.voxels_loaded(pending.request_id, pending.key.generation);
		self.received.push_back(VoxelLoadReceived { id });
	}

	fn cancel(&mut self, id: NetworkRequestId) {
		if self.pending.remove(&id).is_some() {
			self.retries.retain(|retry| retry.id != id);
			self.cancellations.push_back(VoxelLoadCancel { id });
		}
	}
}
