use std::collections::HashMap;

use bevy::ecs::message::MessageReader;
use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, PeerMetadata, RemoteEvent};
use voxel_data::compressed_voxels::CompressedVoxels;
use voxel_data::grid::GridId;
use voxel_sources::{ChunkChanged, ChunkLoaded, LodLoaded, LodLoadRequest, VoxelSourceRequestApi, VoxelSourceRequests};
use voxel_streaming::GridStreaming;

use crate::chunk_source::{ChunkRequest, ChunkResponse, LodRequest, LodResponse, PresenceLoad, PresenceRequest, RemoteChunkChanged};
use crate::chunk_source::messages::RemoteChunkChangeKind;
use crate::ReplicateVoxelsRestriction;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct PendingChunkKey {
	grid: GridId,
	chunk: IVec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct PendingLodKey {
	grid: GridId,
	key: voxel_sources::LodKey,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PendingChunkRequest {
	peer: PeerId,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct PendingLodRequest {
	peer: PeerId,
}

#[derive(Resource, Default)]
pub(crate) struct PendingChunkRequests(HashMap<PendingChunkKey, Vec<PendingChunkRequest>>);

#[derive(Resource, Default)]
pub(crate) struct PendingLodRequests(HashMap<PendingLodKey, Vec<PendingLodRequest>>);

pub(crate) fn receive_chunk_request(
	trigger: On<RemoteEvent<ChunkRequest>>,
	requests: VoxelSourceRequests,
	mut pending: ResMut<PendingChunkRequests>,
) {
	let request = trigger.event().trigger;
	if request.chunk == IVec3::new(0, 5, -3) {
		info!(grid=?request.grid, chunk=?request.chunk, peer=?trigger.event().from, "server received chunk request");
	}
	requests.request_chunk(voxel_sources::ChunkLoadRequest { grid: request.grid, chunk: request.chunk });
	pending.0.entry(PendingChunkKey { grid: request.grid, chunk: request.chunk }).or_default().push(PendingChunkRequest {
		peer: trigger.event().from,
	});
}

pub(crate) fn receive_lod_request(
	trigger: On<RemoteEvent<LodRequest>>,
	requests: VoxelSourceRequests,
	mut pending: ResMut<PendingLodRequests>,
) {
	let request = trigger.event().trigger;
	requests.request_lod(LodLoadRequest {
		grid: request.grid,
		requester: Entity::PLACEHOLDER,
		key: request.key,
		priority: request.priority,
		generation: 0,
	});
	pending.0.entry(PendingLodKey { grid: request.grid, key: request.key }).or_default().push(PendingLodRequest {
		peer: trigger.event().from,
	});
}

pub(crate) fn receive_presence_request(
	trigger: On<RemoteEvent<PresenceRequest>>,
	grids: Query<(&GridStreaming, Option<&ReplicateVoxelsRestriction>)>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<PresenceLoad>>,
) {
	let request = trigger.event().trigger;
	let from = trigger.event().from;
	let Some(peer_metadata) = peer_metadata else { return };
	let Some(&entity) = peer_metadata.mapping.get(&from) else {
		warn!(?from, ?request.grid, "missing peer mapping for presence response");
		return;
	};
	let Ok(mut sender) = senders.get_mut(entity) else {
		warn!(?from, entity=?entity, ?request.grid, "missing sender for presence response");
		return;
	};
	let payload = if let Ok((streaming, restriction)) = grids.get(request.grid) {
		let area = chunk_presence_aabb(streaming)
			.and_then(|(min, size)| {
				restriction
					.and_then(|restriction| restriction.readable_aabb(from))
					.map(|(restriction_min, restriction_size)| intersect_aabbs(min, size, restriction_min, restriction_size))
					.unwrap_or(Some((min, size)))
			});
		PresenceLoad { grid: request.grid, area }
	} else {
		PresenceLoad { grid: request.grid, area: None }
	};
	sender.trigger::<super::ServerToClientChannel>(payload);
}

fn chunk_presence_aabb(streaming: &GridStreaming) -> Option<(IVec3, IVec3)> {
	let mut min = IVec3::splat(i32::MAX);
	let mut max = IVec3::splat(i32::MIN);
	let mut any = false;
	for (origin, extent) in streaming.presence().iter_present() {
		let node_max = origin + IVec3::splat(extent as i32 - 1);
		min = min.min(origin);
		max = max.max(node_max);
		any = true;
	}
	any.then_some((min, max - min + IVec3::ONE))
}

fn intersect_aabbs(min_a: IVec3, size_a: IVec3, min_b: IVec3, size_b: IVec3) -> Option<(IVec3, IVec3)> {
	let max_a = min_a + size_a - IVec3::ONE;
	let max_b = min_b + size_b - IVec3::ONE;
	let min = min_a.max(min_b);
	let max = max_a.min(max_b);
	(max.cmpge(min).all()).then_some((min, max - min + IVec3::ONE))
}

pub(crate) fn flush_chunk_changed(
	mut changed: MessageReader<ChunkChanged>,
	peer_metadata: Option<Res<PeerMetadata>>,
	grids: Query<Option<&ReplicateVoxelsRestriction>>,
	mut senders: Query<&mut EventSender<RemoteChunkChanged>>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for event in changed.read().copied() {
		info!(grid=?event.grid, min=?event.min, size=?event.size, kind=?event.kind, from_save=event.from_save, "server saw chunk changed");
		let kind = match event.kind {
			voxel_sources::ChunkChangeKind::Changed => RemoteChunkChangeKind::Changed,
			voxel_sources::ChunkChangeKind::Removed => RemoteChunkChangeKind::Removed,
		};
		for (&peer, &entity) in &peer_metadata.mapping {
			if peer == PeerId::Server { continue; }
			let Ok(restriction) = grids.get(event.grid) else { continue };
			let Some((min, size)) = restriction
				.and_then(|restriction| restriction.readable_aabb(peer))
				.map(|(restriction_min, restriction_size)| intersect_aabbs(event.min, event.size, restriction_min, restriction_size))
				.unwrap_or(Some((event.min, event.size)))
			else {
				continue;
			};
			let Ok(mut sender) = senders.get_mut(entity) else {
				warn!(grid=?event.grid, peer=?peer, entity=?entity, "missing sender for remote chunk changed");
				continue
			};
			sender.trigger::<super::ServerToClientChannel>(RemoteChunkChanged { grid: event.grid, min, size, kind });
		}
	}
}

pub(crate) fn flush_chunk_results(
	mut loader: MessageReader<ChunkLoaded>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<ChunkResponse>>,
	mut pending: ResMut<PendingChunkRequests>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for ChunkLoaded { grid, chunk, voxels } in loader.read() {
		if *chunk == IVec3::new(0, 5, -3) {
			info!(grid=?grid, chunk=?chunk, has_voxels=voxels.is_some(), "server flushing chunk result");
		}
		let pending_key = PendingChunkKey { grid: *grid, chunk: *chunk };
		let Some(requests) = pending.0.remove(&pending_key) else {
			warn!(?pending_key, "dropping chunk load result with no pending remote requests");
			continue
		};
		let voxels = match voxels.as_ref() {
			Some(voxels) => match CompressedVoxels::new(voxels) {
				Ok(voxels) => Some(voxels),
				Err(err) => {
					warn!(grid=?grid, chunk=?chunk, error=%err, "failed to compress chunk response voxels");
					continue
				}
			},
			None => None,
		};
		for request in requests {
			let Some(&entity) = peer_metadata.mapping.get(&request.peer) else {
				warn!(grid=?grid, chunk=?chunk, peer=?request.peer, "missing peer mapping for chunk response");
				continue
			};
			let Ok(mut sender) = senders.get_mut(entity) else {
				warn!(grid=?grid, chunk=?chunk, peer=?request.peer, entity=?entity, "missing sender for chunk response");
				continue
			};
			sender.trigger::<super::ServerToClientChannel>(ChunkResponse {
				grid: *grid,
				chunk: *chunk,
				voxels: voxels.clone(),
			});
		}
	}
}

pub(crate) fn flush_lod_results(
	mut loader: MessageReader<LodLoaded>,
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<&mut EventSender<LodResponse>>,
	mut pending: ResMut<PendingLodRequests>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for LodLoaded { grid, key, voxels, .. } in loader.read() {
		let Some(requests) = pending.0.remove(&PendingLodKey { grid: *grid, key: *key }) else {
			// warn!(grid=?grid, min=?key.min, size=?key.size, lod=key.lod, "dropping lod load result with no pending remote requests");
			continue
		};
		let voxels = match voxels.as_ref() {
			Some(voxels) => match CompressedVoxels::new(voxels) {
				Ok(voxels) => Some(voxels),
				Err(err) => {
					warn!(grid=?grid, min=?key.min, size=?key.size, lod=key.lod, error=%err, "failed to compress lod response voxels");
					continue
				}
			},
			None => None,
		};
		for request in requests {
			let Some(&entity) = peer_metadata.mapping.get(&request.peer) else {
				warn!(grid=?grid, min=?key.min, size=?key.size, lod=key.lod, peer=?request.peer, "missing peer mapping for lod response");
				continue
			};
			let Ok(mut sender) = senders.get_mut(entity) else {
				warn!(grid=?grid, min=?key.min, size=?key.size, lod=key.lod, peer=?request.peer, entity=?entity, "missing sender for lod response");
				continue
			};
			sender.trigger::<super::ServerToClientChannel>(LodResponse {
				grid: *grid,
				key: *key,
				voxels: voxels.clone(),
			});
		}
	}
}
