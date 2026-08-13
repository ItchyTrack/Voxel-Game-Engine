use bevy::ecs::message::MessageReader;
use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerMetadata, RemoteEvent};
use voxel_sources::{ChunkLoaded, VoxelAreaLoaded};

use super::{LightyearSourceRequestHandle, registry::PendingVoxelLoads};
use crate::chunks::edit_stream::EditSubscriptions;
use crate::chunks::voxel_load::VoxelLoadResponseKind;
use super::super::{VoxelLoadFinished, VoxelLoadRequest, VoxelLoadResponse};

pub(crate) fn receive_voxel_load_request(
	trigger: On<RemoteEvent<VoxelLoadRequest>>,
	sources: Res<LightyearSourceRequestHandle>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	pending.request(trigger.event().from, trigger.event().trigger, sources.requests());
}

pub(crate) fn receive_voxel_load_finished(trigger: On<RemoteEvent<VoxelLoadFinished>>, mut pending: ResMut<PendingVoxelLoads>) {
	pending.finish(trigger.event().from, trigger.event().trigger);
}

pub(crate) fn flush_chunk_results(
	mut loader: MessageReader<ChunkLoaded>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	for ChunkLoaded { grid, chunk, edit_index, voxels } in loader.read() {
		if let Err(err) = pending.complete_chunk(*grid, *chunk, *edit_index, voxels.as_ref()) {
			warn!(grid=?grid, chunk=?chunk, error=%err, "failed to compress chunk response voxels");
		}
	}
}

pub(crate) fn flush_voxel_area_results(
	mut loader: MessageReader<VoxelAreaLoaded>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	for VoxelAreaLoaded { grid, key, edit_index, voxel_type, voxels, .. } in loader.read() {
		if let Err(err) = pending.complete_voxel_area(*grid, *key, *edit_index, *voxel_type, voxels.as_ref()) {
			warn!(grid=?grid, min=?key.min(), size=?key.size(), lod=key.lod, error=%err, "failed to compress lod response voxels");
		}
	}
}

pub(crate) fn send_pending_voxel_load_responses(
	time: Res<Time>,
	peer_metadata: Option<Res<PeerMetadata>>,
	subscriptions: Res<EditSubscriptions>,
	mut senders: Query<&mut EventSender<VoxelLoadResponse>>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	pending.send_due(time.elapsed_secs_f64(), |peer, mut response| {
		match &mut response.kind {
			VoxelLoadResponseKind::Chunk { grid, edit_index, .. }
			| VoxelLoadResponseKind::VoxelArea { grid, edit_index, .. } => {
				*edit_index = subscriptions.snapshot_index(peer, *grid, *edit_index);
			}
		}
		let Some(&entity) = peer_metadata.mapping.get(&peer) else { return };
		let Ok(mut sender) = senders.get_mut(entity) else { return };
		sender.trigger::<crate::chunks::ServerToClientUnreliableChannel>(response);
	});
}

pub(crate) fn cleanup_disconnected_requests(
	peer_metadata: Option<Res<PeerMetadata>>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	pending.remove_disconnected(|peer| peer_metadata.mapping.contains_key(&peer));
}
