use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerMetadata, RemoteEvent};
use voxel_sources::{SourceManager, SourceResult};

use super::registry::{OutgoingVoxelMessage, PendingVoxelLoads};
use super::super::{
	VoxelLoadCancel,
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadReceived,
	VoxelLoadRequest,
	VoxelLoadRetry,
};
use crate::chunks::edit_stream::EditSubscriptions;

pub(crate) fn receive_voxel_load_request(
	trigger: On<RemoteEvent<VoxelLoadRequest>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingVoxelLoads>,
	edits: Res<EditSubscriptions>,
) {
	let event = trigger.event();
	let server_generation = edits.server_generation(
		event.from,
		event.trigger.key.grid,
		event.trigger.key.generation,
	);
	pending.request(event.from, event.trigger, server_generation, &mut sources);
}

pub(crate) fn receive_voxel_load_cancel(
	trigger: On<RemoteEvent<VoxelLoadCancel>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let event = trigger.event();
	pending.cancel(event.from, event.trigger.id, &mut sources);
}

pub(crate) fn receive_voxel_load_retry(
	trigger: On<RemoteEvent<VoxelLoadRetry>>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let event = trigger.event();
	pending.retry(event.from, event.trigger.clone());
}

pub(crate) fn receive_voxel_load_received(
	trigger: On<RemoteEvent<VoxelLoadReceived>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let event = trigger.event();
	pending.received(event.from, event.trigger.id, &mut sources);
}

pub(crate) fn flush_source_results(
	mut results: MessageReader<SourceResult>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	for result in results.read() {
		pending.receive_source_result(result);
	}
}

pub(crate) fn send_pending_voxel_load_responses(
	peer_metadata: Option<Res<PeerMetadata>>,
	mut senders: Query<(&mut EventSender<VoxelLoadManifest>, &mut EventSender<VoxelLoadPayload>)>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for _ in 0..pending.outgoing_len() {
		let Some(message) = pending.pop_outgoing() else { break };
		let Some(&entity) = peer_metadata.mapping.get(&message.peer()) else {
			pending.push_outgoing(message);
			continue;
		};
		let Ok((mut manifest_sender, mut payload_sender)) = senders.get_mut(entity) else {
			pending.push_outgoing(message);
			continue;
		};
		match message {
			OutgoingVoxelMessage::Manifest { manifest, .. } => {
				manifest_sender.trigger::<crate::chunks::ServerToClientChannel>(manifest);
			}
			OutgoingVoxelMessage::Payload { payload, .. } => {
				payload_sender.trigger::<crate::chunks::ServerToClientVoxelDataChannel>(payload);
			}
		}
	}
}

pub(crate) fn cleanup_disconnected_requests(
	peer_metadata: Option<Res<PeerMetadata>>,
	mut sources: ResMut<SourceManager>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	pending.remove_disconnected(|peer| peer_metadata.mapping.contains_key(&peer), &mut sources);
}
