use bevy::prelude::*;
use lightyear::prelude::{EventSender, NetworkingMetadata, RemoteEvent};
use voxel_sources::VoxelRequester;

use super::registry::{OutgoingVoxelMessage, PendingVoxelLoads, ServerVoxelLoadNode};
use super::super::{
	VoxelLoadCancel,
	VoxelLoadManifest,
	VoxelLoadPayload,
	VoxelLoadReceived,
	VoxelLoadRequest,
	VoxelLoadRetry,
};

pub(crate) fn receive_voxel_load_request(
	trigger: On<RemoteEvent<VoxelLoadRequest>>,
	node: Single<Entity, With<ServerVoxelLoadNode>>,
	mut requester: VoxelRequester,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let event = trigger.event();
	pending.request(event.from, event.trigger, *node, &mut requester);
}

pub(crate) fn receive_voxel_load_cancel(
	trigger: On<RemoteEvent<VoxelLoadCancel>>,
	node: Single<Entity, With<ServerVoxelLoadNode>>,
	mut requester: VoxelRequester,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let event = trigger.event();
	pending.cancel(event.from, event.trigger.id, *node, &mut requester);
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
	node: Single<Entity, With<ServerVoxelLoadNode>>,
	mut requester: VoxelRequester,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let event = trigger.event();
	pending.received(event.from, event.trigger.id, *node, &mut requester);
}

pub(crate) fn flush_voxel_data(
	node: Single<(Entity, &mut ServerVoxelLoadNode)>,
	mut requester: VoxelRequester,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let (entity, mut node) = node.into_inner();
	for key in node.take_dirtied() {
		pending.dirty_voxels(key, entity, &mut requester);
	}
	for (key, data) in node.take_received() {
		pending.receive_voxels(key, &data, entity, &mut requester);
	}
}

pub(crate) fn send_pending_voxel_load_responses(
	peer_metadata: Option<Res<NetworkingMetadata>>,
	mut senders: Query<(&mut EventSender<VoxelLoadManifest>, &mut EventSender<VoxelLoadPayload>)>,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for _ in 0..pending.outgoing_len() {
		let Some(message) = pending.pop_outgoing() else { break };
		let Some(&entity) = peer_metadata.peer_map.get(&message.peer()) else {
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
	peer_metadata: Option<Res<NetworkingMetadata>>,
	node: Single<Entity, With<ServerVoxelLoadNode>>,
	mut requester: VoxelRequester,
	mut pending: ResMut<PendingVoxelLoads>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	pending.remove_disconnected(|peer| peer_metadata.peer_map.contains_key(&peer), *node, &mut requester);
}
