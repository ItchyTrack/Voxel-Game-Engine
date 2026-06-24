use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};
use voxel_data::grid::GridId;

use super::chunk_source::ClientChunkSource;
use crate::chunks::{ChunkRequest, ChunkResponse, LodRequest, LodResponse, PresenceLoad, PresenceRequest, RemoteChunkChanged};
use crate::ReplicateVoxels;

pub(crate) fn register_remote_voxel_grids(
	source: Res<ClientChunkSource>,
	grids: Query<GridId, With<ReplicateVoxels>>,
) {
	let mut remote_grids = source.state.remote_grids.lock().unwrap();
	for grid in &grids {
		remote_grids.insert(grid);
	}
}

pub(crate) fn flush_remote_presence_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<PresenceRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.presence_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<super::super::ClientToServerChannel>(request);
	}
}

pub(crate) fn flush_remote_chunk_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<ChunkRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.chunk_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<super::super::ClientToServerChannel>(request);
	}
}

pub(crate) fn flush_remote_lod_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<&mut EventSender<LodRequest>, With<Client>>,
) {
	let Ok(mut sender) = senders.single_mut() else { return };
	let mut requests = source.state.lod_requests.lock().unwrap();
	while let Some(request) = requests.pop_front() {
		sender.trigger::<super::super::ClientToServerChannel>(request);
	}
}

pub(crate) fn receive_remote_chunk_response(
	mut trigger: On<RemoteEvent<ChunkResponse>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let from = trigger.event().from;
	let response = &mut trigger.event_mut().trigger;
	source.state.remote_generations.lock().unwrap().receive_chunk_response(handle, from, response);
}

pub(crate) fn receive_remote_lod_response(
	mut trigger: On<RemoteEvent<LodResponse>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let from = trigger.event().from;
	let response = &mut trigger.event_mut().trigger;
	source.state.remote_generations.lock().unwrap().receive_lod_response(handle, from, response);
}

pub(crate) fn receive_presence_load(
	trigger: On<RemoteEvent<PresenceLoad>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event().trigger;
	if let Some((min, size)) = event.area {
		handle.claim(event.grid, min, size);
	}
	handle.presence_loaded(event.grid);
}

pub(crate) fn receive_remote_chunk_changed(
	trigger: On<RemoteEvent<RemoteChunkChanged>>,
	source: Res<ClientChunkSource>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event().trigger;
	source.state.remote_generations.lock().unwrap().receive_chunk_changed(handle, event);
}
