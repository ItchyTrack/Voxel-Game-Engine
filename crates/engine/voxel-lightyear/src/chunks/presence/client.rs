use std::collections::{HashMap, VecDeque};

use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{Client, EventSender, RemoteEvent};
use voxel_data::grid::GridId;
use voxel_sources::{RequestId, SourceHandle};
use voxel_tasks::CancellationToken;

use super::{PresenceCancel, PresenceLoad, PresenceLoaded, PresenceRequest};
use crate::chunks::client_source::ClientChunkSource;
use crate::chunks::request_id::{NetworkRequestId, NetworkRequestIdAllocator};

struct PendingPresence {
	request_id: RequestId,
	grid: GridId,
	cancellation: CancellationToken,
}

#[derive(Default)]
pub(crate) struct ClientPresenceRegistry {
	ids: NetworkRequestIdAllocator,
	pending: HashMap<NetworkRequestId, PendingPresence>,
	requests: VecDeque<PresenceRequest>,
	cancellations: VecDeque<PresenceCancel>,
}

impl ClientPresenceRegistry {
	pub(crate) fn request(
		&mut self,
		request_id: RequestId,
		cancellation: CancellationToken,
		grid: GridId,
	) {
		let id = self.ids.allocate();
		self.pending.insert(id, PendingPresence { request_id, grid, cancellation });
		self.requests.push_back(PresenceRequest { id, grid });
	}

	fn drain_cancelled(&mut self) {
		let cancelled: Vec<_> = self.pending.iter()
			.filter_map(|(&id, pending)| pending.cancellation.is_cancelled().then_some(id))
			.collect();
		for id in cancelled {
			self.cancel(id);
		}
	}

	fn receive_load(&mut self, handle: &SourceHandle, from: impl std::fmt::Debug, load: PresenceLoad) {
		let Some(pending) = self.pending.get(&load.id) else { return };
		if pending.cancellation.is_cancelled() {
			self.cancel(load.id);
			return;
		}
		if pending.grid != load.grid {
			warn!(id=?load.id, expected=?pending.grid, actual=?load.grid, ?from, "ignoring mismatched remote presence payload");
			return;
		}
		handle.presence(pending.request_id, load.grid, load.region);
	}

	fn receive_loaded(&mut self, handle: &SourceHandle, loaded: PresenceLoaded) {
		let Some(pending) = self.pending.get(&loaded.id) else { return };
		if pending.cancellation.is_cancelled() {
			self.cancel(loaded.id);
			return;
		}
		let request_id = pending.request_id;
		self.pending.remove(&loaded.id);
		handle.presence_loaded(request_id);
	}

	fn cancel(&mut self, id: NetworkRequestId) {
		if self.pending.remove(&id).is_some() {
			self.cancellations.push_back(PresenceCancel { id });
		}
	}
}

pub(super) fn flush_requests(
	source: Res<ClientChunkSource>,
	mut senders: Query<(&mut EventSender<PresenceRequest>, &mut EventSender<PresenceCancel>), With<Client>>,
) {
	let Ok((mut request_sender, mut cancel_sender)) = senders.single_mut() else { return };
	let mut presence = source.state.presence.lock().unwrap();
	presence.drain_cancelled();
	while let Some(request) = presence.requests.pop_front() {
		request_sender.trigger::<crate::chunks::ClientToServerChannel>(request);
	}
	while let Some(cancel) = presence.cancellations.pop_front() {
		cancel_sender.trigger::<crate::chunks::ClientToServerChannel>(cancel);
	}
}

pub(super) fn receive_load(trigger: On<RemoteEvent<PresenceLoad>>, source: Res<ClientChunkSource>) {
	let Some(handle) = source.state.handle.get() else { return };
	let event = trigger.event();
	source.state.presence.lock().unwrap().receive_load(handle, event.from, event.trigger);
}

pub(super) fn receive_loaded(trigger: On<RemoteEvent<PresenceLoaded>>, source: Res<ClientChunkSource>) {
	let Some(handle) = source.state.handle.get() else { return };
	source.state.presence.lock().unwrap().receive_loaded(handle, trigger.event().trigger);
}
