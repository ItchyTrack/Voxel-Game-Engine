use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerMetadata, RemoteEvent};
use voxel_streaming::GridStreaming;

use super::{PresenceLoad, PresenceRequest};
use crate::ReplicateVoxelsRestriction;

pub(super) fn receive_request(
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
	sender.trigger::<crate::chunks::ServerToClientChannel>(payload);
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
