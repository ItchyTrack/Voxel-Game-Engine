use bevy::ecs::message::MessageReader;
use bevy::log::warn;
use bevy::prelude::*;
use lightyear::prelude::{EventSender, PeerId, PeerMetadata};
use voxel_sources::ChunkChanged;

use super::{RemoteChunkChangeKind, RemoteChunkChanged};
use crate::ReplicateVoxelsRestriction;

pub(super) fn flush_changed(
	mut changed: MessageReader<ChunkChanged>,
	peer_metadata: Option<Res<PeerMetadata>>,
	grids: Query<Option<&ReplicateVoxelsRestriction>>,
	mut senders: Query<&mut EventSender<RemoteChunkChanged>>,
) {
	let Some(peer_metadata) = peer_metadata else { return };
	for event in changed.read().copied() {
		let kind = match event.kind {
			voxel_sources::ChunkChangeKind::Changed { generation } => RemoteChunkChangeKind::Changed { generation },
			voxel_sources::ChunkChangeKind::Removed { generation } => RemoteChunkChangeKind::Removed { generation },
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
			sender.trigger::<crate::chunks::ServerToClientChannel>(RemoteChunkChanged { grid: event.grid, min, size, kind });
		}
	}
}

fn intersect_aabbs(min_a: IVec3, size_a: IVec3, min_b: IVec3, size_b: IVec3) -> Option<(IVec3, IVec3)> {
	let max_a = min_a + size_a - IVec3::ONE;
	let max_b = min_b + size_b - IVec3::ONE;
	let min = min_a.max(min_b);
	let max = max_a.min(max_b);
	(max.cmpge(min).all()).then_some((min, max - min + IVec3::ONE))
}
