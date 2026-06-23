use std::collections::HashMap;

use bevy::math::IVec3;
use bevy::prelude::Component;
use lightyear::prelude::PeerId;
use serde::{Deserialize, Serialize};
use voxel_streaming::ChunkPresence;

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplicateVoxels;

#[derive(Component, Default)]
pub struct ReplicateVoxelsRestriction {
	readable_by_client: HashMap<PeerId, ChunkPresence>,
}

impl ReplicateVoxelsRestriction {
	pub fn set_readable_chunk(&mut self, peer: PeerId, chunk: IVec3) {
		self.readable_by_client.entry(peer).or_default().mark_present(chunk);
	}

	pub fn set_readable_area(&mut self, peer: PeerId, min: IVec3, size: IVec3) {
		self.readable_by_client.entry(peer).or_default().mark_present_area(min, size);
	}

	pub fn clear_readable_chunk(&mut self, peer: PeerId, chunk: IVec3) {
		if let Some(presence) = self.readable_by_client.get_mut(&peer) {
			presence.clear_present(chunk);
		}
	}

	pub fn clear_readable_area(&mut self, peer: PeerId, min: IVec3, size: IVec3) {
		if let Some(presence) = self.readable_by_client.get_mut(&peer) {
			presence.clear_present_area(min, size);
		}
	}

	pub fn is_readable_by(&self, peer: PeerId, chunk: IVec3) -> bool {
		self.readable_by_client.get(&peer).is_some_and(|presence| presence.is_present(chunk))
	}

	pub fn readable_aabb(&self, peer: PeerId) -> Option<(IVec3, IVec3)> {
		let presence = self.readable_by_client.get(&peer)?;
		let mut min = IVec3::splat(i32::MAX);
		let mut max = IVec3::splat(i32::MIN);
		let mut any = false;
		for (origin, extent) in presence.iter_present() {
			let node_max = origin + IVec3::splat(extent as i32 - 1);
			min = min.min(origin);
			max = max.max(node_max);
			any = true;
		}
		any.then_some((min, max - min + IVec3::ONE))
	}
}
