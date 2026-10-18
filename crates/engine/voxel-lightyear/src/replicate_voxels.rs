use std::collections::HashMap;

use bevy::math::IVec3;
use bevy::prelude::Component;
use bile_math::NonZeroRegion;
use lightyear::prelude::PeerId;
use serde::{Deserialize, Serialize};
use tile_data::{chunk_origin, chunk_region};
use voxel_trees::{grid_tree::U16Cell, signed_grid_tree::SignedGridTree};

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReplicateVoxels;

#[derive(Component, Default)]
pub struct ReplicateVoxelsRestriction {
	readable_by_client: HashMap<PeerId, SignedGridTree<U16Cell>>,
}

impl ReplicateVoxelsRestriction {
	pub fn set_readable_area(&mut self, peer: PeerId, region: NonZeroRegion) {
		self.readable_by_client.entry(peer).or_default().add_area(region, 0u16);
	}

	pub fn clear_readable_chunk(&mut self, peer: PeerId, chunk: IVec3) {
		self.clear_readable_area(peer, chunk_region(chunk));
	}

	pub fn clear_readable_area(&mut self, peer: PeerId, region: NonZeroRegion) {
		if let Some(readable) = self.readable_by_client.get_mut(&peer) {
			readable.remove_area(region);
		}
	}

	pub fn is_readable_by(&self, peer: PeerId, chunk: IVec3) -> bool {
		self.readable_by_client.get(&peer).is_some_and(|readable| readable.get(chunk_origin(chunk)).is_some())
	}

	pub fn readable_aabb(&self, peer: PeerId) -> Option<NonZeroRegion> {
		let readable = self.readable_by_client.get(&peer)?;
		let mut min = IVec3::splat(i32::MAX);
		let mut end = IVec3::splat(i32::MIN);
		for (origin, extent, _) in readable.iter() {
			min = min.min(origin);
			end = end.max(origin + IVec3::splat(extent as i32));
		}
		NonZeroRegion::from_min_end(min, end)
	}
}
