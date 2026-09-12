use std::collections::HashMap;
use bevy::prelude::*;

use voxel_data::grid::GridId;
use voxel_trees::{grid_tree::U32Cell, region::NonZeroVoxelRegion, signed_grid_tree::SignedGridTree};

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TrackedVoxelId(u64);

#[derive(Default, Debug, Component)]
pub struct GlobalTrackedVoxels {
	last_grid_id: TrackedVoxelId,
	tracked_voxels: HashMap<TrackedVoxelId, (GridId, IVec3)>,
}

impl GlobalTrackedVoxels {
	pub fn get_tracked_voxel(&self, tracked_voxel_id: TrackedVoxelId) -> Option<(GridId, IVec3)> {
		self.tracked_voxels.get(&tracked_voxel_id).copied()
	}

	pub(crate) fn add_tracking(&mut self, grid_id: GridId, pos: IVec3) -> TrackedVoxelId {
		self.last_grid_id.0 += 1;
		self.tracked_voxels.insert(self.last_grid_id, (grid_id, pos));
		self.last_grid_id
	}

	pub(crate) fn update_tracking(&mut self, tracked_voxel_id: TrackedVoxelId, grid_id: GridId, pos: IVec3) {
		self.tracked_voxels.insert(tracked_voxel_id, (grid_id, pos));
	}

	pub(crate) fn remove_tracking(&mut self, tracked_voxel_id: TrackedVoxelId) -> Option<(GridId, IVec3)> {
		self.tracked_voxels.remove(&tracked_voxel_id)
	}
}

#[derive(Default, Debug, Component)]
pub struct GridTrackedVoxels {
	tracking_tree: SignedGridTree<U32Cell>,
	free_ids: Vec<u32>,
	tracked_voxels: Vec<Vec<TrackedVoxelId>>,
}

impl GridTrackedVoxels {
	pub fn get_tracked_voxel(&self, pos: IVec3) -> Option<&Vec<TrackedVoxelId>> {
		let internal_id = self.tracking_tree.get(pos)?;
		Some(&self.tracked_voxels[internal_id as usize])
	}

	// pub fn get_tracking(&self, region: NonZeroVoxelRegion) {
	// 	self.tracking_tree.
	// }

	pub(crate) fn add_tracking(&mut self, tracked_voxel_id: TrackedVoxelId, pos: IVec3) {
		if let Some(internal_id) = self.tracking_tree.get(pos) {
			let vec = &mut self.tracked_voxels[internal_id as usize];
			if let Err(index) = vec.binary_search_by(|probe| probe.0.cmp(&tracked_voxel_id.0)) {
				vec.insert(index, tracked_voxel_id);
			}
		} else if let Some(&internal_id) = self.free_ids.last() {
			self.free_ids.pop();
			self.tracking_tree.insert(pos, internal_id);
			self.tracked_voxels[internal_id as usize].push(tracked_voxel_id);
		} else {
			let internal_id = self.tracked_voxels.len() as u32;
			self.tracking_tree.insert(pos, internal_id);
			self.tracked_voxels.push(vec![tracked_voxel_id]);
		}
	}

	pub(crate) fn remove_tracking(&mut self, tracked_voxel_id: TrackedVoxelId, pos: IVec3) {
		let Some(internal_id) = self.tracking_tree.get(pos) else { return; };
		let vec = &mut self.tracked_voxels[internal_id as usize];

		let Ok(index) = vec.binary_search_by(|probe| probe.0.cmp(&tracked_voxel_id.0)) else { return; };
		vec.remove(index);

		if vec.is_empty() {
			self.tracking_tree.remove(pos);
			self.free_ids.push(internal_id);
		}
	}
}
