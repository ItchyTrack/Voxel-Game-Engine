use std::collections::HashMap;

use glam::IVec3;

use super::{physics_body::PhysicsBodyId, grid::GridId};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TrackedVoxelId(u64);

#[derive(Debug, Clone, Copy)]
pub struct TrackedVoxel {
	pub body_id: PhysicsBodyId,
	pub grid_id: GridId,
	pub voxel_pos: IVec3,
}

pub struct VoxelTracker {
	tracked_voxels: HashMap<TrackedVoxelId, TrackedVoxel>,
	next_id: TrackedVoxelId,
}

impl VoxelTracker {
	pub fn new() -> Self {
		Self { tracked_voxels: HashMap::new(), next_id: TrackedVoxelId(0) }
	}
	pub fn start_tracking(&mut self, body_id: PhysicsBodyId, grid_id: GridId, voxel_pos: IVec3) -> TrackedVoxelId {
		let tracked_voxel_id = self.next_id;
		self.tracked_voxels.insert(tracked_voxel_id, TrackedVoxel {
			body_id,
			grid_id,
			voxel_pos,
		});
		self.next_id.0 += 1;
		tracked_voxel_id
	}
	pub fn stop_tracking(&mut self, tracked_voxel_id: TrackedVoxelId) {
		self.tracked_voxels.remove(&tracked_voxel_id);
	}
	pub fn get_tracked_voxel(&self, tracked_voxel_id: TrackedVoxelId) -> Option<TrackedVoxel> {
		self.tracked_voxels.get(&tracked_voxel_id).cloned()
	}
}
