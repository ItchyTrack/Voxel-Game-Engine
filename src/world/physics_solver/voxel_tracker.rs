use std::collections::HashMap;

use glam::IVec3;

use super::super::{physics_body::PhysicsBodyId, grid::GridId};

pub struct TrackedVoxel {
	pub body_id: PhysicsBodyId,
	pub grid_id: GridId,
	pub voxel: IVec3,
}

pub struct VoxelTracker {
	tracked_voxels: HashMap<u64, TrackedVoxel>,
	next_id: u64,
}

impl VoxelTracker {
	pub fn new() -> Self {
		Self { tracked_voxels: HashMap::new(), next_id: 0 }
	}
	pub fn start_tracking(&mut self, body_id: PhysicsBodyId, grid_id: GridId, voxel: IVec3) -> u64 {
		self.tracked_voxels.insert(self.next_id, TrackedVoxel {
			body_id,
			grid_id,
			voxel,
		});
		self.next_id += 1;
		self.next_id - 1
	}
	pub fn stop_tracking(&mut self, tracked_voxel_id: u64) {
		self.tracked_voxels.remove(&tracked_voxel_id);
	}
	pub fn get_tracked_voxel(&self, tracked_voxel_id: u64) -> Option<&TrackedVoxel> {
		self.tracked_voxels.get(&tracked_voxel_id)
	}
}
