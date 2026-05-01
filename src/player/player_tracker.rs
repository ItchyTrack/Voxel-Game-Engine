use std::cell::Cell;

use glam::Vec3;

use crate::world::{pose::Pose, world::World, voxel_tracker::TrackedVoxelId};

pub struct PlayerTracker {
	voxel_to_move: Option<TrackedVoxelId>,
	error_avg: Cell<Vec3>,
}

impl PlayerTracker {
	pub fn new() -> Self {
		PlayerTracker {
			voxel_to_move: None,
			error_avg: Cell::new(Vec3::ZERO),
		}
	}

	pub fn set(&mut self, voxel_to_move: TrackedVoxelId) {
		self.voxel_to_move = Some(voxel_to_move);
	}

	pub fn reset(&mut self) {
		self.voxel_to_move = None;
	}

	pub fn has_voxel(&self) -> bool {
		self.voxel_to_move.is_some()
	}

	pub fn voxel_get_to_move(&self) -> Option<TrackedVoxelId> {
		self.voxel_to_move
	}

	// this trys to move the COM of the body to pose
	pub fn track_pos(&self, player_pos: &Vec3, _dt: f32, world: World) {
		if let Some(voxel_to_move) = self.voxel_to_move {
			if let Some(tracked_voxel) = world.voxel_tracker.read().get_tracked_voxel(voxel_to_move) {
				if let Some(body) = world.physics_body_mut(tracked_voxel.body_id) {
					if let Some(grid) = world.grid(tracked_voxel.grid_id) {
						let voxel_body_location = grid.grid_to_physics_body(&Pose::from_translation(tracked_voxel.voxel_pos.as_vec3()));
						let voxel_world_location = body.local_to_world(&voxel_body_location);

						let (_com, mass) = body.get_global_center_of_mass_and_mass();
						let error = player_pos - voxel_world_location.translation;
						let error_avg = (self.error_avg.get() * 5.0 + error) / 6.0;
						self.error_avg.set(error_avg);
						let dir = error.normalize();
						let velocity_in_dir = body.velocity.dot(dir);
						let central_impulse = mass * (
							dir * (
								error.length() * 0.2 -
								velocity_in_dir / 9.0 +
								error_avg.length() * 0.1
							) - (body.velocity - dir * velocity_in_dir)
						);
						world.voxel_tracker.write().apply_central_impulse(tracked_voxel.body_id, &Vec3::new(central_impulse.x, 0.0, central_impulse.z));
					}
				}
			}
		}
	}
}
