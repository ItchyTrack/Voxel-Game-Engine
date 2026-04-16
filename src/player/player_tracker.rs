use std::cell::Cell;

use glam::Vec3;

use crate::{physics::physics_engine::PhysicsEngine, pose::Pose};

pub struct PlayerTracker {
	voxel_to_move: Option<u64>,
	error_avg: Cell<Vec3>,
}

impl PlayerTracker {
	pub fn new() -> Self {
		PlayerTracker {
			voxel_to_move: None,
			error_avg: Cell::new(Vec3::ZERO),
		}
	}

	pub fn set(&mut self, voxel_to_move: u64) {
		self.voxel_to_move = Some(voxel_to_move);
	}

	pub fn reset(&mut self) {
		self.voxel_to_move = None;
	}

	pub fn has_voxel(&self) -> bool {
		self.voxel_to_move.is_some()
	}

	pub fn voxel_get_to_move(&self) -> Option<u64> {
		self.voxel_to_move
	}

	// this trys to move the COM of the body to pose
	pub fn track_pos(&self, player_pos: &Vec3, physics_engine: &mut PhysicsEngine) {
		if let Some(voxel_to_move) = self.voxel_to_move {
			if let Some((body_id, grid_id, voxel_pos)) = physics_engine.get_tracked_voxel(voxel_to_move) {
				if let Some(body) = physics_engine.physics_body_mut(body_id) {
					if let Some(grid) = body.grid(grid_id) {
						let voxel_body_location = grid.local_to_body(&Pose::from_translation(voxel_pos.as_vec3()));
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
						physics_engine.apply_central_impulse(body_id, &Vec3::new(central_impulse.x, 0.0, central_impulse.z));
					}
				}
			}
		}
	}
}
