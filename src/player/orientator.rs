use glam::Quat;

use crate::physics::physics_engine::PhysicsEngine;

pub struct Orientator {
	tracked_voxel: Option<u64>
}

impl Orientator {
	pub fn new() -> Self {
		Self {
			tracked_voxel: None
		}
	}

	pub fn set(&mut self, tracked_voxel: u64) {
		self.tracked_voxel = Some(tracked_voxel);
	}

	pub fn reset(&mut self) {
		self.tracked_voxel = None;
	}

	pub fn has_voxel(&self) -> bool {
		self.tracked_voxel.is_some()
	}

	pub fn get_tracked_voxel(&self) -> Option<u64> {
		self.tracked_voxel
	}

	pub fn hold_at_orientation(&self, orientation: &Quat, physics_engine: &mut PhysicsEngine) {
		if let Some(tracked_voxel) = self.tracked_voxel {
			if let Some((body_id, grid_id, _voxel_pos)) = physics_engine.get_tracked_voxel(tracked_voxel) {
				if let Some(body) = physics_engine.physics_body_mut(body_id) {
					if let Some(grid) = body.grid(grid_id) {
						let (axis, angle) = ((body.pose.rotation * grid.pose.rotation) * orientation.inverse()).to_axis_angle();
						let angular_velocity_in_dir = body.angular_velocity.dot(axis);
						let rotational_impulse = body.rotational_inertia().mat.as_mat3() * (
							axis * (
								-angle * 4.0 -
								angular_velocity_in_dir / 2.0
							) - (body.angular_velocity - axis * angular_velocity_in_dir)
						);
						physics_engine.apply_rotational_impulse(body_id, &rotational_impulse);
					}
				}
			} else {
				println!("Error tracked voxel body could not be found!");
			}
		}
	}
}
