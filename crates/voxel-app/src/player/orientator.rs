use glam::Quat;

use crate::{world::{voxel_tracker::TrackedVoxelId, world::World}};

pub struct Orientator {
	tracked_voxel: Option<TrackedVoxelId>
}

impl Orientator {
	pub fn new() -> Self {
		Self {
			tracked_voxel: None
		}
	}

	pub fn set(&mut self, tracked_voxel: TrackedVoxelId) {
		self.tracked_voxel = Some(tracked_voxel);
	}

	pub fn reset(&mut self) {
		self.tracked_voxel = None;
	}

	pub fn has_voxel(&self) -> bool {
		self.tracked_voxel.is_some()
	}

	pub fn get_tracked_voxel(&self) -> Option<TrackedVoxelId> {
		self.tracked_voxel
	}

	pub fn hold_at_orientation(&self, orientation: &Quat, _dt: f32, world: &World) {
		if let Some(tracked_voxel) = self.tracked_voxel {
			if let Some(tracked_voxel) = world.voxel_tracker.read().get_tracked_voxel(tracked_voxel) {
				if let Some(body) = world.physics_body_mut(tracked_voxel.body_id) {
					if let Some(grid) = world.grid(tracked_voxel.grid_id) {
						let (axis, angle) = ((body.pose.rotation * grid.pose().rotation) * orientation.inverse()).to_axis_angle();
						let angular_velocity_in_dir = body.angular_velocity.dot(axis);
						let rotational_impulse = body.rotational_inertia().mat.as_mat3() * (
							axis * (
								-angle * 10.0 -
								angular_velocity_in_dir * 2.0
							) - (body.angular_velocity - axis * angular_velocity_in_dir)
						);
						world.apply_rotational_impulse(tracked_voxel.body_id, &rotational_impulse);
					}
				}
			} else {
				println!("Error tracked voxel body could not be found!");
			}
		}
	}
}
