use glam::Vec3;

use crate::{physics::physics_engine::PhysicsEngine};

pub struct ObjectPickup {
	body_id: Option<u32>
}

impl ObjectPickup {
	pub fn new() -> Self {
		ObjectPickup {
			body_id: None
		}
	}

	pub fn set(&mut self, body_id: u32) {
		self.body_id = Some(body_id);
	}

	pub fn reset(&mut self) {
		self.body_id = None;
	}

	pub fn is_holding(&self) -> bool {
		self.body_id.is_some()
	}

	pub fn get_holding(&self) -> Option<u32> {
		self.body_id
	}

	// this trys to move the COM of the body to pose
	pub fn hold_at_pos(&self, pos: &Vec3, physics_engine: &mut PhysicsEngine) {
		if let Some(body_id) = self.body_id {
			if let Some(body) = physics_engine.physics_body_mut(body_id) {
				let (com, mass) = body.get_global_center_of_mass_and_mass();
				let dir = (pos - com).normalize();
				let velocity_in_dir = body.velocity.dot(dir);
				body.apply_central_impulse(&(mass * (
					dir * (
						(pos - com).length() * 4.0 -
						velocity_in_dir / 2.0
					) - (body.velocity - dir * velocity_in_dir)
				)));
				let (axis, angle) = body.pose.rotation.to_axis_angle();
				let angular_velocity_in_dir = body.angular_velocity.dot(axis);
				body.apply_rotational_impulse(&(body.rotational_inertia().mat.as_mat3() * (
					axis * (
						-angle * 4.0 -
						angular_velocity_in_dir / 2.0
					) - (body.angular_velocity - axis * angular_velocity_in_dir)
				)));
			}
		}
	}
}
