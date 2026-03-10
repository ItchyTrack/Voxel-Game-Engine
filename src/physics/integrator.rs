use glam::{ Quat, Vec3 };

use crate::{entity};

pub fn get_integrated_single(entity: &entity::Entity, dt: f32) -> (Vec3, Quat) {
	if entity.is_static || entity.mass() < f32::EPSILON {
		return (entity.position, entity.orientation);
	}
	let gravity = -90.0;
	let pos = entity.position + entity.velocity * dt + Vec3::new(0.0, gravity, 0.0) * (0.5 * dt * dt);
	let orientation = (Quat::from_scaled_axis(entity.angular_velocity * dt) * entity.orientation).normalize();
	( pos, orientation )
}
