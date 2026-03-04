use glam::{ Quat, Vec3 };

use crate::entity;

pub fn get_integrated_single(entity: &entity::Entity, dt: f32) -> (Vec3, Quat) {
	if entity.is_static {
		return (entity.position, entity.orientation);
	}
	let mut pos: Vec3 = entity.position;
	let mut orientation: Quat = entity.orientation;
	if entity.mass() == 0.0 { return ( pos, orientation ); }
	let gravity = -2.0;
	pos += entity.velocity * dt + Vec3::new(0.0, gravity, 0.0) * (dt * dt);
	orientation = (Quat::from_scaled_axis(entity.angular_velocity * dt) * entity.orientation).normalize();
	( pos, orientation )
}