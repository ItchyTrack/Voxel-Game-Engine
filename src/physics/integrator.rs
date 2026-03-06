use glam::{ Quat, Vec3 };

use crate::{entity, math::add_vec_to_quat};

pub fn get_integrated_single(entity: &entity::Entity, dt: f32) -> (Vec3, Quat) {
	if entity.is_static {
		return (entity.position, entity.orientation);
	}
	let mut pos: Vec3 = entity.position;
	let mut orientation: Quat = entity.orientation;
	if entity.mass() == 0.0 { return ( pos, orientation ); }
	let gravity = -1.0;
	pos += entity.velocity * dt + Vec3::new(0.0, gravity, 0.0) * (dt * dt);
	orientation = add_vec_to_quat(&entity.orientation, &(entity.angular_velocity * dt));//(entity.angular_velocity * dt) * entity.orientation).normalize();
	( pos, orientation )
}