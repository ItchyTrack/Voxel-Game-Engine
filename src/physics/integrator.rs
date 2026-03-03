use glam::{ Quat, Vec3 };

use crate::entity;

pub fn integrate(entities: &mut Vec<entity::Entity>, dt: f32) {
	for entity in entities {
		if entity.mass() == 0.0 { continue; }
		entity.position += entity.velocity * dt;
		let com = entity.center_of_mass();
		entity.position += entity.orientation * com;
		entity.orientation = (Quat::from_scaled_axis(entity.angular_velocity * dt) * entity.orientation).normalize();
		entity.position -= entity.orientation * com;
		// let rotational_inertia = Mat3::from_quat(entity.orientation) * entity.rotational_inertia() * Mat3::from_quat(entity.orientation.inverse());
		// let rotationa_velocity = rotational_inertia.inverse() * entity.angular_momentum;
	}
}

pub fn get_integrated_single(entity: &entity::Entity, dt: f32) -> (Vec3, Quat) {
	let mut pos: Vec3 = entity.position;
	let mut orientation: Quat = entity.orientation;
	if entity.mass() == 0.0 { return ( pos, orientation ); }
	pos += entity.velocity * dt;
	let com = entity.center_of_mass();
	pos += orientation * com;
	orientation = (Quat::from_scaled_axis(entity.angular_velocity * dt) * entity.orientation).normalize();
	pos -= orientation * com;
	( pos, orientation )
}