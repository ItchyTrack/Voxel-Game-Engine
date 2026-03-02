use glam::{ Mat3, Quat };

use crate::entity;

pub fn integrate(entities: &mut Vec<entity::Entity>, dt: f32) {
	for entity in entities {
		if entity.mass() == 0.0 { continue; }
		entity.position += (entity.momentum / entity.mass()) * dt;
		let com = entity.center_of_mass();
		entity.position += entity.orientation * com;
		let rotational_inertia = Mat3::from_quat(entity.orientation) * entity.rotational_inertia() * Mat3::from_quat(entity.orientation.inverse());
		let rotationa_velocity = rotational_inertia.inverse() * entity.angular_momentum;
		entity.orientation = (Quat::from_scaled_axis(rotationa_velocity * dt) * entity.orientation).normalize();
		entity.position -= entity.orientation * com;
	}
}