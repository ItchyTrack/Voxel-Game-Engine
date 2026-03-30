use glam::Vec3;

#[derive(Clone, Debug)]
pub struct CollisionAudioEvent {
	pub body1_id: u32,
	pub body2_id: u32,
	pub contact_position: Vec3,
	pub contact_normal: Vec3,
	pub pre_relative_velocity: Vec3,
	pub post_relative_velocity: Vec3,
	pub friction_coefficient: f32,
}
