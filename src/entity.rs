use glam::{Quat, Vec3};
use crate::voxels::Voxels;

pub struct Entity {
	pub position: Vec3,
	pub rotation: Quat,
	pub velocity: Vec3,
	pub angular_velocity: Quat,
	pub id: u32,
	pub voxels: Voxels,
}
