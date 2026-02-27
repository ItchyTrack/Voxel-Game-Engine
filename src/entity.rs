use glam::{Quat, Vec3};
use crate::voxels::Voxels;

pub struct Entity {
	pub position: Vec3,
	pub orientation: Quat,
	pub momentum: Vec3,
	pub angular_momentum: Vec3,
	pub id: u32,
	pub voxels: Voxels,
}
