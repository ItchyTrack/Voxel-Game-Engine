use bevy::prelude::*;
use vge_math::region::VoxelRegion;

#[derive(Debug, Default)]
pub struct SceneTrasform(pub Transform);

#[derive(Component, Debug)]
pub struct SceneNode {
	pose: SceneTrasform,
	region: VoxelRegion,
}

impl Default for SceneNode {
	fn default() -> Self {
		Self { pose: Default::default(), region: VoxelRegion::new(IVec3::ZERO, UVec3::ZERO) }
	}
}

impl SceneNode {
	pub fn new(pose: SceneTrasform, region: VoxelRegion) -> Self {
		Self {
			pose,
			region,
		}
	}

	pub fn pose(&self) -> &SceneTrasform { &self.pose }
	pub fn move_node(&mut self, new_pose: SceneTrasform) {
		self.pose = new_pose;
	}
	pub fn region(&self) -> VoxelRegion { self.region }
	pub fn update_region(&mut self, new_region: VoxelRegion) {
		self.region = new_region;
	}
}
