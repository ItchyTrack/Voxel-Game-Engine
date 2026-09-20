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
		Self { pose: Default::default(), region: VoxelRegion::ZERO }
	}
}
