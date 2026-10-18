use bevy::prelude::*;
use bile_math::Region;

#[derive(Debug, Default)]
pub struct SceneTrasform(pub Transform);

#[derive(Component, Debug)]
pub struct SceneNode {
	pose: SceneTrasform,
	region: Region,
}

impl Default for SceneNode {
	fn default() -> Self {
		Self { pose: Default::default(), region: Region::new(IVec3::ZERO, UVec3::ZERO) }
	}
}
