use bevy::prelude::*;

use crate::active_collision::detect_collisions;
use crate::collision::Collisions;
use crate::VoxelPhysicsAppExt;

#[derive(Default)]
pub struct ExactPlugin;

impl bevy::app::Plugin for ExactPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<Collisions>().add_physics_collision_systems(detect_collisions);
	}
}
