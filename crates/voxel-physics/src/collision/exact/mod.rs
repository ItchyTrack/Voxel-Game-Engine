mod broadphase;
mod detect;
mod narrowphase;

use bevy::prelude::*;

use crate::collision::Collisions;
use crate::VoxelPhysicsAppExt;

use detect::detect_collisions;

#[derive(Default)]
pub struct ExactPlugin;

impl bevy::app::Plugin for ExactPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<Collisions>().add_physics_collision_systems(detect_collisions);
	}
}
