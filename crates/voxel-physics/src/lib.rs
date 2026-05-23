pub mod ball_joint_constraint;
pub mod collision_constraint;
pub mod physics_constraint;
pub mod inertia_tensor;
pub mod physics_body;
pub mod collision;
pub mod solver;
pub mod math;

use bevy::prelude::*;

use crate::physics_body::PhysicsBody;

#[derive(Default)]
pub struct VoxelPhysicsPlugin;

impl Plugin for VoxelPhysicsPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(voxel_data::VoxelDataPlugin);

		app.add_system(FixedUpdate, crate::update);

	}
}

fn update(
	grid_query: Query<&Grid>,
	physics_body_query: Query<&PhysicsBody>,
) {

}
