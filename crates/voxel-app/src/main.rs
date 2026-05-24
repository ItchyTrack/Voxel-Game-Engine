mod audio;
mod audio_plugin;
mod camera_controller;
mod debug_toggles;
mod scene;
mod world_interaction;

use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa};

use audio_plugin::VoxelAudioPlugin;
use camera_controller::{FlyCamera, FlyCameraPlugin};
use debug_toggles::DebugTogglesPlugin;
use scene::ScenePlugin;
use voxel_physics::VoxelPhysicsPlugin;
use world_interaction::WorldInteractionPlugin;

#[tokio::main]
async fn main() {
	App::new()
		.add_plugins(DefaultPlugins)
		.add_plugins((
			voxel_renderer::VoxelRendererPlugin,
			VoxelPhysicsPlugin,
			ScenePlugin,
			FlyCameraPlugin,
			DebugTogglesPlugin,
			VoxelAudioPlugin,
			WorldInteractionPlugin,
		))
		.add_systems(Startup, setup)
		.run();
}

fn setup(mut commands: Commands) {
	commands.spawn((
		PointLight { shadows_enabled: false, ..default() },
		Transform::from_xyz(100.0, 100.0, 100.0),
	));

	commands.spawn((
		Camera3d::default(),
		Hdr,
		// Voxel coloring pipeline is built sample-count 1; keep the view in sync.
		Msaa::Off,
		Transform::from_xyz(0.0, 0.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
		FlyCamera::default(),
	));
}
