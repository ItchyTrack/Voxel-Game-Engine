mod audio;
mod audio_plugin;
mod camera_controller;
mod debug_toggles;
mod debug_ui;
mod scene;
mod world_interaction;

use bevy::app::PluginGroupBuilder;
use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa};

use audio_plugin::VoxelAudioPlugin;
use bevy::window::WindowResolution;
use camera_controller::{FlyCamera, FlyCameraPlugin};
use debug_toggles::DebugTogglesPlugin;
use debug_ui::DebugUiPlugin;
use scene::ScenePlugin;
use voxel_physics::VoxelPhysicsPlugin;
use voxel_renderer::VoxelRendererPlugin;
use world_interaction::WorldInteractionPlugin;

/// All gameplay, rendering, physics, and debug plugins that make up the app.
struct GamePlugins;

impl PluginGroup for GamePlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(VoxelRendererPlugin)
			.add(VoxelPhysicsPlugin)
			.add(ScenePlugin)
			.add(FlyCameraPlugin)
			.add(DebugTogglesPlugin)
			.add(DebugUiPlugin)
			.add(VoxelAudioPlugin)
			.add(WorldInteractionPlugin)
	}
}

#[tokio::main]
async fn main() {
	App::new()
		.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: WindowResolution::new(800, 600),
                ..Default::default()
			}),
			..Default::default()
		}))
		.add_plugins(GamePlugins)
		.add_systems(Startup, setup)
		.run();
}

fn setup(mut commands: Commands) {
	commands.spawn((
		Camera3d::default(),
		Hdr,
		Msaa::Off,
		Transform::from_xyz(0.0, 0.0, 60.0).looking_at(Vec3::ZERO, Vec3::Y),
		FlyCamera::default(),
	));
}
