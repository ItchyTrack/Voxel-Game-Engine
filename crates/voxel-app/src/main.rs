mod audio;
mod audio_plugin;
mod camera_controller;
mod crosshair;
mod debug_toggles;
mod debug_ui;
mod lod_downsample;
mod memory_store;
mod scene;
mod skybox;
mod sphere_source;
mod streaming_test;
mod vox_loader;
mod world_interaction;

use std::time::Duration;

use bevy::app::PluginGroupBuilder;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa};

use audio_plugin::VoxelAudioPlugin;
use bevy::window::WindowResolution;
use camera_controller::{FlyCamera, FlyCameraPlugin};
use camera_voxel_loader::CameraVoxelLoaderPlugin;
use crosshair::CrosshairPlugin;
use skybox::SkyboxPlugin;
use sphere_source::SphereSourcePlugin;
use debug_toggles::DebugTogglesPlugin;
use debug_ui::DebugUiPlugin;
use scene::ScenePlugin;
use streaming_test::StreamingTestPlugin;
use voxel_physics::VoxelPhysicsPlugin;
use voxel_renderer::VoxelRendererPlugin;
use voxel_edit::VoxelEditPlugin;
use voxel_sources::{VoxelSourcesAppExt, VoxelSourcesPlugin};
use voxel_streaming::VoxelStreamingPlugin;
use memory_store::MemoryStorePlugin;
use crate::lod_downsample::AverageVoxelLodGenerator;
use world_interaction::WorldInteractionPlugin;

struct VoxelLodGeneratorPlugin;

impl Plugin for VoxelLodGeneratorPlugin {
	fn build(&self, app: &mut App) {
		app.set_voxel_lod_generator(AverageVoxelLodGenerator);
	}
}

/// All gameplay, rendering, physics, and debug plugins that make up the app.
struct GamePlugins;

impl PluginGroup for GamePlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(VoxelEditPlugin)
			.add(VoxelStreamingPlugin)
			.add(VoxelSourcesPlugin)
			.add(VoxelLodGeneratorPlugin)
			.add(MemoryStorePlugin)
			// .add(SphereSourcePlugin)
			.add(CameraVoxelLoaderPlugin)
			.add(VoxelRendererPlugin)
			.add(SkyboxPlugin)
			.add(CrosshairPlugin)
			.add(VoxelPhysicsPlugin)
			.add(ScenePlugin)
			.add(StreamingTestPlugin)
			.add(FlyCameraPlugin)
			.add(DebugTogglesPlugin)
			.add(DebugUiPlugin)
			.add(VoxelAudioPlugin)
			.add(WorldInteractionPlugin)
	}
}

fn main() {
	let runtime = tokio::runtime::Builder::new_multi_thread()
		.enable_all()
		.build()
		.unwrap();
	let guard = runtime.enter();

	let mut app = App::new();
	app.add_plugins(
		DefaultPlugins
			.set(WindowPlugin {
				primary_window: Some(Window {
					resolution: WindowResolution::new(800, 600),
					..Default::default()
				}),
				..Default::default()
			})
			.set(LogPlugin {
				custom_layer: tracy_layer,
				..Default::default()
			}),
	)
		.insert_resource(Time::<Virtual>::from_max_delta(Duration::from_millis(16)))
		.add_plugins(GamePlugins)
		.add_systems(Startup, setup);

	#[cfg(feature = "tracy")]
	app.add_systems(Last, || tracing_tracy::client::frame_mark());

	app.run();

	// Don't block on in-flight load/upload tasks; abandon them so the window closes
	// promptly. The OS reclaims the worker threads on process exit.
	drop(app);
	drop(guard);
	runtime.shutdown_background();
}

fn tracy_layer(_app: &mut App) -> Option<bevy::log::BoxedLayer> {
	#[cfg(feature = "tracy")]
	{
		tracing_tracy::client::Client::start();
		Some(Box::new(tracing_tracy::TracyLayer::default()))
	}
	#[cfg(not(feature = "tracy"))]
	None
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
