mod audio;
mod audio_plugin;
mod camera_controller;
mod crosshair;
mod debug_toggles;
mod debug_ui;
mod gravity;
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
#[cfg(target_arch = "wasm32")]
use bevy::render::settings::{Backends, RenderCreation, WgpuSettings};
use bevy::render::view::{Hdr, Msaa};
#[cfg(target_arch = "wasm32")]
use bevy::render::RenderPlugin;
use bevy::window::WindowResolution;
#[cfg(target_arch = "wasm32")]
use bevy::window::ExitCondition;

use audio_plugin::VoxelAudioPlugin;
use camera_controller::{FlyCamera, FlyCameraPlugin};
use camera_voxel_loader::CameraVoxelLoaderPlugin;
use crosshair::CrosshairPlugin;
use debug_toggles::DebugTogglesPlugin;
use debug_ui::DebugUiPlugin;
use gravity::GravityPlugin;
use memory_store::MemoryStorePlugin;
use scene::ScenePlugin;
use skybox::SkyboxPlugin;
use sphere_source::SphereSourcePlugin;
use streaming_test::StreamingTestPlugin;
use voxel_edit::VoxelEditPlugin;
use voxel_physics::VoxelPhysicsPlugin;
use voxel_renderer::VoxelRendererPlugin;
use voxel_sources::{VoxelSourcesAppExt, VoxelSourcesPlugin};
use voxel_streaming::VoxelStreamingPlugin;
use world_interaction::WorldInteractionPlugin;

use crate::lod_downsample::AverageVoxelLodGenerator;

struct VoxelLodGeneratorPlugin;

impl Plugin for VoxelLodGeneratorPlugin {
	fn build(&self, app: &mut App) {
		app.set_voxel_lod_generator(AverageVoxelLodGenerator);
	}
}

struct GamePlugins;

impl PluginGroup for GamePlugins {
	fn build(self) -> PluginGroupBuilder {
		let group = PluginGroupBuilder::start::<Self>()
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
			.add(GravityPlugin)
			.add(ScenePlugin)
			.add(StreamingTestPlugin)
			.add(FlyCameraPlugin)
			.add(DebugTogglesPlugin)
			.add(WorldInteractionPlugin);
		#[cfg(not(target_arch = "wasm32"))]
		let group = group.add(DebugUiPlugin);
		#[cfg(not(target_arch = "wasm32"))]
		let group = group.add(VoxelAudioPlugin);
		group
	}
}

pub fn build_app(window: Window) -> App {
	let mut app = App::new();
	#[cfg(not(target_arch = "wasm32"))]
	let default_plugins = DefaultPlugins.build()
		.set(WindowPlugin {
			primary_window: Some(window),
			..Default::default()
		})
		.set(LogPlugin {
			custom_layer: tracy_layer,
			..Default::default()
		});
	#[cfg(target_arch = "wasm32")]
	let default_plugins = DefaultPlugins.build()
		.disable::<bevy::winit::WinitPlugin>()
		.disable::<bevy::gilrs::GilrsPlugin>()
		.disable::<bevy::audio::AudioPlugin>()
		.disable::<bevy::log::LogPlugin>()
		.set(WindowPlugin {
			primary_window: Some(window),
			exit_condition: ExitCondition::DontExit,
			..Default::default()
		})
		.set(LogPlugin {
			custom_layer: tracy_layer,
			..Default::default()
		})
		.set(RenderPlugin {
			render_creation: RenderCreation::Automatic(WgpuSettings {
				backends: Some(Backends::BROWSER_WEBGPU),
				..Default::default()
			}),
			..Default::default()
		});
	app.add_plugins(default_plugins)
		.insert_resource(Time::<Virtual>::from_max_delta(Duration::from_millis(16)))
		.add_plugins(GamePlugins)
		.add_systems(Startup, setup);

	#[cfg(feature = "tracy")]
	app.add_systems(Last, || tracing_tracy::client::frame_mark());

	app
}

fn run_app() {
	let mut app = build_app(Window {
		resolution: WindowResolution::new(800, 600),
		canvas: Some("#canvas".into()),
		..Default::default()
	});
	app.run();
}

#[cfg(not(target_arch = "wasm32"))]
pub fn run_native_app() {
	let runtime = tokio::runtime::Builder::new_multi_thread()
		.enable_time()
		.build()
		.unwrap();
	let guard = runtime.enter();

	run_app();

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

