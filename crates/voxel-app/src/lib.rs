mod audio;
mod networking;
mod scene;
mod ui;
mod voxel;

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

use audio::plugin::VoxelAudioPlugin;
use scene::camera_controller::{FlyCamera, FlyCameraPlugin};
use camera_voxel_loader::CameraVoxelLoaderPlugin;
use ui::crosshair::CrosshairPlugin;
use ui::debug_toggles::DebugTogglesPlugin;
use ui::debug_ui::DebugUiPlugin;
use scene::gravity::GravityPlugin;
use voxel::memory_store::MemoryStorePlugin;
#[cfg(not(target_arch = "wasm32"))]
use networking::network_client::NetworkClientPlugin;
#[cfg(not(target_arch = "wasm32"))]
use networking::network_server::NetworkServerPlugin;
use scene::scene::ScenePlugin;
use scene::skybox::SkyboxPlugin;
use voxel::streaming_test::StreamingTestPlugin;
use voxel_data::VoxelDataPlugin;
use voxel_edit::VoxelEditPlugin;
use voxel_gpu::GpuVoxelDataPlugin;
use voxel_lightyear::VoxelLightyearPlugins;
use voxel_physics::VoxelPhysicsPlugin;
use voxel_renderer::VoxelRendererPlugin;
use voxel_sources::VoxelSourcesAppExt;
use voxel_streaming::VoxelStreamingPlugin;
use scene::world_interaction::WorldInteractionPlugin;

use crate::voxel::lod_downsample::AverageVoxelLodGenerator;

struct VoxelLodGeneratorPlugin;

impl Plugin for VoxelLodGeneratorPlugin {
	fn build(&self, app: &mut App) {
		app.set_voxel_lod_generator(AverageVoxelLodGenerator);
	}
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NetworkMode {
	#[default]
	Server,
	Client,
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct SelectedClientId(pub u64);

struct SharedPlugins;
struct LocalPlayerPlugins;
struct ServerPlugins;
struct ClientPlugins;

impl PluginGroup for SharedPlugins {
	fn build(self) -> PluginGroupBuilder {
		PluginGroupBuilder::start::<Self>()
			.add(VoxelDataPlugin)
			.add(VoxelEditPlugin)
			.add(VoxelStreamingPlugin)
			.add(VoxelLodGeneratorPlugin)
			.add(VoxelPhysicsPlugin)
			.add(GravityPlugin)
	}
}

impl PluginGroup for LocalPlayerPlugins {
	fn build(self) -> PluginGroupBuilder {
		let group = PluginGroupBuilder::start::<Self>()
			.add(CameraVoxelLoaderPlugin)
			.add(VoxelRendererPlugin)
			.add(SkyboxPlugin)
			.add(CrosshairPlugin)
			.add(FlyCameraPlugin)
			.add(DebugTogglesPlugin)
			.add(WorldInteractionPlugin);
		#[cfg(not(target_arch = "wasm32"))]
		let group = group.add(DebugUiPlugin).add(VoxelAudioPlugin);
		group
	}
}

impl PluginGroup for ServerPlugins {
	fn build(self) -> PluginGroupBuilder {
		let group = PluginGroupBuilder::start::<Self>()
			.add_group(VoxelLightyearPlugins {
				enable_client_chunk_source: false,
				enable_server_chunk_source: true,
			})
			.add(MemoryStorePlugin)
			// .add(SphereSourcePlugin)
			.add(ScenePlugin)
			.add(StreamingTestPlugin)
			.add_group(LocalPlayerPlugins);
		#[cfg(not(target_arch = "wasm32"))]
		let group = group.add(NetworkServerPlugin);
		group
	}
}

impl PluginGroup for ClientPlugins {
	fn build(self) -> PluginGroupBuilder {
		let group = PluginGroupBuilder::start::<Self>()
			.add_group(VoxelLightyearPlugins {
				enable_client_chunk_source: true,
				enable_server_chunk_source: false,
			})
			.add_group(LocalPlayerPlugins);
		#[cfg(not(target_arch = "wasm32"))]
		let group = group.add(NetworkClientPlugin);
		group
	}
}

pub fn build_app(window: Window) -> App {
	build_app_with_mode(window, selected_network_mode())
}

pub fn build_app_with_mode(window: Window, network_mode: NetworkMode) -> App {
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
		.add_plugins(GpuVoxelDataPlugin)
		.add_plugins(SharedPlugins);

	#[cfg(not(target_arch = "wasm32"))]
	app.insert_resource(SelectedClientId(selected_client_id()));

	match network_mode {
		NetworkMode::Server => {
			app.add_plugins(ServerPlugins)
				.add_systems(Startup, setup);
		}
		NetworkMode::Client => {
			app.add_plugins(ClientPlugins)
				.add_systems(Startup, setup);
		}
	}

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
fn selected_network_mode() -> NetworkMode {
	let mut mode = NetworkMode::Server;
	for arg in std::env::args().skip(1) {
		match arg.as_str() {
			"--server" => mode = NetworkMode::Server,
			"--client" => mode = NetworkMode::Client,
			_ => {}
		}
	}
	mode
}

#[cfg(not(target_arch = "wasm32"))]
fn selected_client_id() -> u64 {
	let mut args = std::env::args().skip(1);
	while let Some(arg) = args.next() {
		if arg == "--client" {
			if let Some(id) = args.next()
				&& let Ok(id) = id.parse::<u64>() {
				return id;
			}
			break;
		}
	}
	1
}

#[cfg(target_arch = "wasm32")]
fn selected_network_mode() -> NetworkMode {
	NetworkMode::Server
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

