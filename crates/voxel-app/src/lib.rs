mod audio;
mod networking;
mod scene;
mod ui;
mod voxel;

use std::time::Duration;

use bevy::log::LogPlugin;
use bevy::prelude::*;
#[cfg(target_arch = "wasm32")]
use bevy::render::settings::{Backends, RenderCreation, WgpuSettings};
use bevy::camera::{Hdr, PerspectiveProjection, Projection};
use bevy::render::view::Msaa;
#[cfg(target_arch = "wasm32")]
use bevy::render::RenderPlugin;
use bevy::window::WindowResolution;
#[cfg(target_arch = "wasm32")]
use bevy::window::ExitCondition;

use audio::plugin::VoxelAudioPlugin;
use scene::camera_controller::{FlyCamera, FlyCameraPlugin};
use ui::crosshair::CrosshairPlugin;
use ui::debug_toggles::DebugTogglesPlugin;
use ui::debug_ui::DebugUiPlugin;
use scene::gravity::GravityPlugin;
use voxel_content::VoxelStoreSourcePlugin;
#[cfg(not(target_arch = "wasm32"))]
use networking::network_client::NetworkClientPlugin;
#[cfg(not(target_arch = "wasm32"))]
use networking::network_server::NetworkServerPlugin;
use scene::scene::ScenePlugin;
use scene::skybox::SkyboxPlugin;
use voxel_engine::{VoxelCameraMode, VoxelEngineMode, VoxelEnginePlugins};
use voxel_sources::VoxelSourcesAppExt;
use scene::world_interaction::WorldInteractionPlugin;

use crate::voxel::lod_downsample::AverageVoxelLodGenerator;
use crate::voxel::planet_source::ProceduralPlanetPlugin;

struct VoxelLodGeneratorPlugin;

impl Plugin for VoxelLodGeneratorPlugin {
	fn build(&self, app: &mut App) {
		app.set_voxel_lod_generator(AverageVoxelLodGenerator);
	}
}

#[cfg(not(target_arch = "wasm32"))]
#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct SelectedClientId(pub u64);


pub fn build_app(window: Window) -> App {
	build_app_with_mode(window, selected_engine_mode())
}

pub fn build_app_with_mode(window: Window, mode: VoxelEngineMode) -> App {
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
		.insert_resource(Time::<Virtual>::from_max_delta(Duration::from_millis(16)));

	#[cfg(not(target_arch = "wasm32"))]
	app.insert_resource(SelectedClientId(selected_client_id()));

	app.add_plugins(VoxelEnginePlugins { mode })
		.add_plugins(VoxelLodGeneratorPlugin)
		.add_plugins(GravityPlugin)
		.add_systems(Startup, setup);

	match mode {
		VoxelEngineMode::Server => {
			app.add_plugins((VoxelStoreSourcePlugin, ScenePlugin, ProceduralPlanetPlugin));
			#[cfg(not(target_arch = "wasm32"))]
			app.add_plugins(NetworkServerPlugin);
		}
		VoxelEngineMode::Client => {
			#[cfg(not(target_arch = "wasm32"))]
			app.add_plugins(NetworkClientPlugin);
		}
		VoxelEngineMode::Host => {
			app.add_plugins((VoxelStoreSourcePlugin, ScenePlugin, ProceduralPlanetPlugin));
			#[cfg(not(target_arch = "wasm32"))]
			app.add_plugins(NetworkServerPlugin);
		}
	}

	if matches!(mode, VoxelEngineMode::Client | VoxelEngineMode::Host) {
		app.add_plugins((
			SkyboxPlugin,
			CrosshairPlugin,
			FlyCameraPlugin,
			DebugTogglesPlugin,
			WorldInteractionPlugin,
		));
		#[cfg(not(target_arch = "wasm32"))]
		app.add_plugins((DebugUiPlugin, VoxelAudioPlugin));
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
fn selected_engine_mode() -> VoxelEngineMode {
	let mut mode = VoxelEngineMode::Host;
	for arg in std::env::args().skip(1) {
		match arg.as_str() {
			"--server" => mode = VoxelEngineMode::Server,
			"--client" => mode = VoxelEngineMode::Client,
			"--host" => mode = VoxelEngineMode::Host,
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
fn selected_engine_mode() -> VoxelEngineMode {
	VoxelEngineMode::Server
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
		Projection::Perspective(PerspectiveProjection {
			far: 100_000.0,
			..default()
		}),
		Hdr,
		Msaa::Off,
		Transform::from_xyz(0.0, 0.0, 200.0).looking_at(Vec3::ZERO, Vec3::Y),
		FlyCamera::default(),
		VoxelCameraMode::Ray,
	));
}

