mod debug;
mod grid_control;
mod policy;

use bevy::prelude::*;
use lod_manager::{LodManagerPlugin, LodRequestMap};

pub use debug::{CameraLodDebug, CameraLodDebugChunk, FreezeCameraLods};
pub use grid_control::{update_camera_lod_debug, CameraLodGridControl};
pub use policy::{apply_camera_lod_policy, update_camera_lod_policy, CameraLazyLodScan, CameraLodPolicy, CameraLodTarget};

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraLodSet {
	Policy,
	Debug,
}

#[derive(Default)]
pub struct CameraLodsPlugin;

impl Plugin for CameraLodsPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<LodManagerPlugin>() {
			app.add_plugins(LodManagerPlugin);
		}
		app.init_resource::<FreezeCameraLods>()
			.configure_sets(Update, (CameraLodSet::Policy, CameraLodSet::Debug).chain())
			.add_systems(Update, ensure_camera_lod_components)
			.add_systems(Update, (update_camera_lod_policy, apply_camera_lod_policy).chain().in_set(CameraLodSet::Policy))
			.add_systems(Update, update_camera_lod_debug.in_set(CameraLodSet::Debug));
	}
}

fn ensure_camera_lod_components(mut commands: Commands, cameras: Query<Entity, (With<Camera>, Added<Camera>)>) {
	for entity in cameras.iter() {
		commands.entity(entity).insert((
			LodRequestMap::default(),
			CameraLodGridControl::default(),
			CameraLodPolicy::default(),
			CameraLazyLodScan::default(),
			CameraLodDebug::default(),
		));
	}
}
