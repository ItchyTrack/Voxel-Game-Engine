mod debug;
mod grid_control;
mod policy;
mod render_set;

use bevy::prelude::*;
use lod_manager::{LodManagerPlugin, LodRequestMap};

pub use debug::{CameraLodDebug, CameraLodDebugChunk, CameraLodDebugState, FreezeCameraLods};
pub use grid_control::{apply_loaded_lod_deltas, release_lod_entity, retain_lod_entity, CameraLodGridControl};
pub use policy::{apply_camera_lod_policy, update_camera_lod_policy, CameraLazyLodScan, CameraLodPolicy, CameraLodTarget};
pub use render_set::CameraVoxelRenderSet;

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CameraLodSet {
	Policy,
	Swap,
}

#[derive(Default)]
pub struct CameraLodsPlugin;

impl Plugin for CameraLodsPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<LodManagerPlugin>() {
			app.add_plugins(LodManagerPlugin);
		}
		app.init_resource::<FreezeCameraLods>()
			.configure_sets(Update, (CameraLodSet::Policy, CameraLodSet::Swap).chain())
			.add_systems(Update, ensure_camera_lod_components)
			.add_systems(
				Update,
				(policy::update_camera_full_res_chunks, update_camera_lod_policy, apply_camera_lod_policy)
					.chain()
					.in_set(CameraLodSet::Policy),
			)
			.add_systems(Update, apply_loaded_lod_deltas.in_set(CameraLodSet::Swap));
	}
}

fn ensure_camera_lod_components(mut commands: Commands, cameras: Query<Entity, (With<Camera>, Added<Camera>)>) {
	for entity in cameras.iter() {
		commands.entity(entity).insert((
			LodRequestMap::default(),
			CameraLodGridControl::default(),
			CameraLodPolicy::default(),
			CameraLazyLodScan::default(),
			CameraVoxelRenderSet::default(),
			CameraLodDebug::default(),
		));
	}
}
