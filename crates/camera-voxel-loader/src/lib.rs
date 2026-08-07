mod camera_voxel_loader;
mod coverage;
mod lod_bands;
mod lod_policy;
mod systems;
mod tile_lifecycle;
mod types;
mod unresolved_tile_index;

#[cfg(test)]
mod system_invariants;

use bevy::prelude::*;
use voxel_streaming::{StreamingPhase, StreamingSchedule, VoxelStreamingAppExt};

pub use crate::camera_voxel_loader::{CameraVoxelLoader, CameraVoxelLoaderSettings, CameraVoxelTileClass, CoverageDebugState, CoverageDebugTile};

voxel_streaming::chunk_consumer!(pub CameraVoxelLoaderConsumer);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraVoxelLoader(pub bool);

#[derive(Resource, Default, Debug, Clone)]
pub struct CameraVoxelLoaderDefaultSettings(pub CameraVoxelLoaderSettings);

fn ensure_camera_voxel_loader_components(
	mut commands: Commands, default_settings: Res<CameraVoxelLoaderDefaultSettings>,
	cameras: Query<(Entity, Option<&CameraVoxelLoader>, Option<&CameraVoxelLoaderConsumer>), With<Camera3d>>,
) {
	for (entity, loader, consumer) in &cameras {
		let mut entity_commands = commands.entity(entity);
		if loader.is_none() { entity_commands.insert(CameraVoxelLoader::with_settings(default_settings.0.clone())); }
		if consumer.is_none() { entity_commands.insert(CameraVoxelLoaderConsumer::default()); }
	}
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum CameraVoxelLoaderSet {
	RefreshVisibility,
}

#[derive(Default)]
pub struct CameraVoxelLoaderPlugin;

impl Plugin for CameraVoxelLoaderPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<FreezeCameraVoxelLoader>()
			.init_resource::<CameraVoxelLoaderDefaultSettings>()
			.register_chunk_consumer::<CameraVoxelLoaderConsumer>()
			.add_systems(Update, ensure_camera_voxel_loader_components)
			.add_systems(
				Update,
				systems::update_camera_voxel_loader_requests
					.run_if(|freeze: Res<FreezeCameraVoxelLoader>| !freeze.0)
					.in_set(StreamingPhase::Request)
					.before(CameraVoxelLoaderSet::RefreshVisibility),
			)
			.add_systems(StreamingSchedule, systems::receive_camera_voxel_loader_results.after(voxel_streaming::systems::publish_tile_updates).in_set(StreamingPhase::Receive))
			.add_systems(
				Update,
				systems::refresh_camera_voxel_loader_visibility.in_set(CameraVoxelLoaderSet::RefreshVisibility),
			);
	}
}
