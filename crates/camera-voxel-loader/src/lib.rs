mod camera_voxel_loader;
mod debug;
mod loading;
mod lod_policy;
mod retirement;
mod types;

use bevy::prelude::*;
use voxel_renderer::voxel_camera::VoxelCamera;
use voxel_streaming::{StreamingPhase, StreamingSchedule, VoxelStreamingAppExt};

use crate::camera_voxel_loader::CameraVoxelLoader;

voxel_streaming::chunk_consumer!(pub CameraVoxelLoaderConsumer);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraVoxelLoader(pub bool);

fn ensure_camera_voxel_loader_components(mut commands: Commands, cameras: Query<Entity, (With<Camera3d>, Without<CameraVoxelLoader>)>) {
	for entity in &cameras {
		commands.entity(entity).insert((CameraVoxelLoader::default(), CameraVoxelLoaderConsumer::default(), VoxelCamera::default()));
	}
}

#[derive(Default)]
pub struct CameraVoxelLoaderPlugin;

impl Plugin for CameraVoxelLoaderPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<FreezeCameraVoxelLoader>()
			.register_chunk_consumer::<CameraVoxelLoaderConsumer>()
			.add_systems(Update, ensure_camera_voxel_loader_components)
			.add_systems(
				Update,
				loading::update_camera_voxel_loader_requests
					.run_if(|freeze: Res<FreezeCameraVoxelLoader>| !freeze.0)
					.in_set(StreamingPhase::Request),
			)
			.add_systems(
				StreamingSchedule,
				loading::receive_camera_voxel_loader_results
					.after(voxel_streaming::receive_lod_results)
					.in_set(StreamingPhase::Receive),
			)
			.add_systems(
				Update,
				(
					loading::refresh_camera_voxel_loader_visibility,
					loading::retire_replaced_tiles,
					loading::retire_replaced_chunks,
				)
					.chain()
					.after(gpu_voxel_data::GpuUploadSet::Upload),
			);
			// .add_systems(Update, debug::draw_lod_policy_bounds_gizmos);
		// .add_systems(Update, (debug::draw_policy_delta_gizmos, debug::draw_retiring_lod_gizmos));
	}
}
