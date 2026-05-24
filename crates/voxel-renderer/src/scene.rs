use bevy::camera::primitives::{Frustum, Sphere};
use bevy::camera::Camera;
use bevy::ecs::message::MessageWriter;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, Res};
use bevy::math::{Vec3A};
use bevy::prelude::Entity;
use bevy::transform::components::{GlobalTransform, Transform};

use voxel_data::grid::Grid;
use voxel_data::sub_grid_gpu_state::{
	GpuStateRequestMessage, SubGridGpuUploadingState,
};

use crate::hit_count_feedback::HitCountFeedback;

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeUploads(pub bool);

pub fn request_dirty_subgrids(
	grids: Query<(Entity, &Grid, &GlobalTransform)>,
	cameras: Query<(&Camera, &GlobalTransform, &Frustum)>,
	freeze: Option<Res<FreezeUploads>>,
	hit_feedback: Res<HitCountFeedback>,
	mut messages: MessageWriter<GpuStateRequestMessage>,
) {
	if freeze.map(|f| f.0).unwrap_or(false) { return; }

	let view = cameras.iter().find(|(c, _, _)| c.is_active).map(|(_, gt, f)| (gt.translation(), f));
	let hit_counts = hit_feedback.0.lock().ok();

	for (entity, grid, grid_global_transform) in grids.iter() {
		for (sub_grid_id, sub_grid) in grid.sub_grids().iter() {
			let gpu_state = sub_grid.gpu_state();
			if gpu_state.currently_uploading().is_some() { continue; }

			let sub_world = grid_global_transform.compute_transform() * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
			let Some((aabb_min, aabb_max)) = sub_grid.aabb(&sub_world) else { continue };

			let hit_count = hit_counts
				.as_ref()
				.and_then(|h| h.get(&(entity, *sub_grid_id)).copied())
				.unwrap_or(1);

			let (priority, lod_level) = match view {
				Some((cam_pos, frustum)) => {
					let center = (aabb_min + aabb_max) * 0.5;
					let radius = aabb_min.distance(aabb_max) * 0.5 + 1.0;
					let in_view = frustum.intersects_sphere(
						&Sphere { center: Vec3A::from(center), radius },
						true,
					);
					let distance = cam_pos.distance(grid_global_transform.translation());
					let priority = -distance / 1000.0 + if in_view { 5.0 } else { 0.0 };
					let lod_level = f32::max(distance - 1000.0, 0.0) / 1000.0;
					let lod_level = if hit_count > 0 { lod_level } else { lod_level + 3.0 };
					(priority, lod_level)
				}
				None => (0.0, 0.0),
			};

			let curr_lod = gpu_state.lod_level();
			let lod_dropped_to_zero = lod_level != curr_lod && lod_level == 0.0;
			let lod_diverged = (curr_lod - lod_level).abs() > 0.25;
			let needs_upload = sub_grid.reupload_gpu_grid() || lod_dropped_to_zero || lod_diverged;
			if !needs_upload { continue; }

			messages.write(GpuStateRequestMessage::new(
				SubGridGpuUploadingState { lod_level },
				priority,
				entity,
				*sub_grid_id,
			));
		}
	}
}
