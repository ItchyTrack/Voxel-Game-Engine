use bevy::camera::primitives::{Frustum, Sphere};
use bevy::camera::Camera;
use bevy::ecs::resource::Resource;
use bevy::math::Vec3A;
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};

use gpu_voxel_data::lod_requester;
use voxel_data::subgrid::SubGrid;

use crate::hit_count_feedback::HitCountFeedback;

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeUploads(pub bool);

lod_requester!(pub RenderLod);

pub fn update_render_lod(
	mut commands: Commands,
	mut sub_grids: Query<(Entity, &SubGrid, Option<&mut RenderLod>)>,
	grid_transforms: Query<&GlobalTransform>,
	cameras: Query<(&Camera, &GlobalTransform, &Frustum)>,
	freeze: Option<Res<FreezeUploads>>,
	hit_feedback: Res<HitCountFeedback>,
) {
	if freeze.map(|f| f.0).unwrap_or(false) { return; }

	let view = cameras
		.iter()
		.find(|(c, _, _)| c.is_active)
		.map(|(_, global_transform, frustum)| (global_transform.translation(), frustum));
	let hit_counts = hit_feedback.0.lock().ok();

	for (entity, sub_grid, render_lod) in sub_grids.iter_mut() {
		let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue };
		let sub_world = grid_global.compute_transform()
			* Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
		let Some((aabb_min, aabb_max)) = sub_grid.aabb(&sub_world) else { continue };

		let hit_count = hit_counts
			.as_ref()
			.and_then(|h| h.get(&entity).copied())
			.unwrap_or(1);

		let (priority, lod_level) = match view {
			Some((cam_pos, frustum)) => {
				let center = (aabb_min + aabb_max) * 0.5;
				let radius = aabb_min.distance(aabb_max) * 0.5 + 1.0;
				let in_view = frustum.intersects_sphere(
					&Sphere { center: Vec3A::from(center), radius },
					true,
				);
				let distance = cam_pos.distance(grid_global.translation());
				let priority = -distance / 1000.0 + if in_view { 5.0 } else { 0.0 };
				let lod_level = f32::max(distance - 1000.0, 0.0) / 1000.0;
				let lod_level = if hit_count > 0 { lod_level } else { lod_level + 3.0 };
				(priority, lod_level)
			}
			None => (0.0, 0.0),
		};

		match render_lod {
			Some(mut r) => { r.lod_level = lod_level; r.priority = priority; }
			None => { commands.entity(entity).insert(RenderLod { lod_level, priority }); }
		}
	}
}
