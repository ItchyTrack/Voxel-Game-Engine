use bevy::camera::primitives::{Frustum, Sphere};
use bevy::camera::Camera;
use bevy::ecs::entity::Entity;
use bevy::math::Vec3A;
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};

use gpu_voxel_data::residency::ResidencyBuffers;
use gpu_voxel_data::sub_grid_gpu_state::SubGridGpuState;
use gpu_voxel_data::world_gpu_data::WorldGpuData;
use voxel_data::subgrid::{aabb_from_bounds, SubGrid};

use crate::hit_count_feedback::HitCountFeedback;

struct Candidate {
	entity: Entity,
	tree_id: u32,
	voxels_id: u32,
	tree_bytes: u32,
	voxel_bytes: u32,
	priority: f32,
}

/// Pick the sub-grids to keep resident this frame and copy them into the compact
/// buffers. A grid qualifies if it is in frustum or was hit last frame; the
/// frustum test keeps newly visible grids from popping in a frame late.
pub fn build_residency(
	mut residency: ResMut<ResidencyBuffers>,
	world_gpu: Res<WorldGpuData>,
	sub_grids: Query<(Entity, &SubGrid, &SubGridGpuState)>,
	grid_transforms: Query<&GlobalTransform>,
	cameras: Query<(&Camera, &GlobalTransform, &Frustum)>,
	hit_feedback: Res<HitCountFeedback>,
) {
	let view = cameras
		.iter()
		.find(|(c, _, _)| c.is_active)
		.map(|(_, global_transform, frustum)| (global_transform.translation(), frustum));
	let hit_counts = hit_feedback.0.lock().ok();

	let tree_alignment = residency.tree_alignment();
	let voxel_alignment = residency.voxel_alignment();

	let mut candidates: Vec<Candidate> = Vec::new();
	for (entity, sub_grid, gpu_state) in sub_grids.iter() {
		let Some(tree_held) = world_gpu
			.packed_64_tree_dynamic_buffer
			.held_buffer(gpu_state.tree_id())
		else {
			continue;
		};
		let Some(voxel_held) = world_gpu
			.packed_voxel_data_dynamic_buffer
			.held_buffer(gpu_state.voxels_id())
		else {
			continue;
		};

		let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue };
		let sub_world = grid_global.compute_transform()
			* Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
		let placement = gpu_state.placement();
		let (aabb_min, aabb_max) = aabb_from_bounds(placement.bounds_min, placement.bounds_max, &sub_world);

		let hit_count = hit_counts
			.as_ref()
			.and_then(|h| h.get(&entity).copied())
			.unwrap_or(0);

		let (in_view, priority) = match view {
			Some((cam_pos, frustum)) => {
				let center = (aabb_min + aabb_max) * 0.5;
				let radius = aabb_min.distance(aabb_max) * 0.5 + 1.0;
				let in_view = frustum.intersects_sphere(
					&Sphere { center: Vec3A::from(center), radius },
					true,
				);
				let distance = cam_pos.distance(center);
				(in_view, -distance / 1000.0 + if in_view { 5.0 } else { 0.0 })
			}
			None => (true, 0.0),
		};

		if !in_view && hit_count == 0 {
			continue;
		}

		candidates.push(Candidate {
			entity,
			tree_id: gpu_state.tree_id(),
			voxels_id: gpu_state.voxels_id(),
			tree_bytes: tree_held.size().next_multiple_of(tree_alignment),
			voxel_bytes: voxel_held.size().next_multiple_of(voxel_alignment),
			priority: priority + hit_count as f32 * 0.001,
		});
	}

	candidates.sort_by(|a, b| b.priority.total_cmp(&a.priority));

	let limit = residency.binding_limit();
	let mut tree_total = 0u64;
	let mut voxel_total = 0u64;
	let mut resident: Vec<(Entity, u32, u32)> = Vec::with_capacity(candidates.len());
	let mut dropped = 0usize;
	for candidate in &candidates {
		let next_tree = tree_total + candidate.tree_bytes as u64;
		let next_voxel = voxel_total + candidate.voxel_bytes as u64;
		if next_tree > limit || next_voxel > limit {
			dropped += 1;
			continue;
		}
		tree_total = next_tree;
		voxel_total = next_voxel;
		resident.push((candidate.entity, candidate.tree_id, candidate.voxels_id));
	}

	if dropped > 0 {
		log::warn!(
			"residency budget hit: {dropped} of {} sub-grids dropped this frame ({} resident)",
			candidates.len(),
			resident.len()
		);
	}

	residency.upload(&world_gpu, &resident);
}
