use bevy::camera::primitives::{Frustum, Sphere};
use bevy::camera::Camera;
use bevy::ecs::entity::Entity;
use bevy::math::Vec3A;
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};

use gpu_voxel_data::lod_voxels::LodVoxels;
use gpu_voxel_data::residency::ResidencyBuffers;
use gpu_voxel_data::sub_grid_gpu_state::SubGridGpuState;
use gpu_voxel_data::world_gpu_data::WorldGpuData;
use std::collections::HashSet;

use lod_manager::{LodRequestMap, LodVisibleKind};
use voxel_data::subgrid::{aabb_from_bounds, SubGrid};
use voxel_streaming::CHUNK_SIZE;

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
	lod_voxels: Query<(Entity, &LodVoxels, &SubGridGpuState)>,
	grid_transforms: Query<&GlobalTransform>,
	cameras: Query<(&Camera, &GlobalTransform, &Frustum, Option<&LodRequestMap>)>,
	hit_feedback: Res<HitCountFeedback>,
) {
	let active_camera = cameras
		.iter()
		.find(|(c, _, _, _)| c.is_active);
	let view = active_camera
		.map(|(_, global_transform, frustum, _)| (global_transform.translation(), frustum));
	let lod_requests = active_camera.and_then(|(_, _, _, requests)| requests);
	let render_subgrids: Option<HashSet<Entity>> = lod_requests.map(|requests| {
		requests.visible().iter().filter(|v| v.kind == LodVisibleKind::SubGrid).map(|v| v.entity).collect()
	});
	let render_lods: Option<HashSet<Entity>> = lod_requests.map(|requests| {
		requests.visible().iter().filter(|v| v.kind == LodVisibleKind::Lod).map(|v| v.entity).collect()
	});
	let hit_counts = hit_feedback.0.lock().ok();

	let tree_alignment = residency.tree_alignment();
	let voxel_alignment = residency.voxel_alignment();

	let make_candidate = |entity: Entity, gpu_state: &SubGridGpuState, aabb_min: Vec3, aabb_max: Vec3| -> Option<Candidate> {
		let tree_held = world_gpu.packed_64_tree_dynamic_buffer.held_buffer(gpu_state.tree_id())?;
		let voxel_held = world_gpu.packed_voxel_data_dynamic_buffer.held_buffer(gpu_state.voxels_id())?;

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
			return None;
		}

		Some(Candidate {
			entity,
			tree_id: gpu_state.tree_id(),
			voxels_id: gpu_state.voxels_id(),
			tree_bytes: tree_held.size().next_multiple_of(tree_alignment),
			voxel_bytes: voxel_held.size().next_multiple_of(voxel_alignment),
			priority: priority + hit_count as f32 * 0.001,
		})
	};

	let mut candidates: Vec<Candidate> = Vec::new();
	for (entity, sub_grid, gpu_state) in sub_grids.iter() {
		if render_subgrids.as_ref().is_some_and(|set| !set.contains(&entity)) { continue; }

		let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue };
		let sub_world = grid_global.compute_transform()
			* Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
		let placement = gpu_state.placement();
		let (aabb_min, aabb_max) = aabb_from_bounds(placement.bounds_min, placement.bounds_max, &sub_world);
		if let Some(candidate) = make_candidate(entity, gpu_state, aabb_min, aabb_max) {
			candidates.push(candidate);
		}
	}

	for (entity, lod_voxels, gpu_state) in lod_voxels.iter() {
		if render_lods.as_ref().is_some_and(|set| !set.contains(&entity)) { continue; }
		let Ok(grid_global) = grid_transforms.get(lod_voxels.grid) else { continue };
		let scale = (1u32 << lod_voxels.lod.max(0.0).floor() as u32) as f32;
		let area_origin = (lod_voxels.min * CHUNK_SIZE).as_vec3();
		let area_world = grid_global.compute_transform()
			* Transform::from_translation(area_origin)
			* Transform::from_scale(Vec3::splat(scale));
		let placement = gpu_state.placement();
		let (aabb_min, aabb_max) = aabb_from_bounds(placement.bounds_min, placement.bounds_max, &area_world);
		if let Some(candidate) = make_candidate(entity, gpu_state, aabb_min, aabb_max) {
			candidates.push(candidate);
		}
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
