use std::collections::HashMap;

use bevy::camera::{Camera, Projection};
use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, Res, ResMut};
use bevy::math::Vec3;
use bevy::render::Extract;
use bevy::transform::components::{GlobalTransform, Transform};

use voxel_gpu::lod_voxels::LodVoxels;
use voxel_gpu::residency::ResidencyBuffers;
use voxel_gpu::sub_grid_gpu_state::SubGridGpuState;
use voxel_gpu::world_gpu_data::WorldGpuData;
use voxel_data::bvh::BVH;
use voxel_data::subgrid::{aabb_from_bounds, SubGrid, SubGridId};

use crate::hit_count_feedback::HitCountFeedback;
use crate::voxel_camera::VoxelCamera;
use bevy::render::renderer::WgpuWrapper;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

#[derive(Resource, Default)]
pub struct ExtractedVoxelScene {
	pub camera_transform: Transform,
	pub camera_projection: Option<Projection>,
	pub bvh: Option<BVH<SubGridId>>,
	pub id_to_offsets: HashMap<SubGridId, (u32, u32, Transform)>,
	pub has_camera: bool,
	pub tree_buffer: Option<GpuBuffer>,
	pub voxel_buffer: Option<GpuBuffer>,
}

struct RenderItem {
	entity: Entity,
	tree_id: u32,
	voxels_id: u32,
	aabb: (Vec3, Vec3),
	dda_transform: Transform,
	priority: f32,
}

pub fn extract_voxel_scene(
	mut extracted: ResMut<ExtractedVoxelScene>,
	mut residency: ResMut<ResidencyBuffers>,
	hit_feedback: Res<HitCountFeedback>,
	cameras: Extract<Query<(&VoxelCamera, &Camera, &Projection, &GlobalTransform)>>,
	sub_grids: Extract<Query<(&SubGrid, &SubGridGpuState)>>,
	lod_voxels: Extract<Query<(&LodVoxels, &SubGridGpuState, &GlobalTransform)>>,
	grid_transforms: Extract<Query<&GlobalTransform>>,
	world_gpu: Extract<Res<WorldGpuData>>,
) {
	extracted.has_camera = false;
	extracted.bvh = None;
	extracted.id_to_offsets.clear();

	let mut items = Vec::new();

	for (voxel_camera, camera, projection, global_transform) in cameras.iter() {
		if !camera.is_active { continue; }
		extracted.camera_transform = global_transform.compute_transform();
		extracted.camera_projection = Some(projection.clone());
		extracted.has_camera = true;

		for entity in voxel_camera.subgrids_to_render.iter() {
			let Ok((sub_grid, sub_grid_gpu_state)) = sub_grids.get(*entity) else { continue; };
			let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue; };
			let sub_world = grid_global.compute_transform() * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
			let placement = sub_grid_gpu_state.placement();
			let aabb = aabb_from_bounds(placement.bounds_min, placement.bounds_max, &sub_world);
			let dda_transform = sub_world * Transform::from_translation(placement.tree_root_pos.as_vec3());
			let hit_count = hit_feedback.0.get(entity).copied().unwrap_or(0);

			items.push(RenderItem {
				entity: *entity,
				tree_id: sub_grid_gpu_state.tree_id(),
				voxels_id: sub_grid_gpu_state.voxels_id(),
				aabb,
				dda_transform,
				priority: (-global_transform.translation().distance((aabb.0 + aabb.1) * 0.5) / 1000.0) + hit_count as f32 * 0.001,
			});
		}

		for entity in voxel_camera.lods_to_render.iter() {
			let Ok((lod_grid, lod_grid_gpu_state, lod_global)) = lod_voxels.get(*entity) else { continue; };
			let scale = (1u32 << lod_grid.lod.max(0.0).floor() as u32) as f32;
			let area_world = lod_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			let placement = lod_grid_gpu_state.placement();
			let aabb = aabb_from_bounds(placement.bounds_min, placement.bounds_max, &area_world);
			let dda_transform = area_world * Transform::from_translation(placement.tree_root_pos.as_vec3());
			let hit_count = hit_feedback.0.get(entity).copied().unwrap_or(0);

			items.push(RenderItem {
				entity: *entity,
				tree_id: lod_grid_gpu_state.tree_id(),
				voxels_id: lod_grid_gpu_state.voxels_id(),
				aabb,
				dda_transform,
				priority: (-global_transform.translation().distance((aabb.0 + aabb.1) * 0.5) / 1000.0) + hit_count as f32 * 0.001,
			});
		}
		break;
	}
	if !extracted.has_camera { return; }

	let tree_alignment = residency.tree_alignment();
	let voxel_alignment = residency.voxel_alignment();

	items.sort_by(|a, b| b.priority.total_cmp(&a.priority));

	let limit = residency.binding_limit();
	let mut tree_total = 0u64;
	let mut voxel_total = 0u64;
	let mut resident: Vec<(Entity, u32, u32)> = Vec::with_capacity(items.len());
	let mut dropped = 0usize;
	for item in &items {
		let Some(tree_held) = world_gpu.packed_64_tree_dynamic_buffer.held_buffer(item.tree_id) else { continue; };
		let Some(voxel_held) = world_gpu.packed_voxel_data_dynamic_buffer.held_buffer(item.voxels_id) else { continue; };
		let next_tree = tree_total + tree_held.size().next_multiple_of(tree_alignment) as u64;
		let next_voxel = voxel_total + voxel_held.size().next_multiple_of(voxel_alignment) as u64;
		if next_tree > limit || next_voxel > limit {
			dropped += 1;
			continue;
		}
		tree_total = next_tree;
		voxel_total = next_voxel;
		resident.push((item.entity, item.tree_id, item.voxels_id));
	}

	if dropped > 0 {
		log::warn!(
			"residency budget hit: {dropped} of {} sub-grids dropped this frame ({} resident)",
			items.len(),
			resident.len()
		);
	}

	residency.upload(&world_gpu, &resident);

	let offsets = residency.offsets();
	let mut bvh_items: Vec<(SubGridId, (Vec3, Vec3))> = Vec::new();
	let mut id_to_offsets: HashMap<SubGridId, (u32, u32, Transform)> = HashMap::new();
	for item in &items {
		let Some(&(tree_offset, voxel_offset)) = offsets.get(&item.entity) else { continue; };
		bvh_items.push((item.entity, item.aabb));
		id_to_offsets.insert(item.entity, (tree_offset, voxel_offset, item.dda_transform));
	}

	if !bvh_items.is_empty() {
		extracted.bvh = Some(BVH::new(bvh_items));
	}
	extracted.id_to_offsets = id_to_offsets;

	extracted.tree_buffer = Some(residency.tree_buffer().clone());
	extracted.voxel_buffer = Some(residency.voxel_buffer().clone());
}
