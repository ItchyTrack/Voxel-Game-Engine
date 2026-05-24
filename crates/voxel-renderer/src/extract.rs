use std::collections::HashMap;

use bevy::camera::{Camera, Projection};
use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, ResMut};
use bevy::math::Vec3;
use bevy::render::Extract;
use bevy::transform::components::{GlobalTransform, Transform};

use voxel_data::bvh::bvh::BVH;
use voxel_data::grid::{Grid, SubGridId};
use voxel_data::world_gpu_data::WorldGpuData;

#[derive(Resource, Default)]
pub struct ExtractedVoxelScene {
	pub camera_transform: Transform,
	pub camera_projection: Option<Projection>,
	pub bvh: Option<BVH<(Entity, SubGridId)>>,
	pub id_to_offsets: HashMap<(Entity, SubGridId), (u32, u32, Transform)>,
	pub has_camera: bool,
	pub tree_buffer: Option<wgpu::Buffer>,
	pub voxel_buffer: Option<wgpu::Buffer>,
}

pub fn extract_voxel_scene(
	mut extracted: ResMut<ExtractedVoxelScene>,
	cameras: Extract<Query<(&Camera, &Projection, &GlobalTransform)>>,
	grids: Extract<Query<(Entity, &Grid, Option<&GlobalTransform>)>>,
	world_gpu_data: Extract<bevy::ecs::system::Res<WorldGpuData>>,
) {
	extracted.has_camera = false;
	extracted.bvh = None;
	extracted.id_to_offsets.clear();

	for (camera, projection, global_transform) in cameras.iter() {
		if !camera.is_active { continue; }
		extracted.camera_transform = global_transform.compute_transform();
		extracted.camera_projection = Some(projection.clone());
		extracted.has_camera = true;
		break;
	}
	if !extracted.has_camera { return; }

	let mut bvh_items: Vec<((Entity, SubGridId), (Vec3, Vec3))> = Vec::new();
	let mut id_to_offsets: HashMap<(Entity, SubGridId), (u32, u32, Transform)> = HashMap::new();

	for (entity, grid, maybe_global) in grids.iter() {
		let grid_global = maybe_global
			.map(|gt| gt.compute_transform() * *grid.transform())
			.unwrap_or(*grid.transform());

		for (sub_grid_id, sub_grid) in grid.sub_grids().iter() {
			let gpu_state = sub_grid.gpu_state();
			if !gpu_state.on_gpu() { continue; }

			// Shader treats item_index / item_index_2 as BYTE offsets into the
			// packed tree / voxel buffers.
			let Some(tree_held) = world_gpu_data
				.packed_64_tree_dynamic_buffer
				.get_held_buffer(gpu_state.tree_id()) else { continue };
			let Some(voxel_held) = world_gpu_data
				.packed_voxel_data_dynamic_buffer
				.get_held_buffer(gpu_state.voxels_id()) else { continue };

			let sub_world = grid_global * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
			let Some(aabb) = sub_grid.aabb(&sub_world) else { continue };

			// The GridTree's `root_pos` shifts inside the sub-grid as voxels
			// are inserted; DDA expects origins in tree-local space, so bake
			// it into the per-item transform.
			let (_, tree_root_pos, _) = sub_grid.get_voxels().get_voxels().get_internals();
			let dda_transform = sub_world * Transform::from_translation(tree_root_pos.as_vec3());

			bvh_items.push(((entity, *sub_grid_id), aabb));
			id_to_offsets.insert(
				(entity, *sub_grid_id),
				(tree_held.offset(), voxel_held.offset(), dda_transform),
			);
		}
	}

	if !bvh_items.is_empty() {
		extracted.bvh = Some(BVH::new(bvh_items));
	}
	extracted.id_to_offsets = id_to_offsets;

	extracted.tree_buffer = Some(world_gpu_data.packed_64_tree_dynamic_buffer.get_buffer().clone());
	extracted.voxel_buffer = Some(world_gpu_data.packed_voxel_data_dynamic_buffer.get_buffer().clone());
}
