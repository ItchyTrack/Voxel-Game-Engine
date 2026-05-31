use std::collections::HashMap;

use bevy::camera::{Camera, Projection};
use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, Res, ResMut};
use bevy::math::Vec3;
use bevy::render::Extract;
use bevy::transform::components::{GlobalTransform, Transform};

use gpu_voxel_data::sub_grid_gpu_state::SubGridGpuState;
use gpu_voxel_data::world_gpu_data::WorldGpuData;
use voxel_bvh::bvh::BVH;
use voxel_data::grid::SubGrid;

#[derive(Resource, Default)]
pub struct ExtractedVoxelScene {
	pub camera_transform: Transform,
	pub camera_projection: Option<Projection>,
	pub bvh: Option<BVH<Entity>>,
	pub id_to_offsets: HashMap<Entity, (u32, u32, Transform)>,
	pub has_camera: bool,
	pub tree_buffer: Option<wgpu::Buffer>,
	pub voxel_buffer: Option<wgpu::Buffer>,
}

pub fn extract_voxel_scene(
	mut extracted: ResMut<ExtractedVoxelScene>,
	cameras: Extract<Query<(&Camera, &Projection, &GlobalTransform)>>,
	sub_grids: Extract<Query<(Entity, &SubGrid, &SubGridGpuState)>>,
	grid_transforms: Extract<Query<&GlobalTransform>>,
	world_gpu_data: Extract<Res<WorldGpuData>>,
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

	let mut bvh_items: Vec<(Entity, (Vec3, Vec3))> = Vec::new();
	let mut id_to_offsets: HashMap<Entity, (u32, u32, Transform)> = HashMap::new();

	for (entity, sub_grid, gpu_state) in sub_grids.iter() {
		let Some(tree_held) = world_gpu_data
			.packed_64_tree_dynamic_buffer
			.get_held_buffer(gpu_state.tree_id()) else { continue };
		let Some(voxel_held) = world_gpu_data
			.packed_voxel_data_dynamic_buffer
			.get_held_buffer(gpu_state.voxels_id()) else { continue };

		let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue };
		let sub_world = grid_global.compute_transform()
			* Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
		let Some(aabb) = sub_grid.aabb(&sub_world) else { continue };

		let (_, tree_root_pos, _) = sub_grid.get_voxels().get_voxels().get_internals();
		let dda_transform = sub_world * Transform::from_translation(tree_root_pos.as_vec3());

		bvh_items.push((entity, aabb));
		id_to_offsets.insert(entity, (tree_held.offset(), voxel_held.offset(), dda_transform));
	}

	if !bvh_items.is_empty() {
		extracted.bvh = Some(BVH::new(bvh_items));
	}
	extracted.id_to_offsets = id_to_offsets;

	extracted.tree_buffer = Some(world_gpu_data.packed_64_tree_dynamic_buffer.get_buffer().clone());
	extracted.voxel_buffer = Some(world_gpu_data.packed_voxel_data_dynamic_buffer.get_buffer().clone());
}
