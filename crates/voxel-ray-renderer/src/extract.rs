use std::collections::HashMap;

use bevy::camera::{Camera, Projection};
use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, Res, ResMut};
use bevy::math::Vec3;
use bevy::render::Extract;
use bevy::transform::components::{GlobalTransform, Transform};

use crate::residency::{ResidentVoxels, ResidencyBuffers, ResidencyDirections};
use crate::gpu_data::RayWorldGpuData;
use voxel_data::voxels::VoxelTypeInfo;
use voxel_data::bvh::BVH;
use voxel_data::aabb::aabb_of_transformed_aabb;
use tile_data::DynamicTileData;
use voxel_gpu::AllocationId;

use crate::direction_feedback::DirectionFeedback;
use crate::gpu_bvh::{BvhDataSource, BvhItemData};
use crate::{RayTileCapabilityRegistry, voxel_camera::VoxelCamera};
use bevy::render::renderer::WgpuWrapper;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

#[derive(Resource, Default)]
pub struct ExtractedVoxelScene {
	pub camera_transform: Transform,
	pub camera_projection: Option<Projection>,
	pub bvh: Option<BVH<Entity>>,
	pub bvh_item_data: HashMap<Entity, BvhItemData>,
	pub has_camera: bool,
	pub tree_buffer: Option<GpuBuffer>,
	pub voxel_buffer: Option<GpuBuffer>,
	pub main_tree_buffer: Option<GpuBuffer>,
	pub main_voxel_buffer: Option<GpuBuffer>,
}

struct RenderItem {
	entity: Entity,
	tree: AllocationId,
	voxels: AllocationId,
	generation: u64,
	voxel_type: VoxelTypeInfo,
	aabb: (Vec3, Vec3),
	dda_transform: Transform,
}

const PRIORITY_BUCKET_SCALE: f32 = 16.0;

fn priority_bucket(priority: f32) -> usize {
	priority.max(0.0).ln_1p().mul_add(PRIORITY_BUCKET_SCALE, 0.0) as usize
}

fn bucket_push(buckets: &mut Vec<Vec<RenderItem>>, priority: f32, item: RenderItem) {
	let bucket = priority_bucket(priority);
	if bucket >= buckets.len() {
		buckets.resize_with(bucket + 1, Vec::new);
	}
	buckets[bucket].push(item);
}

pub fn extract_voxel_scene(
	mut extracted: ResMut<ExtractedVoxelScene>,
	mut residency: ResMut<ResidencyBuffers>,
	direction_feedback: Res<DirectionFeedback>,
	tile_capabilities: Extract<Res<RayTileCapabilityRegistry>>,
	cameras: Extract<Query<(&VoxelCamera, &Camera, &Projection, &GlobalTransform)>>,
	tiles: Extract<Query<(&DynamicTileData, &GlobalTransform)>>,
	world_gpu: Extract<Res<RayWorldGpuData>>,
) {
	extracted.has_camera = false;
	extracted.bvh = None;
	extracted.bvh_item_data.clear();

	let mut buckets: Vec<Vec<RenderItem>> = Vec::new();

	for (voxel_camera, camera, projection, global_transform) in cameras.iter() {
		if !camera.is_active { continue; }
		extracted.camera_transform = global_transform.compute_transform();
		extracted.camera_projection = Some(projection.clone());
		extracted.has_camera = true;

		for entity in &voxel_camera.tiles_to_render {
			let Ok((tile_data, tile_global)) = tiles.get(*entity) else { continue };
			let Some(tile) = tile_capabilities.read(tile_data.data()) else { continue };
			let scale = (1u32 << tile.voxel_lod) as f32;
			let area_world = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			let placement = tile.placement;
			let aabb = aabb_of_transformed_aabb(&area_world, placement.bounds_min.as_vec3(), placement.bounds_max.as_vec3() + Vec3::ONE);
			let dda_transform = area_world * Transform::from_translation(placement.tree_root_pos.as_vec3());
			bucket_push(
				&mut buckets,
				global_transform.translation().distance((aabb.0 + aabb.1) * 0.5) / 1000.0,
				RenderItem {
					entity: *entity,
					tree: tile.tree,
					voxels: tile.voxels,
					generation: tile.generation,
					voxel_type: tile.voxel_type,
					aabb,
					dda_transform,
				},
			);
		}
		break;
	}
	if !extracted.has_camera { return; }
	let world_gpu = world_gpu.lock();

	let tree_alignment = residency.tree_alignment();
	let voxel_alignment = residency.voxel_alignment();

	let items: Vec<RenderItem> = buckets.into_iter().flatten().collect();

	let limit = residency.binding_limit();
	let mut tree_total = 0u64;
	let mut voxel_total = 0u64;
	let mut resident: Vec<ResidentVoxels> = Vec::with_capacity(items.len());
	let mut dropped = 0usize;
	for item in &items {
		if !direction_feedback
			.0
			.get(&item.entity)
			.is_some_and(|directions| !directions.is_empty())
		{
			continue;
		}
		let Some(tree_held) = world_gpu.trees.held_buffer(item.tree) else { continue; };
		let Some(voxel_held) = world_gpu.voxels.held_buffer(item.voxels) else { continue; };
		let next_tree = tree_total + tree_held.size().next_multiple_of(tree_alignment) as u64;
		let next_voxel = voxel_total + voxel_held.size().next_multiple_of(voxel_alignment) as u64;
		if next_tree > limit || next_voxel > limit {
			dropped += 1;
			continue;
		}
		tree_total = next_tree;
		voxel_total = next_voxel;
		resident.push(ResidentVoxels {
			entity: item.entity,
			tree: item.tree,
			voxels: item.voxels,
			generation: item.generation,
			loaded_directions: ResidencyDirections::Unculled,
		});
	}

	if dropped > 0 {
		log::warn!(
			"residency budget hit: {dropped} feedback-selected sub-grids dropped ({} resident)",
			resident.len()
		);
	}

	resident.sort_unstable_by_key(|item| item.entity.to_bits());
	residency.upload(&world_gpu, &resident);

	let offsets = residency.offsets();
	let mut bvh_items: Vec<(Entity, (Vec3, Vec3))> = Vec::with_capacity(items.len());
	let mut bvh_item_data = HashMap::with_capacity(items.len());
	for item in &items {
		bvh_items.push((item.entity, item.aabb));
		let data_source = if let Some(&(tree_offset, voxel_offset)) = offsets.get(&item.entity) {
			BvhDataSource::Residency { tree_offset, voxel_offset }
		} else {
			match (
				world_gpu.trees.held_buffer(item.tree),
				world_gpu.voxels.held_buffer(item.voxels),
			) {
				(Some(tree_held), Some(voxel_held)) => BvhDataSource::MainBuffer {
					tree_offset: tree_held.offset(),
					voxel_offset: voxel_held.offset(),
				},
				_ => BvhDataSource::FeedbackOnly,
			}
		};
		bvh_item_data.insert(item.entity, BvhItemData {
			data_source,
			transform: item.dda_transform,
			voxel_type: item.voxel_type,
		});
	}

	if !bvh_items.is_empty() {
		extracted.bvh = Some(BVH::new(bvh_items));
	}
	extracted.bvh_item_data = bvh_item_data;

	extracted.tree_buffer = Some(residency.tree_buffer().clone());
	extracted.voxel_buffer = Some(residency.voxel_buffer().clone());
	extracted.main_tree_buffer = Some(world_gpu.trees.buffer().clone());
	extracted.main_voxel_buffer = Some(world_gpu.voxels.buffer().clone());
}
