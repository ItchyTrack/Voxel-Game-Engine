use std::collections::HashMap;

use bevy::camera::Camera;
use bevy::ecs::resource::Resource;
use bevy::ecs::entity::Entity;
use bevy::ecs::system::{Query, Res, ResMut};
use bevy::math::Vec3;
use bevy::render::sync_world::RenderEntity;
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
use crate::incoming_ray_directions::IncomingRayDirections;
use crate::gpu_bvh::{BvhDataSource, BvhItemData};
use crate::{RayTileCapabilityRegistry, voxel_camera::VoxelCamera};
use bevy::render::renderer::WgpuWrapper;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

pub struct ExtractedVoxelScene {
	pub bvh: Option<BVH<Entity>>,
	pub bvh_item_data: HashMap<Entity, BvhItemData>,
	pub tree_buffer: GpuBuffer,
	pub voxel_buffer: GpuBuffer,
	pub main_tree_buffer: GpuBuffer,
	pub main_voxel_buffer: GpuBuffer,
}

#[derive(Resource, Default)]
pub struct ExtractedVoxelScenes(pub HashMap<Entity, ExtractedVoxelScene>);

struct RenderItem {
	entity: Entity,
	tree: AllocationId,
	voxels: AllocationId,
	generation: u64,
	voxel_type: VoxelTypeInfo,
	aabb: (Vec3, Vec3),
	dda_transform: Transform,
	priority: f32,
}

struct ResidencyCandidate<'a> {
	item: &'a RenderItem,
	directions: IncomingRayDirections,
}

pub fn extract_voxel_scene(
	mut extracted_scenes: ResMut<ExtractedVoxelScenes>,
	mut residency: ResMut<ResidencyBuffers>,
	direction_feedback: Query<&DirectionFeedback>,
	tile_capabilities: Extract<Res<RayTileCapabilityRegistry>>,
	cameras: Extract<Query<(RenderEntity, &VoxelCamera, &Camera, &GlobalTransform)>>,
	tiles: Extract<Query<(&DynamicTileData, &GlobalTransform)>>,
	world_gpu: Extract<Res<RayWorldGpuData>>,
) {
	extracted_scenes.0.clear();

	let mut views: Vec<(Entity, Vec<RenderItem>)> = Vec::new();
	for (render_entity, voxel_camera, camera, global_transform) in cameras.iter() {
		if !camera.is_active { continue; }
		let mut items = Vec::new();
		for entity in &voxel_camera.tiles_to_render {
			let Ok((tile_data, tile_global)) = tiles.get(*entity) else { continue };
			let Some(tile) = tile_capabilities.read(tile_data.data()) else { continue };
			let scale = (1u32 << tile.voxel_lod) as f32;
			let area_world = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			let placement = tile.placement;
			let aabb = aabb_of_transformed_aabb(&area_world, placement.bounds_min.as_vec3(), placement.bounds_max.as_vec3() + Vec3::ONE);
			let dda_transform = area_world * Transform::from_translation(placement.tree_root_pos.as_vec3());
			items.push(RenderItem {
				entity: *entity,
				tree: tile.tree,
				voxels: tile.voxels,
				generation: tile.generation,
				voxel_type: tile.voxel_type,
				aabb,
				dda_transform,
				priority: global_transform.translation().distance((aabb.0 + aabb.1) * 0.5) / 1000.0,
			});
		}
		items.sort_unstable_by_key(|item| ((item.priority * 100.0) as u64, item.entity.to_bits()));
		views.push((render_entity, items));
	}

	let tree_alignment = residency.tree_alignment();
	let voxel_alignment = residency.voxel_alignment();
	let limit = residency.binding_limit();

	let mut candidates = HashMap::<Entity, ResidencyCandidate<'_>>::new();
	for (view_entity, items) in &views {
		let feedback = direction_feedback.get(*view_entity).ok();
		for item in items {
			let directions = feedback
				.map(|feedback| feedback.directions_for(item.entity))
				.unwrap_or_default();
			candidates
				.entry(item.entity)
				.and_modify(|current| {
					let combined_directions = current.directions | directions;
					if item.priority < current.item.priority { current.item = item; }
					current.directions = combined_directions;
				})
				.or_insert(ResidencyCandidate { item, directions });
		}
	}
	let mut candidates: Vec<ResidencyCandidate<'_>> = candidates.into_values().collect();
	candidates.sort_unstable_by_key(|candidate| ((candidate.item.priority * 100.0) as u64, candidate.item.entity.to_bits()));

	let mut tree_total = 0u64;
	let mut voxel_total = 0u64;
	let mut resident: Vec<ResidentVoxels> = Vec::with_capacity(candidates.len());
	let mut dropped = 0usize;
	let world_gpu = world_gpu.lock();
	for candidate in candidates {
		if candidate.directions.is_empty() { continue; }
		let item = candidate.item;
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
	let tree_buffer = residency.tree_buffer().clone();
	let voxel_buffer = residency.voxel_buffer().clone();
	let main_tree_buffer = world_gpu.trees.buffer().clone();
	let main_voxel_buffer = world_gpu.voxels.buffer().clone();

	for (render_entity, items) in views {
		let mut bvh_items: Vec<(Entity, (Vec3, Vec3))> = Vec::with_capacity(items.len());
		let mut bvh_item_data = HashMap::with_capacity(items.len());
		for item in items {
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

		extracted_scenes.0.insert(render_entity, ExtractedVoxelScene {
			bvh: (!bvh_items.is_empty()).then(|| BVH::new(bvh_items)),
			bvh_item_data,
			tree_buffer: tree_buffer.clone(),
			voxel_buffer: voxel_buffer.clone(),
			main_tree_buffer: main_tree_buffer.clone(),
			main_voxel_buffer: main_voxel_buffer.clone(),
		});
	}
}
