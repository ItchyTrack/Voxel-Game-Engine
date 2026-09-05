use std::ops::Range;

use bevy::camera::{Camera, primitives::{Aabb, Frustum}};
use bevy::prelude::*;
use bevy::render::{Extract, sync_world::RenderEntity};
use tile_data::DynamicTileData;
use voxel_gpu::packed_buffer_group::{PackedBufferGroupAllocation, PackedBufferGroupBuffer};

use crate::{
	MarchingTileCapabilityRegistry,
	gpu_data::MarchingWorldGpuData,
	voxel_camera::VoxelMarchingCamera,
};

#[derive(Clone, Debug)]
pub struct ExtractedMarchingItem {
	pub first_vertex: u32,
	pub vertex_count: u32,
	pub transform: Transform,
}

#[derive(Clone, Debug)]
pub struct ExtractedMarchingBatch {
	pub item_range: Range<usize>,
	pub vertex_buffer: PackedBufferGroupBuffer,
}

#[derive(Component)]
pub struct ExtractedMarchingScene {
	pub items: Vec<ExtractedMarchingItem>,
	pub batches: Vec<ExtractedMarchingBatch>,
	/// Keeps packed ranges alive until this extracted frame finishes rendering.
	_allocation_leases: Vec<PackedBufferGroupAllocation>,
}

struct RenderItem {
	entity: Entity,
	vertices: PackedBufferGroupAllocation,
	vertex_count: u32,
	transform: Transform,
}

pub fn extract_marching_scene(
	mut commands: Commands,
	existing: Query<Entity, With<ExtractedMarchingScene>>,
	capabilities: Extract<Res<MarchingTileCapabilityRegistry>>,
	cameras: Extract<Query<(RenderEntity, &VoxelMarchingCamera, &Camera, &Frustum)>>,
	tiles: Extract<Query<(&DynamicTileData, &GlobalTransform)>>,
	world_gpu: Extract<Res<MarchingWorldGpuData>>,
) {
	for entity in &existing { commands.entity(entity).remove::<ExtractedMarchingScene>(); }
	let world_gpu = world_gpu.lock();
	let triangle_stride = std::mem::size_of::<crate::marching_cubes::MarchingTriangle>() as u32;
	for (render_entity, marching_camera, camera, frustum) in &cameras {
		if !camera.is_active { continue; }
		let mut candidates = Vec::new();
		for entity in &marching_camera.tiles_to_render {
			let Ok((data, tile_global)) = tiles.get(*entity) else { continue };
			let Some(tile) = capabilities.read(data.data()) else { continue };
			let scale = (1u32 << tile.voxel_lod) as f32;
			let transform = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			let aabb = Aabb::from_min_max(tile.bounds_min, tile.bounds_max);
			if !frustum.intersects_obb(&aabb, &transform.compute_affine(), true, true) { continue; }
			candidates.push(RenderItem {
				entity: *entity,
				vertices: tile.vertices,
				vertex_count: tile.vertex_count,
				transform,
			});
		}

		candidates.sort_unstable_by_key(|item| (item.vertices.id().buffer_index(), item.entity.to_bits()));
		let mut items = Vec::with_capacity(candidates.len());
		let mut batches: Vec<ExtractedMarchingBatch> = Vec::new();
		let mut allocation_leases = Vec::with_capacity(candidates.len());
		let mut batch_buffer_index = None;
		for candidate in candidates {
			let Some(vertices) = world_gpu.slice(candidate.vertices.id()) else { continue };
			if batch_buffer_index != Some(vertices.buffer_index) {
				if let Some(batch) = batches.last_mut() {
					batch.item_range.end = items.len();
				}
				batches.push(ExtractedMarchingBatch {
					item_range: items.len()..items.len(),
					vertex_buffer: vertices.buffer,
				});
				batch_buffer_index = Some(vertices.buffer_index);
			}
			items.push(ExtractedMarchingItem {
				first_vertex: vertices.offset / triangle_stride * 3,
				vertex_count: candidate.vertex_count,
				transform: candidate.transform,
			});
			allocation_leases.push(candidate.vertices);
		}
		if let Some(batch) = batches.last_mut() {
			batch.item_range.end = items.len();
		}
		commands.entity(render_entity).insert(ExtractedMarchingScene { items, batches, _allocation_leases: allocation_leases });
	}
}
