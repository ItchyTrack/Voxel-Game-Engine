use std::ops::Range;

use bevy::camera::Camera;
use bevy::camera::primitives::{Aabb, Frustum};
use bevy::ecs::component::Component;
use bevy::ecs::entity::Entity;
use bevy::ecs::system::{Commands, Query, Res};
use bevy::math::{UVec3, Vec3};
use bevy::render::Extract;
use bevy::render::sync_world::RenderEntity;
use bevy::transform::components::{GlobalTransform, Transform};
use tile_data::DynamicTileData;
use voxel_data::voxels::VoxelTypeId;
use voxel_gpu::packed_buffer_group::{PackedBufferGroupAllocation, PackedBufferGroupBuffer};

use crate::gpu_data::RasterWorldGpuData;
use crate::{RasterTileCapabilityRegistry, voxel_camera::VoxelRasterCamera};

#[derive(Clone, Debug)]
pub struct ExtractedRasterItem {
	pub face_offset: u32,
	pub voxel_data_offset: u32,
	pub face_count: u32,
	pub voxel_type: VoxelTypeId,
	pub transform: Transform,
}

#[derive(Clone, Debug)]
pub struct ExtractedRasterBatch {
	pub item_range: Range<usize>,
	pub face_buffer: PackedBufferGroupBuffer,
	pub voxel_data_buffer: PackedBufferGroupBuffer,
}

#[derive(Component)]
pub struct ExtractedRasterScene {
	pub items: Vec<ExtractedRasterItem>,
	pub batches: Vec<ExtractedRasterBatch>,
	/// Keeps packed ranges alive until this extracted frame finishes rendering.
	_allocation_leases: Vec<(PackedBufferGroupAllocation, PackedBufferGroupAllocation)>,
}

struct RenderItem {
	entity: Entity,
	faces: PackedBufferGroupAllocation,
	voxel_data: PackedBufferGroupAllocation,
	face_count: u32,
	voxel_type: VoxelTypeId,
	transform: Transform,
}

fn in_frustum(frustum: &Frustum, min: UVec3, max: UVec3, transform: &Transform) -> bool {
	let aabb = Aabb::from_min_max(min.as_vec3(), max.as_vec3() + Vec3::ONE);
	frustum.intersects_obb(&aabb, &transform.compute_affine(), true, true)
}

pub fn extract_raster_scene(
	mut commands: Commands,
	existing_scenes: Query<Entity, bevy::ecs::query::With<ExtractedRasterScene>>,
	tile_capabilities: Extract<Res<RasterTileCapabilityRegistry>>,
	cameras: Extract<Query<(RenderEntity, &VoxelRasterCamera, &Camera, &Frustum)>>,
	tiles: Extract<Query<(&DynamicTileData, &GlobalTransform)>>,
	world_gpu: Extract<Res<RasterWorldGpuData>>,
) {
	for entity in &existing_scenes {
		commands.entity(entity).remove::<ExtractedRasterScene>();
	}

	let world_gpu = world_gpu.lock();
	let face_stride = std::mem::size_of::<crate::gpu_raster_mesh::MeshFace>() as u32;
	for (render_entity, raster_camera, camera, frustum) in &cameras {
		if !camera.is_active { continue; }
		let mut candidates = Vec::new();
		for entity in &raster_camera.tiles_to_render {
			let Ok((tile_data, tile_global)) = tiles.get(*entity) else { continue };
			let Some(tile) = tile_capabilities.read(tile_data.data()) else { continue };
			let scale = (1u32 << tile.voxel_lod) as f32;
			let transform = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			if !in_frustum(frustum, tile.bounds_min, tile.bounds_max, &transform) { continue; }
			candidates.push(RenderItem {
				entity: *entity,
				faces: tile.faces,
				voxel_data: tile.voxel_data,
				face_count: tile.face_count,
				voxel_type: tile.voxel_type,
				transform,
			});
		}

		candidates.sort_unstable_by_key(|item| (item.faces.id().buffer_index(), item.voxel_data.id().buffer_index(), item.entity.to_bits()));
		let mut items = Vec::with_capacity(candidates.len());
		let mut batches: Vec<ExtractedRasterBatch> = Vec::new();
		let mut allocation_leases = Vec::with_capacity(candidates.len());
		let mut batch_key = None;
		for candidate in candidates {
			let Some(faces) = world_gpu.faces.slice(candidate.faces.id()) else { continue };
			let Some(voxel_data) = world_gpu.voxel_data.slice(candidate.voxel_data.id()) else { continue };
			let key = (faces.buffer_index, voxel_data.buffer_index);
			if batch_key != Some(key) {
				if let Some(batch) = batches.last_mut() {
					batch.item_range.end = items.len();
				}
				batches.push(ExtractedRasterBatch {
					item_range: items.len()..items.len(),
					face_buffer: faces.buffer.clone(),
					voxel_data_buffer: voxel_data.buffer.clone(),
				});
				batch_key = Some(key);
			}
			items.push(ExtractedRasterItem {
				face_offset: faces.offset / face_stride,
				voxel_data_offset: voxel_data.offset,
				face_count: candidate.face_count,
				voxel_type: candidate.voxel_type,
				transform: candidate.transform,
			});
			allocation_leases.push((candidate.faces, candidate.voxel_data));
		}
		if let Some(batch) = batches.last_mut() {
			batch.item_range.end = items.len();
		}
		commands.entity(render_entity).insert(ExtractedRasterScene { items, batches, _allocation_leases: allocation_leases });
	}
}
