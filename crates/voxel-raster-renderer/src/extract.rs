use std::collections::HashMap;

use bevy::camera::primitives::{Aabb, Frustum};
use bevy::camera::Camera;
use bevy::ecs::component::Component;
use bevy::ecs::entity::Entity;
use bevy::ecs::system::{Commands, Query, Res, ResMut};
use bevy::math::{U16Vec3, Vec3};
use bevy::render::Extract;
use bevy::render::renderer::WgpuWrapper;
use bevy::render::sync_world::RenderEntity;
use bevy::transform::components::{GlobalTransform, Transform};

use crate::gpu_data::RasterWorldGpuData;
use tile_data::DynamicTileData;
use voxel_gpu::AllocationId;

use crate::residency::{RasterResidencyBuffers, ResidentRasterVoxels};
use crate::{RasterTileCapabilityRegistry, voxel_camera::VoxelRasterCamera};

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

#[derive(Clone, Debug)]
pub struct ExtractedRasterItem {
	pub face_offset: u32,
	pub palette_offset: u32,
	pub face_count: u32,
	pub transform: Transform,
}

#[derive(Component)]
pub struct ExtractedRasterScene {
	pub items: Vec<ExtractedRasterItem>,
	pub face_buffer: GpuBuffer,
	pub palette_buffer: GpuBuffer,
}

struct RenderItem {
	entity: Entity,
	faces: AllocationId,
	palette: AllocationId,
	generation: u64,
	face_count: u32,
	transform: Transform,
	priority: f32,
}

const PRIORITY_BUCKET_SCALE: f32 = 16.0;

fn priority_bucket(priority: f32) -> usize {
	priority.max(0.0).ln_1p().mul_add(PRIORITY_BUCKET_SCALE, 0.0) as usize
}

fn voxel_gpu_bounds(min: U16Vec3, max: U16Vec3) -> (U16Vec3, U16Vec3) { (min, max) }

fn in_frustum(frustum: &Frustum, bounds: (U16Vec3, U16Vec3), transform: &Transform) -> bool {
	let aabb = Aabb::from_min_max(bounds.0.as_vec3(), bounds.1.as_vec3() + Vec3::ONE);
	frustum.intersects_obb(&aabb, &transform.compute_affine(), true, true)
}

pub fn extract_raster_scene(
	mut commands: Commands,
	existing_scenes: Query<Entity, bevy::ecs::query::With<ExtractedRasterScene>>,
	mut residency: ResMut<RasterResidencyBuffers>,
	tile_capabilities: Extract<Res<RasterTileCapabilityRegistry>>,
	cameras: Extract<Query<(RenderEntity, &VoxelRasterCamera, &Camera, &GlobalTransform, &Frustum)>>,
	tiles: Extract<Query<(&DynamicTileData, &GlobalTransform)>>,
	world_gpu: Extract<Res<RasterWorldGpuData>>,
) {
	for entity in &existing_scenes {
		commands.entity(entity).remove::<ExtractedRasterScene>();
	}

	let mut views: Vec<(Entity, Vec<RenderItem>)> = Vec::new();
	for (render_entity, raster_camera, camera, global_transform, frustum) in cameras.iter() {
		if !camera.is_active { continue; }
		let mut items = Vec::new();
		for entity in &raster_camera.tiles_to_render {
			let Ok((tile_data, tile_global)) = tiles.get(*entity) else { continue };
			let Some(tile) = tile_capabilities.read(tile_data.data()) else { continue };
			let scale = (1u32 << tile.voxel_lod) as f32;
			let transform = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			if !in_frustum(frustum, voxel_gpu_bounds(tile.bounds_min, tile.bounds_max), &transform) { continue; }
			items.push(RenderItem {
				entity: *entity,
				faces: tile.faces,
				palette: tile.palette,
				generation: tile.generation,
				face_count: tile.face_count,
				transform,
				priority: global_transform.translation().distance(transform.translation),
			});
		}
		items.sort_unstable_by_key(|item| (priority_bucket(item.priority), item.entity.to_bits()));
		views.push((render_entity, items));
	}

	let world_gpu = world_gpu.lock();
	let limit = residency.binding_limit();
	let face_alignment = residency.face_alignment();
	let palette_alignment = residency.palette_alignment();

	let mut candidates = HashMap::<Entity, &RenderItem>::new();
	for (_, items) in &views {
		for item in items {
			candidates
				.entry(item.entity)
				.and_modify(|current| {
					if item.priority < current.priority { *current = item; }
				})
				.or_insert(item);
		}
	}
	let mut candidates: Vec<&RenderItem> = candidates.into_values().collect();
	candidates.sort_unstable_by_key(|item| (priority_bucket(item.priority), item.entity.to_bits()));

	let mut total_faces = 0u64;
	let mut total_palettes = 0u64;
	let mut resident: Vec<ResidentRasterVoxels> = Vec::with_capacity(candidates.len());
	for item in candidates {
		let Some(face_held) = world_gpu.faces.held_buffer(item.faces) else { continue; };
		let Some(palette_held) = world_gpu.palettes.held_buffer(item.palette) else { continue; };
		let next_faces = total_faces + face_held.size().next_multiple_of(face_alignment) as u64;
		let next_palettes = total_palettes + palette_held.size().next_multiple_of(palette_alignment) as u64;
		if next_faces > limit || next_palettes > limit { continue; }
		total_faces = next_faces;
		total_palettes = next_palettes;
		resident.push(ResidentRasterVoxels {
			entity: item.entity,
			faces: item.faces,
			palette: item.palette,
			generation: item.generation,
		});
	}

	resident.sort_unstable_by_key(|item| item.entity.to_bits());
	residency.upload(&world_gpu, &resident);
	let face_offsets = residency.face_offsets();
	let palette_offsets = residency.palette_offsets();
	let face_stride = std::mem::size_of::<crate::gpu_raster_mesh::MeshFace>() as u32;
	let palette_stride = std::mem::size_of::<u32>() as u32;
	let face_buffer = residency.face_buffer().clone();
	let palette_buffer = residency.palette_buffer().clone();

	for (render_entity, items) in views {
		let mut extracted_items = Vec::with_capacity(items.len());
		for item in items {
			let Some(face_offset_bytes) = face_offsets.get(&item.entity).copied() else { continue; };
			let Some(palette_offset_bytes) = palette_offsets.get(&item.entity).copied() else { continue; };
			extracted_items.push(ExtractedRasterItem {
				face_offset: face_offset_bytes / face_stride,
				palette_offset: palette_offset_bytes / palette_stride,
				face_count: item.face_count,
				transform: item.transform,
			});
		}

		commands.entity(render_entity).insert(ExtractedRasterScene {
			items: extracted_items,
			face_buffer: face_buffer.clone(),
			palette_buffer: palette_buffer.clone(),
		});
	}
}
