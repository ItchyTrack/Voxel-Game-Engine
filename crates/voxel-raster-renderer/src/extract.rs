use bevy::camera::primitives::{Aabb, Frustum};
use bevy::camera::{Camera, Projection};
use bevy::ecs::entity::Entity;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{Query, Res, ResMut};
use bevy::math::Vec3;
use bevy::render::Extract;
use bevy::render::renderer::WgpuWrapper;
use bevy::transform::components::{GlobalTransform, Transform};

use voxel_data::subgrid::SubGrid;
use voxel_gpu::lod_voxels::LodVoxels;
use voxel_gpu::world_gpu_data::WorldGpuData;
use voxel_gpu::VoxelGpuState;

use crate::residency::{RasterResidencyBuffers, ResidentRasterVoxels};
use crate::voxel_camera::VoxelRasterCamera;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

#[derive(Clone, Debug)]
pub struct ExtractedRasterItem {
	pub face_offset: u32,
	pub palette_offset: u32,
	pub face_count: u32,
	pub transform: Transform,
}

#[derive(Resource, Default)]
pub struct ExtractedRasterScene {
	pub camera_transform: Transform,
	pub camera_projection: Option<Projection>,
	pub has_camera: bool,
	pub items: Vec<ExtractedRasterItem>,
	pub face_buffer: Option<GpuBuffer>,
	pub palette_buffer: Option<GpuBuffer>,
}

struct RenderItem {
	entity: Entity,
	buffer_id: u32,
	palette_id: u32,
	generation: u64,
	face_count: u32,
	transform: Transform,
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

fn in_frustum(frustum: &Frustum, bounds: voxel_gpu::voxel_gpu_state::VoxelGpuBounds, transform: &Transform) -> bool {
	let aabb = Aabb::from_min_max(bounds.min.as_vec3(), bounds.max.as_vec3() + Vec3::ONE);
	frustum.intersects_obb(&aabb, &transform.compute_affine(), true, true)
}

pub fn extract_raster_scene(
	mut extracted: ResMut<ExtractedRasterScene>,
	mut residency: ResMut<RasterResidencyBuffers>,
	cameras: Extract<Query<(&VoxelRasterCamera, &Camera, &Projection, &GlobalTransform, &Frustum)>>,
	sub_grids: Extract<Query<(&SubGrid, &VoxelGpuState)>>,
	lod_voxels: Extract<Query<(&LodVoxels, &VoxelGpuState, &GlobalTransform)>>,
	grid_transforms: Extract<Query<&GlobalTransform>>,
	world_gpu: Extract<Res<WorldGpuData>>,
) {
	extracted.has_camera = false;
	extracted.items.clear();
	extracted.face_buffer = None;
	extracted.palette_buffer = None;

	let mut buckets: Vec<Vec<RenderItem>> = Vec::new();
	for (raster_camera, camera, projection, global_transform, frustum) in cameras.iter() {
		if !camera.is_active { continue; }
		extracted.camera_transform = global_transform.compute_transform();
		extracted.camera_projection = Some(projection.clone());
		extracted.has_camera = true;

		for entity in &raster_camera.subgrids_to_render {
			let Ok((sub_grid, gpu_state)) = sub_grids.get(*entity) else { continue; };
			let Some(raster_state) = gpu_state.raster else { continue; };
			let Some(bounds) = gpu_state.bounds() else { continue; };
			let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue; };
			let transform = grid_global.compute_transform() * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
			if !in_frustum(frustum, bounds, &transform) { continue; }
			let center = transform.translation;
			bucket_push(
				&mut buckets,
				global_transform.translation().distance(center),
				RenderItem {
					entity: *entity,
					buffer_id: raster_state.buffer_id(),
					palette_id: raster_state.palette_id(),
					generation: raster_state.generation(),
					face_count: raster_state.face_count(),
					transform,
				},
			);
		}

		for entity in &raster_camera.lods_to_render {
			let Ok((lod_grid, gpu_state, lod_global)) = lod_voxels.get(*entity) else { continue; };
			let Some(raster_state) = gpu_state.raster else { continue; };
			let Some(bounds) = gpu_state.bounds() else { continue; };
			let scale = (1u32 << lod_grid.lod.max(0.0).floor() as u32) as f32;
			let transform = lod_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			if !in_frustum(frustum, bounds, &transform) { continue; }
			bucket_push(
				&mut buckets,
				global_transform.translation().distance(transform.translation),
				RenderItem {
					entity: *entity,
					buffer_id: raster_state.buffer_id(),
					palette_id: raster_state.palette_id(),
					generation: raster_state.generation(),
					face_count: raster_state.face_count(),
					transform,
				},
			);
		}
		break;
	}
	if !extracted.has_camera { return; }

	let items_len: usize = buckets.iter().map(Vec::len).sum();
	let mut items = Vec::with_capacity(items_len);
	let limit = residency.binding_limit();
	let face_alignment = residency.face_alignment();
	let palette_alignment = residency.palette_alignment();
	let mut total_faces = 0u64;
	let mut total_palettes = 0u64;
	let mut resident: Vec<ResidentRasterVoxels> = Vec::with_capacity(items_len);
	for bucket in &buckets {
		for item in bucket {
			let Some(face_held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(item.buffer_id) else { continue; };
			let Some(palette_held) = world_gpu.packed_raster_palette_dynamic_buffer.held_buffer(item.palette_id) else { continue; };
			let next_faces = total_faces + face_held.size().next_multiple_of(face_alignment) as u64;
			let next_palettes = total_palettes + palette_held.size().next_multiple_of(palette_alignment) as u64;
			if next_faces > limit || next_palettes > limit {
				continue;
			}
			total_faces = next_faces;
			total_palettes = next_palettes;
			resident.push(ResidentRasterVoxels {
				entity: item.entity,
				buffer_id: item.buffer_id,
				palette_id: item.palette_id,
				generation: item.generation,
			});
			items.push(RenderItem {
				entity: item.entity,
				buffer_id: item.buffer_id,
				palette_id: item.palette_id,
				generation: item.generation,
				face_count: item.face_count,
				transform: item.transform,
			});
		}
	}

	resident.sort_unstable_by_key(|item| item.entity.to_bits());
	residency.upload(&world_gpu, &resident);
	let face_offsets = residency.face_offsets();
	let palette_offsets = residency.palette_offsets();
	let face_stride = std::mem::size_of::<voxel_gpu::gpu_raster_mesh::MeshFace>() as u32;
	let palette_stride = std::mem::size_of::<u32>() as u32;
	for item in items {
		let Some(face_offset_bytes) = face_offsets.get(&item.entity).copied() else { continue; };
		let Some(palette_offset_bytes) = palette_offsets.get(&item.entity).copied() else { continue; };
		extracted.items.push(ExtractedRasterItem {
			face_offset: face_offset_bytes / face_stride,
			palette_offset: palette_offset_bytes / palette_stride,
			face_count: item.face_count,
			transform: item.transform,
		});
	}

	extracted.face_buffer = Some(residency.face_buffer().clone());
	extracted.palette_buffer = Some(residency.palette_buffer().clone());
}
