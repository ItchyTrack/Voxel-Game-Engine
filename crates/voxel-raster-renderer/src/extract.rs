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

use crate::residency::RasterResidencyBuffers;
use crate::voxel_camera::VoxelRasterCamera;

type GpuBuffer = WgpuWrapper<wgpu::Buffer>;

#[derive(Clone, Debug)]
pub struct ExtractedRasterItem {
	pub face_offset: u32,
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
}

struct RenderItem {
	entity: Entity,
	buffer_id: u32,
	face_count: u32,
	transform: Transform,
	priority: f32,
}

pub fn extract_raster_scene(
	mut extracted: ResMut<ExtractedRasterScene>,
	mut residency: ResMut<RasterResidencyBuffers>,
	cameras: Extract<Query<(&VoxelRasterCamera, &Camera, &Projection, &GlobalTransform)>>,
	sub_grids: Extract<Query<(&SubGrid, &VoxelGpuState)>>,
	lod_voxels: Extract<Query<(&LodVoxels, &VoxelGpuState, &GlobalTransform)>>,
	grid_transforms: Extract<Query<&GlobalTransform>>,
	world_gpu: Extract<Res<WorldGpuData>>,
) {
	extracted.has_camera = false;
	extracted.items.clear();
	extracted.face_buffer = None;

	let mut items = Vec::new();
	for (raster_camera, camera, projection, global_transform) in cameras.iter() {
		if !camera.is_active { continue; }
		extracted.camera_transform = global_transform.compute_transform();
		extracted.camera_projection = Some(projection.clone());
		extracted.has_camera = true;

		for entity in &raster_camera.subgrids_to_render {
			let Ok((sub_grid, gpu_state)) = sub_grids.get(*entity) else { continue; };
			let Some(raster_state) = gpu_state.raster else { continue; };
			let Ok(grid_global) = grid_transforms.get(sub_grid.grid()) else { continue; };
			let transform = grid_global.compute_transform() * Transform::from_translation(sub_grid.sub_grid_pos().as_vec3());
			let center = transform.translation;
			items.push(RenderItem {
				entity: *entity,
				buffer_id: raster_state.buffer_id(),
				face_count: raster_state.face_count(),
				transform,
				priority: -global_transform.translation().distance(center),
			});
		}

		for entity in &raster_camera.lods_to_render {
			let Ok((lod_grid, gpu_state, lod_global)) = lod_voxels.get(*entity) else { continue; };
			let Some(raster_state) = gpu_state.raster else { continue; };
			let scale = (1u32 << lod_grid.lod.max(0.0).floor() as u32) as f32;
			let transform = lod_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			items.push(RenderItem {
				entity: *entity,
				buffer_id: raster_state.buffer_id(),
				face_count: raster_state.face_count(),
				transform,
				priority: -global_transform.translation().distance(transform.translation),
			});
		}
		break;
	}
	if !extracted.has_camera { return; }

	items.sort_by(|a, b| b.priority.total_cmp(&a.priority));
	let limit = residency.binding_limit();
	let alignment = residency.alignment();
	let mut total = 0u64;
	let mut resident = Vec::with_capacity(items.len());
	for item in &items {
		let Some(held) = world_gpu.packed_raster_face_dynamic_buffer.held_buffer(item.buffer_id) else { continue; };
		let next = total + held.size().next_multiple_of(alignment) as u64;
		if next > limit {
			continue;
		}
		total = next;
		resident.push((item.entity, item.buffer_id));
	}

	residency.upload(&world_gpu, &resident);
	let offsets = residency.offsets();
	let face_stride = std::mem::size_of::<voxel_gpu::gpu_raster_mesh::MeshFace>() as u32;
	for item in items {
		let Some(offset_bytes) = offsets.get(&item.entity).copied() else { continue; };
		extracted.items.push(ExtractedRasterItem {
			face_offset: offset_bytes / face_stride,
			face_count: item.face_count,
			transform: item.transform,
		});
	}

	extracted.face_buffer = Some(residency.face_buffer().clone());
}
