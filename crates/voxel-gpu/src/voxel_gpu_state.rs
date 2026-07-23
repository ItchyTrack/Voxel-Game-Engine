use bevy::ecs::lifecycle::HookContext;
use bevy::ecs::world::DeferredWorld;
use bevy::math::U16Vec3;
use bevy::prelude::*;
use voxel_data::voxels::VoxelTypeInfo;

use crate::world_gpu_data::WorldGpuData;

// Snapshotted when the GPU tree is built, so the renderer positions the data
// from the same snapshot it was encoded from, not the live sub-grid.
#[derive(Clone, Copy, Debug)]
pub struct SubGridPlacement {
	pub tree_root_pos: U16Vec3,
	pub bounds_min: U16Vec3,
	pub bounds_max: U16Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct VoxelGpuBounds {
	pub min: U16Vec3,
	pub max: U16Vec3,
}

impl VoxelGpuBounds {
	pub fn from_placement(placement: SubGridPlacement) -> Self {
		Self {
			min: placement.bounds_min,
			max: placement.bounds_max,
		}
	}
}

#[derive(Clone, Copy, Debug)]
pub struct RayGpuState {
	tree_id: u32,
	voxels_id: u32,
	generation: u64,
	placement: SubGridPlacement,
	voxel_type: VoxelTypeInfo,
}

impl RayGpuState {
	pub(crate) fn new(tree_id: u32, voxels_id: u32, generation: u64, placement: SubGridPlacement, voxel_type: VoxelTypeInfo) -> Self {
		Self {
			tree_id,
			voxels_id,
			generation,
			placement,
			voxel_type,
		}
	}

	pub fn tree_id(&self) -> u32 { self.tree_id }
	pub fn voxels_id(&self) -> u32 { self.voxels_id }
	pub fn generation(&self) -> u64 { self.generation }
	pub fn placement(&self) -> SubGridPlacement { self.placement }
	pub fn voxel_type(&self) -> VoxelTypeInfo { self.voxel_type }
}

#[derive(Clone, Copy, Debug)]
pub struct RasterGpuState {
	buffer_id: u32,
	palette_id: u32,
	face_count: u32,
	generation: u64,
}

impl RasterGpuState {
	pub(crate) fn new(buffer_id: u32, palette_id: u32, face_count: u32, generation: u64) -> Self {
		Self {
			buffer_id,
			palette_id,
			face_count,
			generation,
		}
	}

	pub fn buffer_id(&self) -> u32 { self.buffer_id }
	pub fn palette_id(&self) -> u32 { self.palette_id }
	pub fn face_count(&self) -> u32 { self.face_count }
	pub fn generation(&self) -> u64 { self.generation }
}

#[derive(Component, Clone, Debug, Default)]
#[component(on_remove = free_voxel_gpu_buffers)]
pub struct VoxelGpuState {
	pub ray: Option<RayGpuState>,
	pub raster: Option<RasterGpuState>,
	bounds: Option<VoxelGpuBounds>,
}

impl VoxelGpuState {
	pub fn bounds(&self) -> Option<VoxelGpuBounds> { self.bounds }
	pub fn is_empty(&self) -> bool { self.ray.is_none() && self.raster.is_none() }

	pub fn matches(&self, format: VoxelGpuFormat) -> bool {
		match format {
			VoxelGpuFormat::Volume => self.ray.is_some(),
			VoxelGpuFormat::Surface => self.raster.is_some(),
		}
	}

	pub(crate) fn set_ray(&mut self, state: RayGpuState) {
		self.bounds = Some(VoxelGpuBounds::from_placement(state.placement()));
		self.ray = Some(state);
	}

	pub(crate) fn set_raster(&mut self, state: RasterGpuState, bounds: VoxelGpuBounds) {
		self.bounds = Some(bounds);
		self.raster = Some(state);
	}

	pub(crate) fn clear_ray(&mut self) {
		self.ray = None;
	}

	pub(crate) fn clear_raster(&mut self) {
		self.raster = None;
		self.refresh_bounds();
	}

	fn refresh_bounds(&mut self) {
		self.bounds = self.ray.map(|state| VoxelGpuBounds::from_placement(state.placement()));
	}
}

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Reflect)]
pub enum VoxelGpuFormat {
	#[default]
	Volume,
	Surface,
}

pub(crate) fn free_ray_buffers(gpu_data: &mut WorldGpuData, state: RayGpuState) {
	if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(state.tree_id) {
		log::warn!("{err}");
	}
	if let Err(err) = gpu_data.packed_voxel_data_dynamic_buffer.remove_buffer(state.voxels_id) {
		log::warn!("{err}");
	}
}

pub(crate) fn free_raster_buffer(gpu_data: &mut WorldGpuData, state: RasterGpuState) {
	if let Err(err) = gpu_data.packed_raster_face_dynamic_buffer.remove_buffer(state.buffer_id) {
		log::warn!("{err}");
	}
	if let Err(err) = gpu_data.packed_raster_palette_dynamic_buffer.remove_buffer(state.palette_id) {
		log::warn!("{err}");
	}
}

fn free_voxel_gpu_buffers(mut world: DeferredWorld, ctx: HookContext) {
	let Some(state) = world.get_mut::<VoxelGpuState>(ctx.entity).map(|s| s.clone()) else { return; };
	let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return; };
	if let Some(ray) = state.ray {
		free_ray_buffers(&mut gpu_data, ray);
	}
	if let Some(raster) = state.raster {
		free_raster_buffer(&mut gpu_data, raster);
	}
}
