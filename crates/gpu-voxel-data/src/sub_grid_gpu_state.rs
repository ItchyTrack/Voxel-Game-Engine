use bevy::ecs::lifecycle::HookContext;
use bevy::ecs::world::DeferredWorld;
use bevy::math::I16Vec3;
use bevy::prelude::*;

use crate::world_gpu_data::WorldGpuData;

// Snapshotted when the GPU tree is built, so the renderer positions the data
// from the same snapshot it was encoded from, not the live sub-grid.
#[derive(Clone, Copy, Debug)]
pub struct SubGridPlacement {
	pub tree_root_pos: I16Vec3,
	pub bounds_min: I16Vec3,
	pub bounds_max: I16Vec3,
}

/// GPU residency of a sub-grid. Present on a sub-grid entity exactly when its
/// voxels are uploaded; removing it (or despawning the entity) frees the packed
/// buffers via the `on_remove` hook.
#[derive(Component, Clone, Copy, Debug)]
#[component(on_remove = free_subgrid_gpu_buffers)]
pub struct SubGridGpuState {
	lod_level: f32,
	tree_id: u32,
	voxels_id: u32,
	placement: SubGridPlacement,
}

impl SubGridGpuState {
	pub(crate) fn new(lod_level: f32, tree_id: u32, voxels_id: u32, placement: SubGridPlacement) -> Self {
		Self { lod_level, tree_id, voxels_id, placement }
	}

	pub fn lod_level(&self) -> f32 { self.lod_level }
	pub fn tree_id(&self) -> u32 { self.tree_id }
	pub fn voxels_id(&self) -> u32 { self.voxels_id }
	pub fn placement(&self) -> SubGridPlacement { self.placement }
}

fn free_subgrid_gpu_buffers(mut world: DeferredWorld, ctx: HookContext) {
	let Some(state) = world.get_mut::<SubGridGpuState>(ctx.entity).map(|s| *s) else { return };
	let Some(mut gpu_data) = world.get_resource_mut::<WorldGpuData>() else { return };
	if let Err(err) = gpu_data.packed_64_tree_dynamic_buffer.remove_buffer(state.tree_id) {
		log::warn!("{err}");
	}
	if let Err(err) = gpu_data.packed_voxel_data_dynamic_buffer.remove_buffer(state.voxels_id) {
		log::warn!("{err}");
	}
}
