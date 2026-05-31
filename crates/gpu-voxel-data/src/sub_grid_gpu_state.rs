use bevy::ecs::lifecycle::HookContext;
use bevy::ecs::world::DeferredWorld;
use bevy::prelude::*;

use crate::world_gpu_data::WorldGpuData;

/// GPU residency of a sub-grid. Present on a sub-grid entity exactly when its
/// voxels are uploaded; removing it (or despawning the entity) frees the packed
/// buffers via the `on_remove` hook.
#[derive(Component, Clone, Copy, Debug)]
#[component(on_remove = free_subgrid_gpu_buffers)]
pub struct SubGridGpuState {
	lod_level: f32,
	tree_id: u32,
	voxels_id: u32,
}

impl SubGridGpuState {
	pub(crate) fn new(lod_level: f32, tree_id: u32, voxels_id: u32) -> Self {
		Self { lod_level, tree_id, voxels_id }
	}

	pub fn lod_level(&self) -> f32 { self.lod_level }
	pub fn tree_id(&self) -> u32 { self.tree_id }
	pub fn voxels_id(&self) -> u32 { self.voxels_id }
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
