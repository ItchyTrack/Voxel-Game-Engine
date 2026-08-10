use basic_voxel::MarchingVoxel;
use bevy::prelude::*;
use tile_data::TileGenerationContext;
use voxel_data::{grid::Grid, voxels::VoxelType};
use voxel_gpu::{RenderingContext, RenderingType};

#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VoxelRenderMode {
	#[default]
	Ray,
	Raster,
}

#[derive(Default)]
pub struct VoxelAppRenderingPlugin;

impl Plugin for VoxelAppRenderingPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<VoxelRenderMode>()
			.add_systems(Update, apply_grid_rendering_contexts);
	}
}

fn apply_grid_rendering_contexts(
	mut commands: Commands,
	mode: Res<VoxelRenderMode>,
	grids: Query<(Entity, &Grid, Option<&TileGenerationContext>)>,
) {
	let rendering_type = match *mode {
		VoxelRenderMode::Ray => RenderingType::Ray,
		VoxelRenderMode::Raster => RenderingType::Raster,
	};
	let mut context = None;
	for (entity, grid, current) in &grids {
		if grid.voxel_type_info().id == MarchingVoxel::TYPE_INFO.id { continue; }
		if current.is_none() || mode.is_changed() {
			let context = context.get_or_insert_with(|| TileGenerationContext::new(RenderingContext { rendering_type }));
			commands.entity(entity).insert(context.clone());
		}
	}
}
