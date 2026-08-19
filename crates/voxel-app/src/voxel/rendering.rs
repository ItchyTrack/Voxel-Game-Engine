use basic_voxel::MarchingVoxel;
use bevy::prelude::*;
use tile_data::TileGenerationParameters;
use voxel_data::{grid::Grid, voxels::VoxelType};
use voxel_gpu::RenderingContext;
use voxel_marching_renderer::MarchingRenderingType;
use voxel_raster_renderer::RasterRenderingType;
use voxel_ray_renderer::RayRenderingType;

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
	ray: Res<RayRenderingType>,
	raster: Res<RasterRenderingType>,
	marching: Res<MarchingRenderingType>,
	grids: Query<(Entity, &Grid, Option<&TileGenerationParameters>)>,
) {
	let selected = match *mode {
		VoxelRenderMode::Ray => ray.0,
		VoxelRenderMode::Raster => raster.0,
	};
	for (entity, grid, current) in &grids {
		let rendering_type = if grid.voxel_type_info().id == MarchingVoxel::TYPE_INFO.id {
			marching.0
		} else {
			selected
		};
		let matches = current
			.and_then(|parameters| parameters.downcast_ref::<RenderingContext>())
			.is_some_and(|context| context.rendering_type == rendering_type);
		if !matches {
			commands.entity(entity).insert(TileGenerationParameters::new(RenderingContext { rendering_type }));
		}
	}
}
