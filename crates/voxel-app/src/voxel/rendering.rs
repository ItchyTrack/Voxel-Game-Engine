use basic_voxel::MarchingVoxel;
use bevy::{ecs::component::ComponentIdFor, prelude::*};
use voxel_data::{grid::Grid, voxels::VoxelType};
use voxel_engine::GridRenderClass;
use voxel_marching_renderer::tile_data::MarchingTileClass;
use voxel_raster_renderer::tile_data::RasterTileClass;
use voxel_ray_renderer::tile_data::RayTileClass;

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
			.add_systems(Update, apply_grid_render_classes);
	}
}

fn apply_grid_render_classes(
	mut commands: Commands,
	mode: Res<VoxelRenderMode>,
	ray: ComponentIdFor<RayTileClass>,
	raster: ComponentIdFor<RasterTileClass>,
	marching: ComponentIdFor<MarchingTileClass>,
	grids: Query<(Entity, &Grid, Option<&GridRenderClass>)>,
) {
	let selected = match *mode {
		VoxelRenderMode::Ray => ray.get(),
		VoxelRenderMode::Raster => raster.get(),
	};
	for (entity, grid, current) in &grids {
		let class = if grid.voxel_type_info().id == MarchingVoxel::TYPE_INFO.id { marching.get() } else { selected };
		if current.map(|current| current.0) != Some(class) {
			commands.entity(entity).insert(GridRenderClass(class));
		}
	}
}
