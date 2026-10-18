use bevy::{ecs::component::ComponentId, prelude::*};
use bile_math::Region;
use bile_region_loader::{LodBand, RegionLoaderComponent, RegionLoaderPlugin};
use tile_data::{CHUNK_SIZE, chunk_origin};
use voxel_data::grid::Grid;

pub struct CameraTiles;

pub type CameraTileLoader = RegionLoaderComponent<CameraTiles>;

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct GridRenderClass(pub ComponentId);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct FreezeCameraVoxelLoader(pub bool);

#[derive(Resource, Debug, Clone, PartialEq, Eq)]
pub struct CameraTileSettings {
	pub max_lod: u8,
	pub near_radius_chunks: u32,
	pub rings_per_lod: u32,
}

impl Default for CameraTileSettings {
	fn default() -> Self {
		Self { max_lod: 6, near_radius_chunks: 5, rings_per_lod: 5 }
	}
}

fn align_bounds(min: IVec3, max: IVec3, size: i32) -> (IVec3, IVec3) {
	(min.div_euclid(IVec3::splat(size)) * size, (max + (size - 1)).div_euclid(IVec3::splat(size)) * size)
}

fn camera_bands(center: IVec3, class: ComponentId, settings: &CameraTileSettings) -> Vec<LodBand> {
	let to_region = |(min, max): (IVec3, IVec3)| Region::from_min_end(chunk_origin(min), chunk_origin(max)).unwrap();
	let radius = settings.near_radius_chunks as i32;
	let near = align_bounds(center - radius, center + radius + 1, 2);
	let mut bands = vec![LodBand { class, lod: 0, outer: to_region(near), inner: None }];
	let mut previous = near;
	for lod in 1..=settings.max_lod {
		let expand = IVec3::splat((1 << lod) * settings.rings_per_lod as i32);
		let mut outer = (previous.0 - expand, previous.1 + expand);
		if lod < settings.max_lod {
			outer = align_bounds(outer.0, outer.1, 1 << (lod + 1));
		}
		bands.push(LodBand { class, lod, outer: to_region(outer), inner: Some(to_region(previous)) });
		previous = outer;
	}
	bands
}

fn update_camera_tile_regions(
	mut commands: Commands,
	freeze: Res<FreezeCameraVoxelLoader>,
	settings: Res<CameraTileSettings>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	mut grids: Query<(Entity, &GlobalTransform, &GridRenderClass, Option<&mut CameraTileLoader>), With<Grid>>,
) {
	if freeze.0 { return; }
	let camera = cameras.iter().find(|(camera, _)| camera.is_active).map(|(_, transform)| transform.translation());
	for (entity, grid_transform, class, loader) in &mut grids {
		let bands = camera.map_or_else(Vec::new, |camera| {
			let local = grid_transform.affine().inverse().transform_point3(camera);
			camera_bands((local / CHUNK_SIZE as f32).round().as_ivec3(), class.0, &settings)
		});
		match loader {
			Some(mut loader) => loader.set_regions(bands),
			None => {
				let mut loader = CameraTileLoader::new(CHUNK_SIZE);
				loader.set_regions(bands);
				commands.entity(entity).insert(loader);
			}
		}
	}
}

pub struct CameraTilesPlugin;

impl Plugin for CameraTilesPlugin {
	fn build(&self, app: &mut App) {
		app.add_plugins(RegionLoaderPlugin::<CameraTiles>::default())
			.init_resource::<FreezeCameraVoxelLoader>()
			.init_resource::<CameraTileSettings>()
			.add_systems(Update, update_camera_tile_regions);
	}
}
