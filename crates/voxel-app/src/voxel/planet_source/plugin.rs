use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use bevy::prelude::*;
use basic_voxel::{BasicVoxel, LodVoxel};
use tracy_client::span;
use voxel_data::{grid::{Grid, GridId}, voxels::{VoxelType, VoxelTypeId}};
use voxel_edit::GridEdits;
use voxel_lightyear::ReplicateVoxels;
use voxel_physics::{components::VoxelCollider, IsStatic, RigidBody};
use voxel_sources::{CancellationToken, ChunkSource, SourceHandle, VoxelSourcesAppExt};
use voxel_streaming::GridStreaming;

use super::config::PLANET_COST;
use super::generation::{build_planet_chunk, build_planet_lod_region};
use super::tiles::{planet_tiles, tile_has_any_chunk_in_region, tile_has_chunk};

pub struct ProceduralPlanetPlugin;

impl Plugin for ProceduralPlanetPlugin {
	fn build(&self, app: &mut App) {
		let _ = planet_tiles();
		let grids = Arc::new(OnceLock::new());
		app.register_voxel_source(ProceduralPlanetSource {
			grids: grids.clone(),
			handle: OnceLock::new(),
		})
		.insert_resource(PlanetGridMap(grids))
		.add_systems(Startup, spawn_planet);
	}
}

#[derive(Resource, Clone)]
struct PlanetGridMap(Arc<OnceLock<HashMap<GridId, usize>>>);

struct ProceduralPlanetSource {
	grids: Arc<OnceLock<HashMap<GridId, usize>>>,
	handle: OnceLock<SourceHandle>,
}

impl ProceduralPlanetSource {
	fn tile_index(&self, grid_id: GridId) -> Option<usize> {
		self.grids.get()?.get(&grid_id).copied()
	}
}

impl ChunkSource for ProceduralPlanetSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn request_available_area(&self, grid: GridId) {
		if let Some(handle) = self.handle.get() {
			handle.presence_loaded(grid);
		}
	}

	fn cost(&self, grid_id: GridId, chunk: IVec3) -> Option<u32> {
		let tile = planet_tiles().get(self.tile_index(grid_id)?)?;
		tile_has_chunk(tile, chunk).then_some(PLANET_COST)
	}

	fn request_load(&self, grid_id: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) {
		let _zone = span!("planet source request_load chunk");
		let voxels = self
			.tile_index(grid_id)
			.and_then(|tile_index| build_planet_chunk(tile_index, chunk, &cancellation));
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.handle.get() {
			let _zone = span!("planet source publish chunk");
			handle.loaded(grid_id, chunk, generation, voxels);
		}
	}

	fn cost_voxels(&self, grid_id: GridId, min: IVec3, size: IVec3, _lod: f32, voxel_type: VoxelTypeId) -> Option<u32> {
		let tile = planet_tiles().get(self.tile_index(grid_id)?)?;
		assert_eq!(voxel_type, LodVoxel::TYPE_INFO.id, "planet source does not support requested voxel type");
		tile_has_any_chunk_in_region(tile, min, size).then_some(PLANET_COST)
	}

	fn request_voxel_area(
		&self,
		grid_id: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		cancellation: CancellationToken,
	) {
		let _zone = span!("planet source request_voxel_area");
		tracy_client::plot!("planet lod level", lod as f64);
		let voxels = self
			.tile_index(grid_id)
			.and_then(|tile_index| build_planet_lod_region(tile_index, min, size, lod, &cancellation));
		if cancellation.is_cancelled() { return; }
		if let Some(handle) = self.handle.get() {
			let _zone = span!("planet source publish lod");
			handle.voxels_loaded(grid_id, min, size, lod, voxel_type, generation, voxels);
		}
	}
}

fn spawn_planet(mut commands: Commands, grids: Res<PlanetGridMap>) {
	let parent = commands
		.spawn((RigidBody, IsStatic, Transform::from_xyz(0.0, 0.0, -2000.0)))
		.id();

	let mut grid_map = HashMap::with_capacity(planet_tiles().len());

	for tile in planet_tiles() {
		let mut streaming = GridStreaming::default();
		for &chunk in &tile.present_chunks {
			streaming.mark_present(chunk);
		}

		let rotation = Quat::from_mat3(&Mat3::from_cols(tile.axis_x, tile.axis_y, tile.normal));
		let transform = Transform {
			translation: tile.origin,
			rotation,
			scale: Vec3::ONE,
		};

		let grid_entity = commands
			.spawn((
				transform,
				Grid::new::<BasicVoxel>(),
				VoxelCollider,
				GridEdits::default(),
				ReplicateVoxels,
				streaming,
			))
			.id();
		grid_map.insert(grid_entity, tile.index);
		commands.entity(parent).add_child(grid_entity);
	}

	let _ = grids.0.set(grid_map);
}
