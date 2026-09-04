use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use bevy::prelude::*;
use basic_voxel::{BasicVoxel, LodVoxel};
use tile_data::NonZeroChunkRegion;
use voxel_data::voxels::VoxelType;
use voxel_data::{grid::{Grid, GridId}, voxels::VoxelTypeId};
use voxel_physics::{components::{VoxelCollider, VoxelMass}, IsStatic, RigidBody};
use voxel_lightyear::ReplicateVoxels;
use voxel_mass::{MassError, SourceMassChange, SourceMassState, VoxelMassSet};
use voxel_sources::{ForgottenChunks, ChunkSource, RequestId, SourceCoverage, SourceHandle, SourceManager, VoxelSourcesAppExt, edit::{GridEditIdManager, GridGeneration}};
use voxel_streaming::GridStreaming;
use voxel_tasks::{AsyncPriorityTaskPool, CancellationToken};

use super::generation::{build_planet_region, planet_mass_properties, planet_voxel_unchecked, planet_lod_voxel_unchecked};
use super::tiles::{planet_tiles, tile_has_chunk};

pub struct ProceduralPlanetPlugin;

impl Plugin for ProceduralPlanetPlugin {
	fn build(&self, app: &mut App) {
		let _ = planet_tiles();
		let grids = Arc::new(OnceLock::new());
		let mass_state = SourceMassState::default();
		app.register_voxel_source(ProceduralPlanetSource {
			grids: grids.clone(),
			handle: OnceLock::new(),
			forgotten: ForgottenChunks::default(),
			mass_state: mass_state.clone(),
		})
		.insert_resource(PlanetGridMap { grids, mass_state })
		.add_systems(Startup, spawn_planet)
		.add_systems(FixedUpdate, drain_planet_source_mass_changes.in_set(VoxelMassSet::SourceDrain));
	}
}

#[derive(Resource, Clone)]
struct PlanetGridMap {
	grids: Arc<OnceLock<HashMap<GridId, usize>>>,
	mass_state: SourceMassState,
}

struct ProceduralPlanetSource {
	grids: Arc<OnceLock<HashMap<GridId, usize>>>,
	handle: OnceLock<SourceHandle>,
	forgotten: ForgottenChunks,
	mass_state: SourceMassState,
}

impl ProceduralPlanetSource {
	fn tile_index(&self, grid_id: GridId) -> Option<usize> {
		self.grids.get()?.get(&grid_id).copied()
	}
}

impl ChunkSource for ProceduralPlanetSource {
	fn init(&mut self, handle: SourceHandle) {
		self.mass_state.set_source_id(handle.id());
		let _ = self.handle.set(handle);
	}

	fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		let Some(tile_index) = self.tile_index(grid) else { return SourceCoverage::None };
		let Some(tile) = planet_tiles().get(tile_index) else { return SourceCoverage::None };

		let mut owned_chunks = Vec::new();
		for z in region.min().z..region.end().z {
			for y in region.min().y..region.end().y {
				for x in region.min().x..region.end().x {
					let chunk = IVec3::new(x, y, z);
					if tile_has_chunk(tile, chunk) && !self.forgotten.contains(grid, chunk) {
						owned_chunks.push(chunk);
					}
				}
			}
		}
		let coverage = if owned_chunks.is_empty() {
			return SourceCoverage::None;
		} else if owned_chunks.len() == region.area() as usize {
			SourceCoverage::All
		} else {
			SourceCoverage::Some
		};

		let use_basic_voxel = match voxel_type {
			Some(id) if id == BasicVoxel::TYPE_INFO.id && lod == 0 => true,
			Some(id) if id == LodVoxel::TYPE_INFO.id => false,
			_ => lod == 0,
		};

		let handle = self.handle.get().expect("planet source was not initialized").clone();
		let cancellation = cancellation.clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("PlanetSource build").entered();
			let voxels = if use_basic_voxel {
				build_planet_region(tile_index, region, &owned_chunks, lod, &cancellation, planet_voxel_unchecked)
			} else {
				build_planet_region(tile_index, region, &owned_chunks, lod, &cancellation, planet_lod_voxel_unchecked)
			};
			if !cancellation.is_cancelled() && let Some(voxels) = voxels {
				handle.voxels(request_id, grid, region, lod, generation, voxels);
			}
			handle.voxels_loaded(request_id, generation);
		});
		coverage
	}

	fn request_presence(&mut self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let handle = self.handle.get().expect("planet source was not initialized");
		if let Some(tile_index) = self.tile_index(grid)
			&& let Some(tile) = planet_tiles().get(tile_index) {
			for &chunk in &tile.present_chunks {
				handle.presence(request_id, grid, NonZeroChunkRegion::from_single(chunk));
			}
		}
		handle.presence_loaded(request_id);
	}

	fn acquire_ownership(&mut self, grid: GridId, region: NonZeroChunkRegion) {
		let Some(tile_index) = self.tile_index(grid) else { return };
		if planet_tiles().get(tile_index).is_none() { return; }
		self.forgotten.remember_area(grid, region);
	}

	fn relinquish_ownership(&mut self, grid: GridId, region: NonZeroChunkRegion) {
		let Some(tile_index) = self.tile_index(grid) else { return };
		if planet_tiles().get(tile_index).is_none() { return; }
		self.forgotten.forget_area(grid, region);
	}
}

fn spawn_planet(mut commands: Commands, grids: Res<PlanetGridMap>) {
	let parent = commands
		.spawn((
			RigidBody,
			IsStatic,
			Transform::from_xyz(0.0, 0.0, -2000.0)
		))
		.id();

	let mut grid_map = HashMap::with_capacity(planet_tiles().len());

	for tile in planet_tiles() {
		let mut streaming = GridStreaming::default();
		for &chunk in &tile.present_chunks {
			streaming.mark_present_area(NonZeroChunkRegion::from_single(chunk));
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
				VoxelMass,
				GridEditIdManager::default(),
				ReplicateVoxels,
				streaming,
			))
			.id();
		grid_map.insert(grid_entity, tile.index);
		grids.mass_state.initialize_grid_once(
			grid_entity,
			planet_mass_properties(tile),
			MassError::ZERO,
		);
		commands.entity(parent).add_child(grid_entity);
	}

	let _ = grids.grids.set(grid_map);
}

fn drain_planet_source_mass_changes(
	sources: Res<SourceManager>,
	mut changes: MessageWriter<SourceMassChange>,
) {
	let Some(source) = sources.get_source::<ProceduralPlanetSource>() else { return };
	changes.write_batch(source.mass_state.drain_changes());
}
