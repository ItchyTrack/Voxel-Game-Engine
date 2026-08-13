use std::collections::HashMap;
use std::sync::{Arc, OnceLock};

use bevy::prelude::*;
use basic_voxel::{BasicVoxel, LodVoxel};
use tracy_client::span;
use voxel_data::{grid::{Grid, GridId}, voxels::{VoxelType, VoxelTypeId}};
use voxel_edit::GridEdits;
use voxel_physics::{components::VoxelCollider, IsStatic, RigidBody};
use voxel_lightyear::ReplicateVoxels;
use voxel_sources::{CancellationToken, ChunkSource, LendResult, LentChunks, SourceHandle, VoxelSourcesAppExt};
use voxel_streaming::{ForgottenChunks, GridStreaming};

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
			forgotten: ForgottenChunks::default(),
			lent: LentChunks::default(),
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
	forgotten: ForgottenChunks,
	lent: LentChunks,
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
		if self.forgotten.contains(grid_id, chunk) || self.lent.contains(grid_id, chunk) { return None; }
		let tile = planet_tiles().get(self.tile_index(grid_id)?)?;
		tile_has_chunk(tile, chunk).then_some(PLANET_COST)
	}

	fn request_load(&self, grid_id: GridId, chunk: IVec3, edit_index: u64, cancellation: CancellationToken) -> bool {
		if cancellation.is_cancelled() || self.forgotten.contains(grid_id, chunk) || self.lent.contains(grid_id, chunk) { return false; }
		let _zone = span!("planet source request_load chunk");
		let voxels = (!self.forgotten.contains(grid_id, chunk) && !self.lent.contains(grid_id, chunk))
			.then(|| {
				self.tile_index(grid_id)
					.and_then(|tile_index| build_planet_chunk(tile_index, chunk, &cancellation))
			})
			.flatten();
		if cancellation.is_cancelled() { return false; }
		if let Some(handle) = self.handle.get() {
			let _zone = span!("planet source publish chunk");
			handle.loaded(grid_id, chunk, edit_index, voxels);
		}
		true
	}

	fn cost_voxels(&self, grid_id: GridId, min: IVec3, size: IVec3, _lod: f32, voxel_type: VoxelTypeId) -> Option<u32> {
		if !self.forgotten.any_remembered_in(grid_id, min, size) || !self.lent.any_available_in(grid_id, min, size) { return None; }
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
		edit_index: u64,
		cancellation: CancellationToken,
	) -> bool {
		if cancellation.is_cancelled() || !self.lent.any_available_in(grid_id, min, size) { return false; }
		let _zone = span!("planet source request_voxel_area");
		tracy_client::plot!("planet lod level", lod as f64);
		let voxels = self.tile_index(grid_id).and_then(|tile_index| {
			let step = 1i32 << lod.max(0.0).floor() as u32;
			let extent = IVec3::splat(tile_data::CHUNK_SIZE / step);
			let mut out: Option<voxel_data::voxels::Voxels> = None;
			for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
				let offset = IVec3::new(x, y, z);
				let chunk = min + offset;
				if self.forgotten.contains(grid_id, chunk) || self.lent.contains(grid_id, chunk) { continue; }
				let Some(voxels) = build_planet_lod_region(tile_index, chunk, IVec3::ONE, lod, &cancellation) else { continue };
				out.get_or_insert_with(|| voxel_data::voxels::Voxels::new_with_type(voxels.voxel_type_info())).merge_from(&voxels, offset * extent);
			}}}
			out
		});
		if cancellation.is_cancelled() { return false; }
		if let Some(handle) = self.handle.get() {
			let _zone = span!("planet source publish lod");
			handle.voxels_loaded(grid_id, min, size, lod, voxel_type, edit_index, voxels);
		}
		true
	}

	fn lend(&self, grid_id: GridId, chunk: IVec3, cancellation: CancellationToken) -> LendResult {
		if self.forgotten.contains(grid_id, chunk) || !self.lent.begin(grid_id, chunk) { return LendResult::Unavailable; }
		let voxels = self.tile_index(grid_id).and_then(|tile_index| build_planet_chunk(tile_index, chunk, &cancellation));
		if cancellation.is_cancelled() {
			self.lent.end(grid_id, chunk);
			return LendResult::Unavailable;
		}
		LendResult::Borrowed(voxels)
	}


	fn return_area(&self, grid_id: GridId, min: IVec3, size: IVec3) { self.lent.end_area(grid_id, min, size); }

	fn forget(&self, grid_id: GridId, chunk: IVec3) {
		self.forgotten.forget(grid_id, chunk);
		self.lent.end(grid_id, chunk);
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
