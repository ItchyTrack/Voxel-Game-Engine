use std::collections::HashMap;

use bevy::ecs::{component::Component, entity::Entity};
use tile_data::ChunkRegion;
use voxel_data::grid::GridId;
use voxel_streaming::TileClassId;

use crate::lod_bands::LodBand;
use crate::tile_lifecycle::{TileLifecycle, TileResolution};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraVoxelLoaderSettings {
	pub max_lod: u8,
	pub near_radius_chunks: u32,
	pub rings_per_lod: u32,
}

impl Default for CameraVoxelLoaderSettings {
	fn default() -> Self {
		Self {
			max_lod: 6,
			near_radius_chunks: 5,
			rings_per_lod: 5,
		}
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoverageDebugState {
	Pending,
	Loaded,
	Empty,
	Waiting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoverageDebugTile {
	pub grid: Entity,
	pub region: ChunkRegion,
	pub lod: u8,
	pub state: CoverageDebugState,
}

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct CameraVoxelTileClass(pub TileClassId);

#[derive(Component, Default, Debug)]
pub struct CameraVoxelLoader {
	pub(crate) settings: CameraVoxelLoaderSettings,
	pub(crate) bands: HashMap<GridId, Vec<LodBand>>,
	pub(crate) classes: HashMap<GridId, TileClassId>,
	pub(crate) tiles: TileLifecycle,
}

impl CameraVoxelLoader {
	pub fn with_settings(settings: CameraVoxelLoaderSettings) -> Self { Self { settings, ..Default::default() } }
	pub fn settings(&self) -> &CameraVoxelLoaderSettings { &self.settings }
	pub fn set_settings(&mut self, settings: CameraVoxelLoaderSettings) { self.settings = settings; }
	pub fn tiles_to_render(&self) -> impl Iterator<Item = Entity> + '_ { self.tiles.tiles_to_render() }
	pub fn coverage_debug_tiles(&self) -> Vec<CoverageDebugTile> {
		let mut states = HashMap::new();
		for (key, _, retained) in self.tiles.coverage_debug_tiles() {
			states.insert(key, if retained { CoverageDebugState::Waiting } else { CoverageDebugState::Pending });
		}
		for (key, entry) in self.tiles.entries() {
			let state = if !self.tiles.contains_desired(key) {
				CoverageDebugState::Waiting
			} else {
				match &entry.resolution {
					TileResolution::Requested => CoverageDebugState::Pending,
					TileResolution::Empty => CoverageDebugState::Empty,
					TileResolution::Tile(_) => CoverageDebugState::Loaded,
				}
			};
			states.entry(key).and_modify(|current| {
				if !matches!(current, CoverageDebugState::Waiting) { *current = state; }
			}).or_insert(state);
		}
		let mut tiles: Vec<_> = states
			.into_iter()
			.map(|(key, state)| CoverageDebugTile { grid: key.grid, region: key.region.into(), lod: key.lod, state })
			.collect();
		tiles.sort_by_key(|tile| {
			let min = tile.region.min();
			(tile.grid.to_bits(), tile.lod, min.x, min.y, min.z)
		});
		tiles
	}
}
