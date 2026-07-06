use std::collections::{HashMap, HashSet, VecDeque};

use bevy::ecs::component::Component;
use voxel_data::grid::GridId;
use voxel_streaming::TileIndex;

use crate::coverage::SourceState;
use crate::lod_bands::LodBand;
use crate::replacement_graph::ReplacementGraph;
use crate::types::{TileKey, TileRecord};
use crate::unresolved_tile_index::UnresolvedTileIndex;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraVoxelLoaderSettings {
	pub max_lod: u8,
	pub near_radius_chunks: i32,
	pub rings_per_lod: i32,
	pub requests_per_frame: usize,
}

impl Default for CameraVoxelLoaderSettings {
	fn default() -> Self {
		Self {
			max_lod: 6,
			near_radius_chunks: 5,
			rings_per_lod: 3,
			requests_per_frame: 20000,
		}
	}
}

#[derive(Component, Default, Debug, Clone)]
pub struct CameraVoxelLoader {
	pub(crate) settings: CameraVoxelLoaderSettings,
	pub(crate) desired_tiles: HashSet<TileKey>,
	pub(crate) desired_tile_index: HashMap<GridId, TileIndex<TileKey>>,
	pub(crate) bands: HashMap<GridId, Vec<LodBand>>,
	pub(crate) coverage_sources: HashMap<TileKey, SourceState>,
	pub(crate) unresolved_tiles: UnresolvedTileIndex,
	pub(crate) replacement_graph: ReplacementGraph,
	pub(crate) queue: VecDeque<TileKey>,
	pub(crate) tiles: HashMap<TileKey, TileRecord>,
}

impl CameraVoxelLoader {
	pub fn with_settings(settings: CameraVoxelLoaderSettings) -> Self { Self { settings, ..Default::default() } }
	pub fn settings(&self) -> &CameraVoxelLoaderSettings { &self.settings }
	pub fn set_settings(&mut self, settings: CameraVoxelLoaderSettings) { self.settings = settings; }

	pub(crate) fn insert_desired_tile(&mut self, key: TileKey) -> bool {
		if !self.desired_tiles.insert(key) {
			return false;
		}
		self.desired_tile_index.entry(key.grid).or_default().insert(key);
		true
	}

	pub(crate) fn remove_desired_tile(&mut self, key: TileKey) -> bool {
		if !self.desired_tiles.remove(&key) {
			return false;
		}
		if let Some(index) = self.desired_tile_index.get_mut(&key.grid) {
			index.remove(key);
			if index.is_empty() {
				self.desired_tile_index.remove(&key.grid);
			}
		}
		true
	}

	pub(crate) fn desired_tiles_in_area(&self, grid: GridId, min: bevy::math::IVec3, size: bevy::math::IVec3) -> Vec<TileKey> {
		let mut out = Vec::new();
		if let Some(index) = self.desired_tile_index.get(&grid) {
			index.for_each_overlapping(min, size, self.settings.max_lod, None, |key| out.push(key));
		}
		out
	}
}
