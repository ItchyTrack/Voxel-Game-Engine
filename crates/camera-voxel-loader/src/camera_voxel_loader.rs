use std::collections::{HashMap, HashSet, VecDeque};

use bevy::ecs::component::Component;

use crate::coverage::{CoverageRecord, CoverageSource};
use crate::replacement_graph::ReplacementGraph;
use crate::types::{TileKey, TileRecord};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CameraVoxelLoaderSettings {
	pub max_lod: u8,
	pub near_radius_chunks: i32,
	pub rings_per_lod: i32,
	pub requests_per_frame: usize,
	pub max_in_flight: usize,
}

impl Default for CameraVoxelLoaderSettings {
	fn default() -> Self { Self { max_lod: 6, near_radius_chunks: 3, rings_per_lod: 2, requests_per_frame: 16, max_in_flight: 128 } }
}

#[derive(Component, Default, Debug, Clone)]
pub struct CameraVoxelLoader {
	pub(crate) settings: CameraVoxelLoaderSettings,
	pub(crate) desired_tiles: HashSet<TileKey>,
	pub(crate) coverage_sources: HashMap<CoverageSource, CoverageRecord>,
	pub(crate) replacement_graph: ReplacementGraph,
	pub(crate) queue: VecDeque<TileKey>,
	pub(crate) tiles: HashMap<TileKey, TileRecord>,
}

impl CameraVoxelLoader {
	pub fn with_settings(settings: CameraVoxelLoaderSettings) -> Self { Self { settings, ..Default::default() } }
	pub fn settings(&self) -> &CameraVoxelLoaderSettings { &self.settings }
	pub fn set_settings(&mut self, settings: CameraVoxelLoaderSettings) { self.settings = settings; }
}
