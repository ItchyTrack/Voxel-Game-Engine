use std::collections::{HashMap, HashSet, VecDeque};

use bevy::{ecs::component::Component, math::IVec3};
use voxel_data::grid::GridId;

use crate::retirement::{RetireDeps, RetireTarget};
use crate::types::{ChunkKey, TileKey, TileRecord};

#[derive(Debug, Clone)]
pub struct CameraVoxelLoaderSettings {
	pub max_lod: u8,
	pub near_radius_chunks: i32,
	pub rings_per_lod: i32,
	pub requests_per_frame: usize,
	pub max_in_flight: usize,
}

impl Default for CameraVoxelLoaderSettings {
	fn default() -> Self {
		Self {
			max_lod: 6,
			near_radius_chunks: 3,
			rings_per_lod: 2,
			requests_per_frame: 16,
			max_in_flight: 128,
		}
	}
}

#[derive(Component, Default, Debug, Clone)]
pub struct CameraVoxelLoader {
	pub(crate) settings: CameraVoxelLoaderSettings,
	pub(crate) desired_chunks: HashSet<ChunkKey>,
	pub(crate) desired_tiles: HashSet<TileKey>,
	pub(crate) retiring_chunks: HashMap<ChunkKey, RetireDeps>,
	pub(crate) retiring_tiles: HashMap<TileKey, RetireDeps>,
	pub(crate) tile_dependents: HashMap<TileKey, Vec<RetireTarget>>,
	pub(crate) grid_centers: HashMap<GridId, IVec3>,
	pub(crate) queue: VecDeque<TileKey>,
	pub(crate) tiles: HashMap<TileKey, TileRecord>,
}
