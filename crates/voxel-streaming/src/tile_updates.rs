use bevy::prelude::*;
use tile_data::TileKey;
use voxel_data::grid::GridId;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TileLoadStatus {
	Ready(Entity),
	Empty,
}

#[derive(Clone, Copy, Debug)]
pub struct TileLoadUpdate {
	pub grid: GridId,
	pub requester: Entity,
	pub key: TileKey,
	pub status: TileLoadStatus,
}
