use std::sync::Arc;

use bevy::{ecs::system::SystemParam, prelude::*};
use bile_data::{DataRequester, DataType, DirtyState, Request};
use bile_math::NonZeroRegion;
use bile_tiles::TileIndexKey;
use tile_data::VoxelRegionResult;
use voxel_data::{grid::GridId, voxels::VoxelTypeId};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelRequest {
	pub grid: GridId,
	pub area: NonZeroRegion,
	pub lod: u8,
	pub voxel_type: Option<VoxelTypeId>,
}

impl TileIndexKey for VoxelRequest {
	fn region(self) -> NonZeroRegion { self.area }
}

/// Every voxel region the sources returned for a [`VoxelRequest`]. Empty when no source owns the area.
pub type VoxelRegions = Arc<Vec<VoxelRegionResult>>;

pub struct VoxelData;

impl DataType for VoxelData {
	type Params = VoxelRequest;
	type Data = VoxelRegions;
}

/// Every region the sources reported as present for a grid.
pub type PresenceRegions = Arc<Vec<NonZeroRegion>>;

pub struct VoxelPresence;

impl DataType for VoxelPresence {
	type Params = GridId;
	type Data = PresenceRegions;
}

#[derive(SystemParam)]
pub struct VoxelRequester<'w, 's> {
	requester: DataRequester<'w, 's, VoxelData>,
}

impl<'w, 's> VoxelRequester<'w, 's> {
	pub fn request(&mut self, request: VoxelRequest, requester_node: Entity) -> Option<VoxelRegions> {
		self.requester.request(Request::new(request, DirtyState::UpToDate), requester_node)
	}

	pub fn cancel(&mut self, request: &VoxelRequest, requester_node: Entity) {
		self.requester.cancel(request, requester_node);
	}
}

#[derive(SystemParam)]
pub struct PresenceRequester<'w, 's> {
	requester: DataRequester<'w, 's, VoxelPresence>,
}

impl<'w, 's> PresenceRequester<'w, 's> {
	pub fn request(&mut self, grid: GridId, requester_node: Entity) -> Option<PresenceRegions> {
		self.requester.request(Request::new(grid, DirtyState::UpToDate), requester_node)
	}

	pub fn cancel(&mut self, grid: GridId, requester_node: Entity) {
		self.requester.cancel(&grid, requester_node);
	}
}
