use std::future::Future;

use bevy::prelude::*;
use bile_math::NonZeroRegion;
use bile_tiles::TileKey;
use voxel_data::{
	grid::GridId,
	voxels::{VoxelTypeId, Voxels},
};

pub use async_trait::async_trait;

#[async_trait]
pub trait TileBuilder: Send + Sync + 'static {
	type Output: Component;
	const REBUILD_ON_EDIT: bool = true;
	async fn build(&self, session: TileBuildingSession) -> Option<Self::Output>;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelRegionRequest {
	pub area: NonZeroRegion,
	pub lod: u8,
	pub voxel_type: Option<VoxelTypeId>,
}

#[derive(Debug)]
pub struct VoxelRegionResult {
	pub area: NonZeroRegion,
	pub lod: u8,
	pub voxels: Voxels,
}

pub type ReceiveVoxelsFuture<'a> = std::pin::Pin<Box<dyn Future<Output = Option<VoxelRegionResult>> + Send + 'a>>;

pub trait TileBuildingVoxelReader: Send + 'static {
	fn request_voxels(&mut self, request: VoxelRegionRequest);
	fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_>;
}

pub struct TileBuildingSession {
	pub grid: GridId,
	pub key: TileKey,
	reader: Box<dyn TileBuildingVoxelReader>,
}

impl TileBuildingSession {
	pub fn new(grid: GridId, key: TileKey, reader: Box<dyn TileBuildingVoxelReader>) -> Self {
		Self { grid, key, reader }
	}

	pub fn request_voxels(&mut self, area: NonZeroRegion, lod: u8, voxel_type: Option<VoxelTypeId>) {
		self.reader.request_voxels(VoxelRegionRequest { area, lod, voxel_type });
	}

	pub fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_> {
		self.reader.receive_voxels()
	}
}
