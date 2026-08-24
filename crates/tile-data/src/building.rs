use std::{future::Future, sync::Arc};

use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_data::{
	grid::GridId,
	voxels::{VoxelTypeId, Voxels},
};

use crate::{NonZeroChunkRegion, TileClassId, TileData, TileBuildingParameters, TileKey, class::TileBuildingData};

pub use async_trait::async_trait;

#[async_trait]
pub trait TileBuilder: Send + Sync + 'static {
	async fn build(&self, session: TileBuildingSession) -> Option<Box<dyn TileData>>;
}

#[derive(Resource, Default)]
pub struct TileBuilderRegistry {
	builders: FxHashMap<TileClassId, Arc<dyn TileBuilder>>,
}

impl TileBuilderRegistry {
	pub fn insert<G: TileBuilder>(&mut self, tile_class_id: TileClassId, builder: G) {
		self.builders.insert(tile_class_id, Arc::new(builder));
	}

	pub fn builder(&self, class: TileClassId) -> Arc<dyn TileBuilder> {
		self.builders.get(&class).cloned().unwrap_or_else(|| panic!("no tile builder registered for {class:?}"))
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VoxelRegionRequest {
	pub area: NonZeroChunkRegion,
	pub lod: u8,
	pub voxel_type: VoxelTypeId,
}

#[derive(Debug)]
pub struct VoxelRegionResult {
	pub area: NonZeroChunkRegion,
	pub lod: u8,
	pub voxels: Voxels,
}

pub type ReceiveVoxelsFuture<'a> = std::pin::Pin<Box<dyn Future<Output = Option<VoxelRegionResult>> + Send + 'a>>;

pub trait TileBuildingVoxelReader: Send + 'static {
	fn request_voxels(&mut self, tile_key: TileKey, request: VoxelRegionRequest);
	fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_>;
}

pub struct TileBuildingSession {
	pub grid: GridId,
	pub key: TileKey,
	context: TileBuildingParameters,
	reader: Box<dyn TileBuildingVoxelReader>,
}

impl TileBuildingSession {
	pub fn new(
		grid: GridId,
		key: TileKey,
		context: TileBuildingParameters,
		reader: Box<dyn TileBuildingVoxelReader>,
	) -> Self {
		Self { grid, key, context, reader }
	}

	pub fn context<T: TileBuildingData + 'static>(&self) -> &T {
		self.context.downcast_ref().unwrap_or_else(|| panic!("tile builder received building context of the wrong type"))
	}

	pub fn request_voxels(
		&mut self,
		area: NonZeroChunkRegion,
		lod: u8,
		voxel_type: VoxelTypeId,
	) {
		self.reader.request_voxels(
			self.key,
			VoxelRegionRequest {
				area,
				lod,
				voxel_type,
			}
		);
	}

	pub fn receive_voxels(&mut self) -> ReceiveVoxelsFuture<'_> {
		self.reader.receive_voxels()
	}
}
