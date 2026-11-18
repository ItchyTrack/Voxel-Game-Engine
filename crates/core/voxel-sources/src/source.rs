use std::any::Any;

use bevy::prelude::*;
use bile_math::NonZeroRegion;
use crossbeam_channel::Sender;
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_tasks::CancellationToken;

use crate::request::{RequestId, SourceResult, SourceResultData};

/// The entity of a registered source.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SourceId(pub Entity);

#[derive(Clone)]
pub struct SourceHandle {
	pub(crate) id: SourceId,
	pub(crate) messages: Sender<SourceResult>,
}

impl SourceHandle {
	pub fn id(&self) -> SourceId { self.id }

	pub fn voxels(&self, request_id: RequestId, grid: GridId, region: NonZeroRegion, lod: u8, voxels: Voxels) {
		let _ = self.messages.send(SourceResult { request_id, data: SourceResultData::Voxels { grid, region, lod, voxels } });
	}

	pub fn voxels_loaded(&self, request_id: RequestId) {
		let _ = self.messages.send(SourceResult { request_id, data: SourceResultData::VoxelsLoaded });
	}

	pub fn presence(&self, request_id: RequestId, grid: GridId, region: NonZeroRegion) {
		let _ = self.messages.send(SourceResult { request_id, data: SourceResultData::Presence { grid, region } });
	}

	pub fn presence_loaded(&self, request_id: RequestId) {
		let _ = self.messages.send(SourceResult { request_id, data: SourceResultData::PresenceLoaded });
	}
}

/// A source of voxels. Each registered source is a Bile gather node, see [`crate::VoxelSourceNode`].
pub trait ChunkSource: Any + Send + Sync {
	fn init(&mut self, handle: SourceHandle);

	/// Whether the source owns any of the region. Only the sources that do are asked for its voxels.
	fn covers(&self, grid: GridId, region: NonZeroRegion) -> bool;

	/// Sends the voxels the source owns in the region through its handle, then `voxels_loaded`.
	fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
	);

	/// Sends the regions the source has for the grid through its handle, then `presence_loaded`.
	fn request_presence(&mut self, request_id: RequestId, cancellation: CancellationToken, grid: GridId);

	fn acquire_ownership(&mut self, grid: GridId, region: NonZeroRegion);

	fn relinquish_ownership(&mut self, grid: GridId, region: NonZeroRegion);
}
