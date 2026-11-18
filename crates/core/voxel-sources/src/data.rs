use std::{collections::HashMap, hash::Hash};

use bevy::{ecs::system::SystemParam, prelude::*};
use bile_data::{DirtyState, streaming::{self, StreamingDataType}};
use bile_math::NonZeroRegion;
use bile_tiles::TileIndexKey;
use tile_data::VoxelRegion;
use voxel_data::{grid::GridId, voxels::VoxelTypeId};

use crate::edit::VoxelEdit;

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

/// The voxels of a [`VoxelRequest`], gathered from every source that owns part of the area.
/// Each push is one region from one source. A stream with no pushes means no source has voxels there.
pub struct VoxelData;

impl StreamingDataType for VoxelData {
	type Params = VoxelRequest;
	type Data = VoxelRegion;
	type Diff = VoxelEdit;
}

/// The regions the sources report as present for a grid, one region per push.
pub struct VoxelPresence;

impl StreamingDataType for VoxelPresence {
	type Params = GridId;
	type Data = NonZeroRegion;
	type Diff = ();
}

#[derive(SystemParam)]
pub struct VoxelRequester<'w, 's> {
	requester: streaming::DataRequester<'w, 's, VoxelData>,
}

impl<'w, 's> VoxelRequester<'w, 's> {
	/// The voxels are streamed to the requester node, which must be a [`VoxelData`] consumer.
	pub fn request(&mut self, request: VoxelRequest, requester_node: Entity) {
		self.requester.request(streaming::Request { params: request, allowed_tick: DirtyState::UpToDate }, requester_node);
	}

	pub fn cancel(&mut self, request: &VoxelRequest, requester_node: Entity) {
		self.requester.cancel(request, requester_node);
	}
}

#[derive(SystemParam)]
pub struct PresenceRequester<'w, 's> {
	requester: streaming::DataRequester<'w, 's, VoxelPresence>,
}

impl<'w, 's> PresenceRequester<'w, 's> {
	/// The regions are streamed to the requester node, which must be a [`VoxelPresence`] consumer.
	pub fn request(&mut self, grid: GridId, requester_node: Entity) {
		self.requester.request(streaming::Request { params: grid, allowed_tick: DirtyState::UpToDate }, requester_node);
	}

	pub fn cancel(&mut self, grid: GridId, requester_node: Entity) {
		self.requester.cancel(&grid, requester_node);
	}
}

/// Collects the pushes of streams for consumers that use the data once a stream finishes.
/// A stream that started before its params were last dirtied is stale and is dropped when it finishes.
pub struct StreamCollector<P, D> {
	streams: HashMap<P, (u64, Vec<D>)>,
	dirtied: HashMap<P, u64>,
}

impl<P, D> Default for StreamCollector<P, D> {
	fn default() -> Self {
		Self { streams: HashMap::new(), dirtied: HashMap::new() }
	}
}

impl<P: Clone + Eq + Hash, D> StreamCollector<P, D> {
	pub fn start(&mut self, params: &P, tick: u64) {
		self.streams.insert(params.clone(), (tick, Vec::new()));
	}

	pub fn push(&mut self, params: &P, data: D) {
		if let Some((_, collected)) = self.streams.get_mut(params) {
			collected.push(data);
		}
	}

	pub fn dirty(&mut self, params: &P, last_clean_tick: u64) {
		let dirtied = self.dirtied.entry(params.clone()).or_default();
		*dirtied = (*dirtied).max(last_clean_tick);
	}

	/// The collected data, or None when the stream is stale or was never started.
	pub fn finish(&mut self, params: &P) -> Option<Vec<D>> {
		let (tick, collected) = self.streams.remove(params)?;
		if self.dirtied.get(params).is_some_and(|&dirtied| tick <= dirtied) { return None; }
		self.dirtied.remove(params);
		Some(collected)
	}

	pub fn forget(&mut self, params: &P) {
		self.streams.remove(params);
		self.dirtied.remove(params);
	}
}
