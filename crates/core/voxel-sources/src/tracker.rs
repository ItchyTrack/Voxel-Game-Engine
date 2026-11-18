use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bile_data::{
	DirtyState,
	gather,
	streaming::{self, StreamId},
};
use bile_tiles::TileIndex;
use tile_data::chunk_aligned;
use voxel_data::grid::GridId;

use crate::{
	ChunkEditInterest, ChunkEditInterestChanged,
	data::{VoxelData, VoxelPresence, VoxelRequest},
	edit::GridEditMessage,
};

enum Tracked {
	Voxels(VoxelRequest, StreamId),
	Presence(GridId, StreamId),
}

/// A gather node that is part of every [`VoxelData`] and [`VoxelPresence`] stream. It streams nothing,
/// so a request no source covers still finishes. It knows every voxel request that is in use, so it
/// dirties them when their voxels are edited and keeps the [`ChunkEditInterest`] of each grid.
#[derive(Component, Default)]
pub struct VoxelRequestTracker {
	to_finish: Vec<Tracked>,
	loads: HashSet<VoxelRequest>,
	loads_by_grid: HashMap<GridId, TileIndex<VoxelRequest>>,
	edit_interest: HashMap<GridId, ChunkEditInterest>,
	queued_edit_interest: Vec<ChunkEditInterestChanged>,
}

impl VoxelRequestTracker {
	pub fn edit_interest(&self, grid: GridId) -> Option<&ChunkEditInterest> {
		self.edit_interest.get(&grid)
	}
}

impl gather::DataProducingNode<VoxelData> for VoxelRequestTracker {
	fn cost(&self, _request: &streaming::Request<VoxelData>) -> Option<f32> { Some(0.0) }

	fn request(&mut self, request: streaming::Request<VoxelData>, stream_id: StreamId) {
		let request = request.params;
		self.to_finish.push(Tracked::Voxels(request, stream_id));
		if !self.loads.insert(request) { return; }
		self.loads_by_grid.entry(request.grid).or_default().insert(request);
		self.edit_interest.entry(request.grid).or_default().retain(request.area);
		self.queued_edit_interest.push(ChunkEditInterestChanged { grid: request.grid, region: request.area, interested: true });
	}

	fn drop(&mut self, request: &VoxelRequest) {
		if !self.loads.remove(request) { return; }
		if let Some(index) = self.loads_by_grid.get_mut(&request.grid) {
			index.remove(*request);
			if index.is_empty() { self.loads_by_grid.remove(&request.grid); }
		}
		if let Some(interest) = self.edit_interest.get_mut(&request.grid) && interest.release(request.area) {
			self.queued_edit_interest.push(ChunkEditInterestChanged { grid: request.grid, region: request.area, interested: false });
		}
	}
}

impl gather::DataProducingNode<VoxelPresence> for VoxelRequestTracker {
	fn cost(&self, _request: &streaming::Request<VoxelPresence>) -> Option<f32> { Some(0.0) }

	fn request(&mut self, request: streaming::Request<VoxelPresence>, stream_id: StreamId) {
		self.to_finish.push(Tracked::Presence(request.params, stream_id));
	}

	fn drop(&mut self, _grid: &GridId) {}
}

pub(crate) fn finish_tracker_streams(
	tracker: Single<(Entity, &mut VoxelRequestTracker)>,
	mut voxel_passer: streaming::DataPasser<VoxelData>,
	mut presence_passer: streaming::DataPasser<VoxelPresence>,
) {
	let (entity, mut tracker) = tracker.into_inner();
	for tracked in std::mem::take(&mut tracker.to_finish) {
		match tracked {
			Tracked::Voxels(request, stream_id) => {
				voxel_passer.start(entity, &request, stream_id, DirtyState::UpToDate);
				voxel_passer.finish(entity, &request, stream_id);
			}
			Tracked::Presence(grid, stream_id) => {
				presence_passer.start(entity, &grid, stream_id, DirtyState::UpToDate);
				presence_passer.finish(entity, &grid, stream_id);
			}
		}
	}
}

/// Edited voxels are dirtied with the edit as the diff. Their consumers request them again.
pub(crate) fn dirty_edited_voxels(
	tracker: Single<&VoxelRequestTracker>,
	mut edits: MessageReader<GridEditMessage>,
	mut dirtier: streaming::DataDirtier<VoxelData>,
) {
	for edit in edits.read() {
		let Some(index) = tracker.loads_by_grid.get(&edit.grid_id()) else { continue };
		let mut touched = Vec::new();
		index.for_each_overlapping(chunk_aligned(edit.edit().affected_region()), |request| touched.push(request));
		for request in touched {
			dirtier.mark_dirty(&request, Some(edit.shared_edit()));
		}
	}
}

pub(crate) fn publish_edit_interest_changes(
	mut tracker: Single<&mut VoxelRequestTracker>,
	mut events: MessageWriter<ChunkEditInterestChanged>,
) {
	if tracker.queued_edit_interest.is_empty() { return; }
	events.write_batch(std::mem::take(&mut tracker.queued_edit_interest));
}
