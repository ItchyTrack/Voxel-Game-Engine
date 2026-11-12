use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use bile_data::{DataDirtier, DataPasser, DataProducingNode, DirtyState, Request};
use bile_math::NonZeroRegion;
use bile_tiles::TileIndex;
use tile_data::{VoxelRegionResult, chunk_aligned};
use voxel_data::grid::GridId;

use crate::{
	ChunkEditInterest, ChunkEditInterestChanged, RequestId, SourceManager, SourceResult, SourceResultData,
	data::{VoxelData, VoxelPresence, VoxelRequest},
	edit::{GridEditIdManager, GridEditMessage},
};

enum NodeEvent<P> {
	Request(P),
	Drop(P),
}

#[derive(Default)]
struct VoxelLoad {
	in_flight: Option<RequestId>,
	results: Vec<VoxelRegionResult>,
}

#[derive(Default)]
struct PresenceLoad {
	in_flight: Option<RequestId>,
	regions: Vec<NonZeroRegion>,
}

/// The single Bile node that provides [`VoxelData`] and [`VoxelPresence`]. Requests fan out to every
/// registered [`crate::ChunkSource`] through the [`SourceManager`].
#[derive(Component, Default)]
pub struct VoxelSourceNode {
	voxel_events: Vec<NodeEvent<VoxelRequest>>,
	presence_events: Vec<NodeEvent<GridId>>,
	loads: HashMap<VoxelRequest, VoxelLoad>,
	loads_by_request: HashMap<RequestId, VoxelRequest>,
	loads_by_grid: HashMap<GridId, TileIndex<VoxelRequest>>,
	presence: HashMap<GridId, PresenceLoad>,
	presence_by_request: HashMap<RequestId, GridId>,
	edit_interest: HashMap<GridId, ChunkEditInterest>,
	queued_edit_interest: Vec<ChunkEditInterestChanged>,
}

impl DataProducingNode<VoxelData> for VoxelSourceNode {
	fn cost(&self, _request: &Request<VoxelData>) -> Option<f32> { Some(1.0) }

	fn request(&mut self, request: Request<VoxelData>) {
		self.voxel_events.push(NodeEvent::Request(request.params));
	}

	fn drop(&mut self, params: &VoxelRequest) {
		self.voxel_events.push(NodeEvent::Drop(*params));
	}
}

impl DataProducingNode<VoxelPresence> for VoxelSourceNode {
	fn cost(&self, _request: &Request<VoxelPresence>) -> Option<f32> { Some(1.0) }

	fn request(&mut self, request: Request<VoxelPresence>) {
		self.presence_events.push(NodeEvent::Request(request.params));
	}

	fn drop(&mut self, grid: &GridId) {
		self.presence_events.push(NodeEvent::Drop(*grid));
	}
}

impl VoxelSourceNode {
	pub fn edit_interest(&self, grid: GridId) -> Option<&ChunkEditInterest> {
		self.edit_interest.get(&grid)
	}

	fn fetch(&mut self, request: VoxelRequest, sources: &mut SourceManager, grids: &Query<&GridEditIdManager>) {
		let generation = grids.get(request.grid).map(|ids| ids.latest_generation()).unwrap_or_default();
		let request_id = sources.request_voxels(request.grid, request.area, request.lod, request.voxel_type, generation);
		self.loads_by_request.insert(request_id, request);
		let load = self.loads.get_mut(&request).expect("fetched voxels for an untracked request");
		load.results.clear();
		if let Some(old) = load.in_flight.replace(request_id) {
			self.loads_by_request.remove(&old);
			sources.cancel_voxels(old);
		}
	}

	fn start_voxel_load(&mut self, request: VoxelRequest) {
		self.loads.insert(request, VoxelLoad::default());
		self.loads_by_grid.entry(request.grid).or_default().insert(request);
		self.edit_interest.entry(request.grid).or_default().retain(request.area);
		self.queued_edit_interest.push(ChunkEditInterestChanged { grid: request.grid, region: request.area, interested: true });
	}

	fn stop_voxel_load(&mut self, request: VoxelRequest, sources: &mut SourceManager) {
		let Some(load) = self.loads.remove(&request) else { return };
		if let Some(request_id) = load.in_flight {
			self.loads_by_request.remove(&request_id);
			sources.cancel_voxels(request_id);
		}
		if let Some(index) = self.loads_by_grid.get_mut(&request.grid) {
			index.remove(request);
			if index.is_empty() { self.loads_by_grid.remove(&request.grid); }
		}
		if let Some(interest) = self.edit_interest.get_mut(&request.grid) && interest.release(request.area) {
			self.queued_edit_interest.push(ChunkEditInterestChanged { grid: request.grid, region: request.area, interested: false });
		}
	}
}

pub(crate) fn process_voxel_requests(
	node: Single<&mut VoxelSourceNode>,
	mut sources: ResMut<SourceManager>,
	grids: Query<&GridEditIdManager>,
) {
	let node = node.into_inner().into_inner();
	for event in std::mem::take(&mut node.voxel_events) {
		match event {
			NodeEvent::Request(request) => {
				match node.loads.get(&request) {
					Some(load) if load.in_flight.is_some() => continue,
					Some(_) => {}
					None => node.start_voxel_load(request),
				}
				node.fetch(request, &mut sources, &grids);
			}
			NodeEvent::Drop(request) => node.stop_voxel_load(request, &mut sources),
		}
	}
	for event in std::mem::take(&mut node.presence_events) {
		match event {
			NodeEvent::Request(grid) => {
				let load = node.presence.entry(grid).or_default();
				if load.in_flight.is_some() { continue; }
				load.regions.clear();
				let request_id = sources.request_presence(grid);
				load.in_flight = Some(request_id);
				node.presence_by_request.insert(request_id, grid);
			}
			NodeEvent::Drop(grid) => {
				let Some(load) = node.presence.remove(&grid) else { continue };
				if let Some(request_id) = load.in_flight {
					node.presence_by_request.remove(&request_id);
					sources.cancel_presence(request_id);
				}
			}
		}
	}
}

/// Edited voxels are fetched again when they are still loading, and dirtied for their consumers otherwise.
pub(crate) fn dirty_edited_voxels(
	node: Single<&mut VoxelSourceNode>,
	mut edits: MessageReader<GridEditMessage>,
	mut sources: ResMut<SourceManager>,
	grids: Query<&GridEditIdManager>,
	mut dirtier: DataDirtier<VoxelData>,
) {
	let node = node.into_inner().into_inner();
	for edit in edits.read() {
		let Some(index) = node.loads_by_grid.get(&edit.grid_id()) else { continue };
		let mut touched = Vec::new();
		index.for_each_overlapping(chunk_aligned(edit.edit().affected_region()), |request| touched.push(request));
		for request in touched {
			if node.loads[&request].in_flight.is_some() {
				node.fetch(request, &mut sources, &grids);
			} else {
				dirtier.mark_dirty(&request);
			}
		}
	}
}

pub(crate) fn receive_voxel_results(
	node: Single<(Entity, &mut VoxelSourceNode)>,
	mut results: MessageReader<SourceResult>,
	mut voxel_passer: DataPasser<VoxelData>,
	mut presence_passer: DataPasser<VoxelPresence>,
) {
	let (entity, node) = node.into_inner();
	let node = node.into_inner();
	for result in results.read() {
		match &result.data {
			SourceResultData::Voxels { region, lod, voxels, .. } => {
				let Some(request) = node.loads_by_request.get(&result.request_id) else { continue };
				let load = node.loads.get_mut(request).expect("routed voxel request lost its load");
				load.results.push(VoxelRegionResult { area: *region, lod: *lod, voxels: voxels.clone() });
			}
			SourceResultData::VoxelsLoaded { .. } => {
				let Some(request) = node.loads_by_request.remove(&result.request_id) else { continue };
				let load = node.loads.get_mut(&request).expect("routed voxel request lost its load");
				load.in_flight = None;
				let regions = Arc::new(std::mem::take(&mut load.results));
				voxel_passer.pass_data(entity, &request, DirtyState::UpToDate, regions);
			}
			SourceResultData::Presence { grid, region } => {
				let Some(&expected) = node.presence_by_request.get(&result.request_id) else { continue };
				if expected != *grid {
					warn!(request_id=?result.request_id, ?expected, actual=?grid, "ignoring mismatched source presence payload");
					continue;
				}
				node.presence.get_mut(grid).expect("routed presence request lost its load").regions.push(*region);
			}
			SourceResultData::PresenceLoaded => {
				let Some(grid) = node.presence_by_request.remove(&result.request_id) else { continue };
				let load = node.presence.get_mut(&grid).expect("routed presence request lost its load");
				load.in_flight = None;
				let regions = Arc::new(std::mem::take(&mut load.regions));
				presence_passer.pass_data(entity, &grid, DirtyState::UpToDate, regions);
			}
		}
	}
}

pub(crate) fn publish_edit_interest_changes(
	node: Single<&mut VoxelSourceNode>,
	mut events: MessageWriter<ChunkEditInterestChanged>,
) {
	let mut node = node.into_inner();
	if node.queued_edit_interest.is_empty() { return; }
	events.write_batch(std::mem::take(&mut node.queued_edit_interest));
}
