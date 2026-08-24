use std::{collections::{HashMap, HashSet}, sync::{Arc, Mutex}};

use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, unbounded};
use futures::{StreamExt, channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded as async_unbounded}};
use tile_data::{
	TileBuildingVoxelReader, TileData, TileBuildingParameters, TileBuildingSession, TileKey,
	VoxelRegionRequest, VoxelRegionResult,
};
use voxel_data::grid::GridId;
use voxel_sources::{RequestId, SourceManager, SourceResult, SourceResultData, edit::GridChunkGeneration};
use voxel_tasks::CancellationToken;

use crate::tile_dependency_index::TileDependency;

#[derive(Default)]
pub(crate) struct TileBuildingMetadata {
	pub(crate) dependencies: HashSet<TileDependency>,
}

enum VoxelLoadEvent {
	Result { result: VoxelRegionResult, generation: u64 },
	Loaded,
	Cancelled,
}

pub(crate) struct TileBuildingVoxelRequest {
	request: VoxelRegionRequest,
	grid: GridId,
	events: UnboundedSender<VoxelLoadEvent>,
	cancellation: CancellationToken,
}

struct VoxelRequestRoute {
	events: UnboundedSender<VoxelLoadEvent>,
	cancellation: CancellationToken,
}

#[derive(Resource)]
pub(crate) struct TileVoxelSourceBridge {
	request_tx: Sender<TileBuildingVoxelRequest>,
	request_rx: Receiver<TileBuildingVoxelRequest>,
	routes: HashMap<RequestId, VoxelRequestRoute>,
}

impl Default for TileVoxelSourceBridge {
	fn default() -> Self {
		let (request_tx, request_rx) = unbounded();
		Self { request_tx, request_rx, routes: HashMap::new() }
	}
}

impl TileVoxelSourceBridge {
	pub(crate) fn sender(&self) -> Sender<TileBuildingVoxelRequest> { self.request_tx.clone() }
}

pub(crate) struct StreamingVoxelReader {
	grid: GridId,
	requests: Sender<TileBuildingVoxelRequest>,
	events_tx: UnboundedSender<VoxelLoadEvent>,
	events_rx: UnboundedReceiver<VoxelLoadEvent>,
	outstanding: usize,
	cancellation: CancellationToken,
	metadata: Arc<Mutex<TileBuildingMetadata>>,
}

impl StreamingVoxelReader {
	fn new(
		grid: GridId,
		requests: Sender<TileBuildingVoxelRequest>,
		cancellation: CancellationToken,
		metadata: Arc<Mutex<TileBuildingMetadata>>,
	) -> (Self, UnboundedSender<VoxelLoadEvent>) {
		let (events_tx, events_rx) = async_unbounded();
		(
			Self {
				grid,
				requests,
				events_tx: events_tx.clone(),
				events_rx,
				outstanding: 0,
				cancellation,
				metadata,
			},
			events_tx,
		)
	}
}

impl TileBuildingVoxelReader for StreamingVoxelReader {
	fn request_voxels(&mut self, request: VoxelRegionRequest) {
		if self.cancellation.is_cancelled() { return; }
		self.outstanding = self.outstanding.checked_add(1).expect("tile voxel request count overflow");
		if self.requests.send(TileBuildingVoxelRequest {
			request,
			grid: self.grid,
			events: self.events_tx.clone(),
			cancellation: self.cancellation.clone(),
		}).is_err() {
			self.outstanding -= 1;
		}
	}

	fn receive_voxels(&mut self) -> tile_data::ReceiveVoxelsFuture<'_> {
		Box::pin(async move {
			while self.outstanding > 0 {
				let event = self.events_rx.next().await.expect("voxel result channel closed with requests outstanding");
				match event {
					VoxelLoadEvent::Cancelled => {
						self.outstanding = 0;
						return None;
					}
					VoxelLoadEvent::Loaded => self.outstanding -= 1,
					VoxelLoadEvent::Result { result, generation: _ } => {
						// self.metadata.lock().unwrap().dependencies.insert(TileDependency {
						// 	region: result.area,
						// 	generation,
						// });
						return Some(result);
					}
				}
			}
			None
		})
	}
}

#[derive(Debug)]
pub(crate) struct TileBuildingCancellationToken {
	token: CancellationToken,
	wake: UnboundedSender<VoxelLoadEvent>,
}

impl TileBuildingCancellationToken {
	fn new(token: CancellationToken, wake: UnboundedSender<VoxelLoadEvent>) -> Self {
		Self { token, wake }
	}

	pub(crate) fn cancel(self) {
		self.token.cancel();
		let _ = self.wake.unbounded_send(VoxelLoadEvent::Cancelled);
	}
}

pub(crate) struct TileBuildingResult {
	pub(crate) grid: GridId,
	pub(crate) tile_key: TileKey,
	pub(crate) generation: GridChunkGeneration, // The lowest read result generation
	pub(crate) context: TileBuildingParameters,
	pub(crate) dependencies: HashSet<TileDependency>,
	pub(crate) data: Option<Box<dyn TileData>>,
}

#[derive(Resource)]
pub(crate) struct TileBuildingChannel {
	tx: Sender<TileBuildingResult>,
	rx: Receiver<TileBuildingResult>,
}

impl Default for TileBuildingChannel {
	fn default() -> Self {
		let (tx, rx) = unbounded();
		Self { tx, rx }
	}
}

impl TileBuildingChannel {
	pub(crate) fn sender(&self) -> Sender<TileBuildingResult> { self.tx.clone() }
	pub(crate) fn drain(&self) -> impl Iterator<Item = TileBuildingResult> + '_ { self.rx.try_iter() }
}

pub(crate) fn session(
	grid: GridId,
	key: TileKey,
	context: TileBuildingParameters,
	requests: Sender<TileBuildingVoxelRequest>,
	cancellation: CancellationToken,
	metadata: Arc<Mutex<TileBuildingMetadata>>,
) -> (TileBuildingSession, TileBuildingCancellationToken) {
	let (reader, wake) = StreamingVoxelReader::new(grid, requests, cancellation.clone(), metadata);
	(
		TileBuildingSession::new(grid, key, context, Box::new(reader)),
		TileBuildingCancellationToken::new(cancellation, wake),
	)
}

pub(crate) fn submit_tile_voxel_requests(
	mut bridge: ResMut<TileVoxelSourceBridge>,
	mut sources: ResMut<SourceManager>,
) {
	for request in bridge.request_rx.try_iter().collect::<Vec<_>>() {
		if request.cancellation.is_cancelled() { continue; }
		let request_id = sources.request_voxels(
			request.grid,
			request.request.area,
			request.request.lod,
			Some(request.request.voxel_type),
		);
		bridge.routes.insert(request_id, VoxelRequestRoute {
			events: request.events,
			cancellation: request.cancellation,
		});
	}

	let cancelled: Vec<_> = bridge.routes.iter()
		.filter_map(|(request_id, route)| route.cancellation.is_cancelled().then_some(*request_id))
		.collect();
	for request_id in cancelled {
		bridge.routes.remove(&request_id);
		sources.cancel_voxels(request_id);
	}
}

pub(crate) fn route_tile_source_results(
	mut bridge: ResMut<TileVoxelSourceBridge>,
	mut results: MessageReader<SourceResult>,
) {
	for result in results.read() {
		let Some(route) = bridge.routes.get(&result.request_id) else { continue };
		if route.cancellation.is_cancelled() { continue; }
		match &result.data {
			SourceResultData::Voxels { grid: _, region, lod, generation, voxels } => {
				let _ = route.events.unbounded_send(VoxelLoadEvent::Result {
					result: VoxelRegionResult { area: *region, lod: *lod, voxels: voxels.clone() },
					generation: *generation,
				});
			}
			SourceResultData::VoxelsLoaded => {
				let route = bridge.routes.remove(&result.request_id).unwrap();
				let _ = route.events.unbounded_send(VoxelLoadEvent::Loaded);
			}
			SourceResultData::Presence { .. } | SourceResultData::PresenceLoaded => {}
		}
	}
}
