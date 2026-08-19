use std::{collections::{HashMap, HashSet}, sync::{Arc, Mutex}};

use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, unbounded};
use futures::{StreamExt, channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded as async_unbounded}};
use tile_data::{
	GenerationVoxelReader, TileData, TileGenerationParameters, TileGenerationSession, TileKey,
	VoxelRegionRequest, VoxelRegionResult,
};
use voxel_data::{
	grid::GridId,
	grid_tree::GridType,
};
use voxel_sources::{RequestId, SourceManager, SourceResult, SourceResultData};
use voxel_tasks::CancellationToken;

use crate::tile_dependency_index::TileDependency;

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct GenerationCell;

impl GridType for GenerationCell {
	type Data<'a> = u64;
	const MAX_NODE_OFFSET: u32 = u32::MAX;

	fn data_size_bytes(&self) -> usize { std::mem::size_of::<u64>() }

	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a> {
		u64::from_le_bytes(bytes[..8].try_into().expect("generation bytes"))
	}

	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]) {
		bytes[..8].copy_from_slice(&data.to_le_bytes());
	}

	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool {
		bytes[..8] == data.to_le_bytes()
	}
}

#[derive(Default)]
pub(crate) struct TileGenerationMetadata {
	pub(crate) dependencies: HashSet<TileDependency>,
}

enum VoxelLoadEvent {
	Result { result: VoxelRegionResult, generation: u64 },
	Loaded,
	Cancelled,
}

pub(crate) struct PendingVoxelRequest {
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
	request_tx: Sender<PendingVoxelRequest>,
	request_rx: Receiver<PendingVoxelRequest>,
	routes: HashMap<RequestId, VoxelRequestRoute>,
}

impl Default for TileVoxelSourceBridge {
	fn default() -> Self {
		let (request_tx, request_rx) = unbounded();
		Self { request_tx, request_rx, routes: HashMap::new() }
	}
}

impl TileVoxelSourceBridge {
	fn sender(&self) -> Sender<PendingVoxelRequest> { self.request_tx.clone() }
}

pub(crate) struct StreamingVoxelReader {
	grid: GridId,
	requests: Sender<PendingVoxelRequest>,
	events_tx: UnboundedSender<VoxelLoadEvent>,
	events_rx: UnboundedReceiver<VoxelLoadEvent>,
	outstanding: usize,
	cancellation: CancellationToken,
	metadata: Arc<Mutex<TileGenerationMetadata>>,
}

impl StreamingVoxelReader {
	fn new(
		grid: GridId,
		requests: Sender<PendingVoxelRequest>,
		cancellation: CancellationToken,
		metadata: Arc<Mutex<TileGenerationMetadata>>,
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

impl GenerationVoxelReader for StreamingVoxelReader {
	fn request_voxels(&mut self, request: VoxelRegionRequest) {
		if self.cancellation.is_cancelled() { return; }
		self.outstanding = self.outstanding.checked_add(1).expect("tile voxel request count overflow");
		let pending = PendingVoxelRequest {
			request,
			grid: self.grid,
			events: self.events_tx.clone(),
			cancellation: self.cancellation.clone(),
		};
		if self.requests.send(pending).is_err() {
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
					VoxelLoadEvent::Result { result, generation } => {
						self.metadata.lock().unwrap().dependencies.insert(TileDependency {
							area: result.area,
							generation,
						});
						return Some(result);
					}
				}
			}
			None
		})
	}
}

#[derive(Debug)]
pub(crate) struct TileGenerationCancellation {
	token: CancellationToken,
	wake: UnboundedSender<VoxelLoadEvent>,
}

impl TileGenerationCancellation {
	fn new(token: CancellationToken, wake: UnboundedSender<VoxelLoadEvent>) -> Self {
		Self { token, wake }
	}

	pub(crate) fn cancel(self) {
		self.token.cancel();
		let _ = self.wake.unbounded_send(VoxelLoadEvent::Cancelled);
	}
}

pub(crate) struct TileGenerationResult {
	pub(crate) grid: GridId,
	pub(crate) tag: u64,
	pub(crate) context: TileGenerationParameters,
	pub(crate) dependencies: HashSet<TileDependency>,
	pub(crate) data: Option<Box<dyn TileData>>,
}

#[derive(Resource)]
pub(crate) struct TileGenerationChannel {
	tx: Sender<TileGenerationResult>,
	rx: Receiver<TileGenerationResult>,
}

impl Default for TileGenerationChannel {
	fn default() -> Self {
		let (tx, rx) = unbounded();
		Self { tx, rx }
	}
}

impl TileGenerationChannel {
	pub(crate) fn sender(&self) -> Sender<TileGenerationResult> { self.tx.clone() }
	pub(crate) fn drain(&self) -> impl Iterator<Item = TileGenerationResult> + '_ { self.rx.try_iter() }
}

pub(crate) fn session(
	grid: GridId,
	key: TileKey,
	context: TileGenerationParameters,
	requests: Sender<PendingVoxelRequest>,
	cancellation: CancellationToken,
	metadata: Arc<Mutex<TileGenerationMetadata>>,
) -> (TileGenerationSession, TileGenerationCancellation) {
	let (reader, wake) = StreamingVoxelReader::new(grid, requests, cancellation.clone(), metadata);
	(
		TileGenerationSession::new(grid, key, context, Box::new(reader)),
		TileGenerationCancellation::new(cancellation, wake),
	)
}

pub(crate) fn submit_tile_voxel_requests(
	mut bridge: ResMut<TileVoxelSourceBridge>,
	mut sources: ResMut<SourceManager>,
) {
	let pending: Vec<_> = bridge.request_rx.try_iter().collect();
	for request in pending {
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
	mut grids: Query<&mut crate::GridStreaming>,
) {
	for result in results.read() {
		let Some(route) = bridge.routes.get(&result.request_id) else { continue };
		if route.cancellation.is_cancelled() { continue; }
		match &result.data {
			SourceResultData::Voxels { grid, region, lod, generation, voxels } => {
				if let Ok(mut streaming) = grids.get_mut(*grid) {
					streaming.note_source_generation(*region, *generation);
					streaming.dirty_stale_tiles(*region, *generation);
				}
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

pub(crate) fn bridge_sender(bridge: &TileVoxelSourceBridge) -> Sender<PendingVoxelRequest> {
	bridge.sender()
}
