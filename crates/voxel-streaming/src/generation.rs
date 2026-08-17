use std::{collections::HashSet, sync::{Arc, Mutex}};

use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender, unbounded};
use futures::{StreamExt, channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded as async_unbounded}};
use tile_data::{
	GenerationVoxelReader, TileData, TileGenerationParameters, TileGenerationSession, TileKey,
	VoxelAreaRequest, VoxelAreaResult,
};
use voxel_data::grid::GridId;
use voxel_tasks::CancellationToken;

use crate::tile_dependency_index::TileDependency;

#[derive(Default)]
pub(crate) struct TileGenerationMetadata {
	pub(crate) dependencies: HashSet<TileDependency>,
}

pub(crate) struct StreamingVoxelReader {
	grid: GridId,
	priority: f32,
	requests: VoxelSourcesRequestHandle,
	events_tx: UnboundedSender<VoxelAreaLoadEvent>,
	events_rx: UnboundedReceiver<VoxelAreaLoadEvent>,
	outstanding: usize,
	cancellation: CancellationToken,
	metadata: Arc<Mutex<TileGenerationMetadata>>,
}

impl StreamingVoxelReader {
	pub(crate) fn new(
		grid: GridId,
		priority: f32,
		requests: VoxelSourcesRequestHandle,
		cancellation: CancellationToken,
		metadata: Arc<Mutex<TileGenerationMetadata>>,
	) -> (Self, UnboundedSender<VoxelAreaLoadEvent>) {
		let (events_tx, events_rx) = async_unbounded();
		(
			Self {
				grid,
				priority,
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
	fn request_voxels(&mut self, request: VoxelAreaRequest) {
		assert!(request.area.size().cmpgt(UVec3::ZERO).all(), "voxel area request must have positive size");
		if self.cancellation.is_cancelled() { return; }
		self.outstanding += 1;
		let _cancellation = self.requests.request_voxels(
			VoxelAreaLoadRequest {
				grid: self.grid,
				key: VoxelAreaKey {
					region: request.area,
					lod: request.lod,
				},
				voxel_type: request.voxel_type,
				priority: self.priority,
			},
			self.events_tx.clone(),
		);
	}

	fn receive_voxels(&mut self) -> tile_data::ReceiveVoxelsFuture<'_> {
		Box::pin(async move {
			while self.outstanding > 0 {
				let event = self.events_rx.next().await.expect("voxel result channel closed with requests outstanding");
				match event {
					VoxelAreaLoadEvent::Cancelled => {
						self.outstanding = 0;
						return None;
					}
					VoxelAreaLoadEvent::Loaded(result) => {
						self.outstanding -= 1;
						let mut metadata = self.metadata.lock().unwrap();
						metadata.dependencies.insert(TileDependency {
							area: result.key.region,
							generation: result.generation,
						});
						drop(metadata);

						if let Some(voxels) = result.voxels {
							return Some(VoxelAreaResult {
								area: result.key.region,
								lod: result.key.lod,
								voxels,
							});
						}
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
	wake: UnboundedSender<VoxelAreaLoadEvent>,
}

impl TileGenerationCancellation {
	pub(crate) fn new(token: CancellationToken, wake: UnboundedSender<VoxelAreaLoadEvent>) -> Self {
		Self { token, wake }
	}

	pub(crate) fn cancel(self) {
		self.token.cancel();
		let _ = self.wake.unbounded_send(VoxelAreaLoadEvent::Cancelled);
	}
}

pub(crate) struct TileGenerationResult {
	pub(crate) grid: GridId,
	pub(crate) tag: u64,
	pub(crate) context_version: u64,
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
	priority: f32,
	requests: VoxelSourcesRequestHandle,
	cancellation: CancellationToken,
	metadata: Arc<Mutex<TileGenerationMetadata>>,
) -> (TileGenerationSession, UnboundedSender<VoxelAreaLoadEvent>) {
	let (reader, wake) = StreamingVoxelReader::new(grid, priority, requests, cancellation, metadata);
	(TileGenerationSession::new(grid, key, context, Box::new(reader)), wake)
}
