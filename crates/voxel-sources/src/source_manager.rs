use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender};
use rustc_hash::FxHashMap;
use tile_data::NonZeroChunkRegion;
use voxel_data::{grid::GridId, voxels::VoxelTypeId};
use voxel_tasks::CancellationToken;

use crate::{edit::GridGeneration, request::{RequestId, SourceResult}, source::{ChunkSource, SourceCoverage, SourceHandle, SourceId}};

#[derive(Resource)]
pub struct SourceManager {
	last_request_id: RequestId,
	pending_requests: FxHashMap<RequestId, (CancellationToken, u32)>,
	sources: Vec<Box<dyn ChunkSource>>,
	source_result_sender: Sender<SourceResult>,
	source_result_receiver: Receiver<SourceResult>,
}

impl Default for SourceManager {
	fn default() -> Self {
		let (source_result_sender, source_result_receiver) = crossbeam_channel::unbounded();
		Self {
			last_request_id: RequestId(0),
			pending_requests: Default::default(),
			sources: Vec::new(),
			source_result_sender,
			source_result_receiver,
		}
	}
}

impl SourceManager {
	fn new_request_id(&mut self) -> RequestId {
		let request_id = self.last_request_id;
		self.last_request_id.0 += 1;
		request_id
	}

	pub fn request_voxels(
		&mut self,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> RequestId {
		let request_id = self.new_request_id();
		let cancellation_token = CancellationToken::new();
		let mut source_request_count = 0;
		for source in &self.sources {
			match source.request_voxels(request_id, &cancellation_token, grid, region, lod, voxel_type, generation) {
				SourceCoverage::None => {},
				SourceCoverage::Some => { source_request_count += 1; },
				SourceCoverage::All => { source_request_count += 1; break; },
			};
		}
		self.pending_requests.insert(request_id, (cancellation_token, source_request_count));
		if source_request_count == 0 {
			let _ = self.source_result_sender.send(SourceResult {
				request_id,
				data: crate::request::SourceResultData::VoxelsLoaded { generation },
			});
		}
		request_id
	}

	pub fn cancel_voxels(&mut self, request_id: RequestId) {
		if let Some((cancellation_token, _)) = self.pending_requests.remove(&request_id) {
			cancellation_token.cancel();
		}
	}

	pub fn request_presence(
		&mut self,
		grid: GridId,
	) -> RequestId {
		let request_id = self.new_request_id();
		let cancellation_token = CancellationToken::new();
		let mut source_request_count = 0;
		for source in &self.sources {
			source_request_count += 1;
			source.request_presence(request_id, cancellation_token.clone(), grid);
		}
		self.pending_requests.insert(request_id, (cancellation_token, source_request_count));
		if source_request_count == 0 {
			let _ = self.source_result_sender.send(SourceResult {
				request_id,
				data: crate::request::SourceResultData::PresenceLoaded,
			});
		}
		request_id
	}

	pub fn cancel_presence(&mut self, request_id: RequestId) {
		if let Some((cancellation_token, _)) = self.pending_requests.remove(&request_id) {
			cancellation_token.cancel();
		}
	}

	pub fn transfer_ownership(&mut self, new_owner: SourceId, grid: GridId, region: NonZeroChunkRegion) {
		let Some(new_owner_source) = self.sources.get(new_owner.0) else {
			println!("transfer_ownership failed! Invalid new_owner {new_owner:?}");
			return;
		};
		new_owner_source.acquire_ownership(grid, region);
		for (source_id, source) in self.sources.iter().enumerate() {
			if source_id != new_owner.0 {
				source.relinquish_ownership(grid, region);
			}
		}
	}

	pub(crate) fn add_source<S: ChunkSource + 'static>(&mut self, source: S) {
		assert!(self.get_source::<S>().is_none()); // only allow one of each source type. 2 of the same type should just both use the same source...
		source.init(SourceHandle { id: SourceId(self.sources.len()), messages: self.source_result_sender.clone() });
		self.sources.push(Box::new(source));
	}

	pub fn get_source<S: ChunkSource + 'static>(&self) -> Option<&S> {
		for source in &self.sources {
			if let Some(source) = source.as_any().downcast_ref::<S>() {
				return Some(source);
			}
		};
		None
	}

	pub fn get_source_mut<S: ChunkSource + 'static>(&mut self) -> Option<&mut S> {
		for source in &mut self.sources {
			if let Some(source) = source.as_any_mut().downcast_mut::<S>() {
				return Some(source);
			}
		};
		None
	}
}

pub(crate) fn publish_source_results_messages(
	mut source_manager: ResMut<SourceManager>,
	mut result_writter: MessageWriter<SourceResult>,
) {
	result_writter.write_batch(source_manager.source_result_receiver.clone().try_iter().filter_map(|result| {
		let (_, source_request_count) = source_manager.pending_requests.get_mut(&result.request_id)?;
		match result.data {
			crate::request::SourceResultData::PresenceLoaded | crate::request::SourceResultData::VoxelsLoaded { .. } => {
				if *source_request_count > 1 {
					*source_request_count -= 1;
					return None;
				}
				source_manager.pending_requests.remove(&result.request_id);
			},
			_ => {}
		}
		Some(result)
	}));
}
