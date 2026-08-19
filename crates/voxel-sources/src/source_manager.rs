use std::sync::Arc;

use bevy::prelude::*;
use crossbeam_channel::{Receiver, Sender};
use rustc_hash::FxHashMap;
use tile_data::NonZeroChunkRegion;
use voxel_data::{grid::GridId, voxels::VoxelTypeId};
use voxel_tasks::CancellationToken;

use crate::{request::{RequestId, SourceResult}, source::{ChunkSource, SourceCoverage, SourceHandle, SourceId}};

#[derive(Resource)]
pub struct SourceManager {
	last_request_id: RequestId,
	pending_requests: FxHashMap<RequestId, (CancellationToken, u32)>,
	sources: Vec<Arc<dyn ChunkSource>>,
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
	) -> RequestId {
		let request_id = self.new_request_id();
		let cancellation_token = CancellationToken::new();
		let mut source_request_count = 0;
		for source in &self.sources {
			match source.request_voxels(request_id, &cancellation_token, grid, region, lod, voxel_type) {
				SourceCoverage::None => {},
				SourceCoverage::Some => { source_request_count += 1; },
				SourceCoverage::All => { source_request_count += 1; break; },
			};
		}
		self.pending_requests.insert(request_id, (cancellation_token, source_request_count));
		if source_request_count == 0 {
			let _ = self.source_result_sender.send(SourceResult {
				request_id,
				data: crate::request::SourceResultData::VoxelsLoaded,
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
		request_id
	}

	pub fn cancel_presence(&mut self, request_id: RequestId) {
		if let Some((cancellation_token, _)) = self.pending_requests.remove(&request_id) {
			cancellation_token.cancel();
		}
	}

	// forces every source but new_owner to forget that region. Does not call anything on new_owner so setting ownership on it is the responsibility of the user
	pub fn transfer_onwership(&mut self, new_owner: SourceId, grid: GridId, region: NonZeroChunkRegion) {
		if new_owner.0 >= self.sources.len() { println!("transfer_onwership failed! Invalid new_owner {new_owner:?}"); return; }
		for (source_id, source) in self.sources.iter().enumerate() {
			if source_id != new_owner.0 {
				source.take_ownership(grid, region);
			}
		}
	}

	pub(crate) fn add_source<S: ChunkSource + 'static>(&mut self, source: S) {
		source.init(SourceHandle { id: SourceId(self.sources.len()), messages: self.source_result_sender.clone() });
		self.sources.push(Arc::new(source));
	}
}

pub(crate) fn publish_source_results_messages(
	mut source_manager: ResMut<SourceManager>,
	mut result_writter: MessageWriter<SourceResult>,
) {
	result_writter.write_batch(source_manager.source_result_receiver.clone().try_iter().filter_map(|result| {
		let (_, source_request_count) = source_manager.pending_requests.get_mut(&result.request_id)?;
		match result.data {
			crate::request::SourceResultData::PresenceLoaded | crate::request::SourceResultData::VoxelsLoaded => {
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
