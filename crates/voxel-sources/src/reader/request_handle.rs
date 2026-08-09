use std::sync::atomic::{AtomicU64, Ordering};

use bevy::prelude::*;
use crossbeam_channel::Sender;
use futures::channel::mpsc::UnboundedSender;
use voxel_tasks::CancellationToken;

use super::VoxelReader;
use super::loader::{
	ChunkLoadRequest, PresenceLoadRequest, ReaderRequest, VoxelAreaCancellation,
	VoxelAreaMessageRequest, VoxelAreaLoadEvent, VoxelAreaLoadRequest, VoxelCompletionTarget,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct VoxelSourceRequesterId(pub(crate) u64);

/// Information needed to allocate distinct voxel-source requester handles.
#[derive(Resource)]
pub struct VoxelSourcesRequestHandleGetter {
	sender: Sender<ReaderRequest>,
	next_id: AtomicU64,
}

impl FromWorld for VoxelSourcesRequestHandleGetter {
	fn from_world(world: &mut World) -> Self {
		Self {
			sender: world.resource::<VoxelReader>().request_sender(),
			next_id: AtomicU64::new(1),
		}
	}
}

impl VoxelSourcesRequestHandleGetter {
	pub fn get(&self) -> VoxelSourcesRequestHandle {
		VoxelSourcesRequestHandle {
			id: VoxelSourceRequesterId(self.next_id.fetch_add(1, Ordering::Relaxed)),
			sender: self.sender.clone(),
		}
	}
}

/// RAII requester identity and convenient access to voxel-source request functions.
pub struct VoxelSourcesRequestHandle {
	id: VoxelSourceRequesterId,
	sender: Sender<ReaderRequest>,
}

impl VoxelSourcesRequestHandle {
	pub fn request_presence(&self, request: PresenceLoadRequest) {
		let _ = self.sender.send(ReaderRequest::Presence { requester: self.id, request });
	}

	pub fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken {
		let cancellation = CancellationToken::new();
		let _ = self.sender.send(ReaderRequest::Chunk {
			requester: self.id,
			request,
			cancellation: cancellation.clone(),
		});
		cancellation
	}

	pub fn request_voxel_area(&self, request: VoxelAreaMessageRequest) -> VoxelAreaCancellation {
		let target = VoxelCompletionTarget::Message { requester: request.requester, tag: request.tag };
		self.request_voxels_to(request.area_request(), target)
	}

	pub fn request_voxels(
		&self,
		request: VoxelAreaLoadRequest,
		results: UnboundedSender<VoxelAreaLoadEvent>,
	) -> VoxelAreaCancellation {
		self.request_voxels_to(request, VoxelCompletionTarget::Channel(results))
	}

	fn request_voxels_to(&self, request: VoxelAreaLoadRequest, target: VoxelCompletionTarget) -> VoxelAreaCancellation {
		let cancellation = CancellationToken::new();
		let result = VoxelAreaCancellation::new(
			self.id,
			request.grid,
			request.key,
			request.voxel_type,
			cancellation.clone(),
			self.sender.clone(),
		);
		let _ = self.sender.send(ReaderRequest::VoxelArea {
			requester: self.id,
			request,
			cancellation,
			target,
		});
		result
	}
}

impl Drop for VoxelSourcesRequestHandle {
	fn drop(&mut self) {
		let _ = self.sender.send(ReaderRequest::ReleaseRequester { requester: self.id });
	}
}
