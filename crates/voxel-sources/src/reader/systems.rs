use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

use crate::source_manager::{Completed, SourceManager};
use crate::{ChunkChanged, ChunkLoaded, ChunkPresenceLoaded, VoxelAreaLoaded, VoxelAreaKey};

use super::VoxelReader;
use super::loader::{VoxelAreaLoadEvent, VoxelAreaLoadResult, VoxelCompletionTarget};

pub(crate) fn init_sources(manager: Res<SourceManager>) {
	manager.init_sources();
}

pub(crate) fn process_source_requests(mut reader: ResMut<VoxelReader>, manager: Res<SourceManager>) {
	reader.process_requests(&manager);
}

pub(crate) fn publish_source_messages(
	mut reader: ResMut<VoxelReader>,
	mut manager: ResMut<SourceManager>,
	mut chunk_changed: MessageWriter<ChunkChanged>,
	mut presence_loaded: MessageWriter<ChunkPresenceLoaded>,
	mut chunk_writer: MessageWriter<ChunkLoaded>,
	mut voxel_area_writer: MessageWriter<VoxelAreaLoaded>,
) {
	reader.process_requests(&manager);

	for change in manager.get_source_changes() {
		chunk_changed.write(change);
	}

	for completed in manager.get_completed() {
		match completed {
			Completed::PresenceLoaded { grid } => {
				if reader.complete_presence(grid) {
					presence_loaded.write(ChunkPresenceLoaded { grid });
				}
			}
			Completed::ChunkLoaded { grid, chunk, generation, voxels } => {
				if reader.complete_chunk(grid, chunk, generation) {
					chunk_writer.write(ChunkLoaded { grid, chunk, generation, voxels });
				}
			}
			Completed::VoxelsLoaded { grid, min, size, lod, voxel_type, generation, voxels } => {
				let requests = reader.complete_voxels(grid, min, size, lod, voxel_type, generation);
				for request in requests {
					match request.target {
						VoxelCompletionTarget::Message { requester, tag } => {
							voxel_area_writer.write(VoxelAreaLoaded {
								grid: request.request.grid,
								requester,
								key: VoxelAreaKey {
									min: request.request.key.min,
									size: request.request.key.size,
									lod: lod.max(0.0).floor() as u8,
								},
								voxel_type: request.request.voxel_type,
								tag,
								priority: request.request.priority,
								generation,
								voxels: voxels.clone(),
							});
						}
						VoxelCompletionTarget::Channel(sender) => {
							let _ = sender.unbounded_send(VoxelAreaLoadEvent::Loaded(VoxelAreaLoadResult {
								grid,
								key: request.request.key,
								voxel_type,
								generation,
								voxels: voxels.clone(),
							}));
						}
					}
				}
			}
		}
	}
}
