use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;
use voxel_tasks::{AsyncTaskPriorityQueueResource, PriorityTask};

use crate::source_manager::{Completed, SourceManager};
use crate::{ChunkChanged, ChunkLoaded, ChunkPresenceLoaded, TileLoaded, TileVoxelsLoaded, TileVoxelKey};

use super::VoxelReader;

pub(crate) fn init_sources(manager: Res<SourceManager>) {
	manager.init_sources();
}

pub(crate) fn process_source_requests(mut reader: ResMut<VoxelReader>, manager: Res<SourceManager>) {
	reader.process_requests(&manager);
}

pub(crate) fn publish_source_messages(
	mut reader: ResMut<VoxelReader>,
	mut manager: ResMut<SourceManager>,
	async_queue: Res<AsyncTaskPriorityQueueResource>,
	mut chunk_changed: MessageWriter<ChunkChanged>,
	mut presence_loaded: MessageWriter<ChunkPresenceLoaded>,
	mut chunk_writer: MessageWriter<ChunkLoaded>,
	mut tile_writer: MessageWriter<TileLoaded>,
	mut tile_voxels_writer: MessageWriter<TileVoxelsLoaded>,
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
					let Some(tile) = request.request.tile else {
						tile_voxels_writer.write(TileVoxelsLoaded {
							grid: request.request.grid,
							requester: request.request.requester,
							key: TileVoxelKey {
								min: request.request.key.min,
								size: request.request.key.size,
								lod: lod.max(0.0).floor() as u8,
							},
							voxel_type: request.request.voxel_type,
							tag: request.request.tag,
							priority: request.request.priority,
							generation,
							voxels: voxels.clone(),
						});
						continue;
					};

					let Some(voxels) = voxels.clone() else {
						tile_writer.write(TileLoaded {
							grid: request.request.grid,
							requester: request.request.requester,
							key: tile.key,
							tag: request.request.tag,
							generation,
							data: None,
						});
						continue;
					};

					let cancellation = request.cancellation;
					let priority = request.request.priority;
					let result_tx = reader.tile_sender();
					async_queue.pusher().push(PriorityTask::new(priority, async move {
						if cancellation.is_cancelled() { return; }
						let data = tile.generator.generate(tile_data::TileGeneratorInput {
							grid: request.request.grid,
							key: tile.key,
							voxel_lod: lod.max(0.0).floor() as u8,
							voxels,
						});
						if cancellation.is_cancelled() { return; }
						let _ = result_tx.send(TileLoaded {
							grid: request.request.grid,
							requester: request.request.requester,
							key: tile.key,
							tag: request.request.tag,
							generation,
							data,
						});
					}));
				}
			}
		}
	}

	for tile in reader.get_completed_tiles() {
		tile_writer.write(tile);
	}
}
