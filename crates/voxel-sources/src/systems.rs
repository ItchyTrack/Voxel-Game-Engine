use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

use crate::handle::{SourceHandle, SourceMessage};
use crate::registry::SourceRegistry;
use crate::source::SourceId;
use crate::{ChunkChangeKind, ChunkChanged, ChunkLoaded, ChunkPresenceLoaded, LodLoaded, LodKey, SourceEvent};

pub(crate) fn init_sources(registry: ResMut<SourceRegistry>) {
	let messages = registry.message_tx.clone();
	for (i, source) in registry.sources.iter().enumerate() {
		source.init(SourceHandle {
			id: SourceId(i),
			messages: messages.clone(),
		});
	}
}

pub(crate) fn publish_source_messages(
	registry: Res<SourceRegistry>,
	mut source_events: MessageWriter<SourceEvent>,
	mut chunk_changed: MessageWriter<ChunkChanged>,
	mut presence_loaded: MessageWriter<ChunkPresenceLoaded>,
	mut chunk_writer: MessageWriter<ChunkLoaded>,
	mut lod_writer: MessageWriter<LodLoaded>,
) {
	while let Some(message) = registry.try_recv_message() {
		match message {
			SourceMessage::Event(event) => {
				match event {
					SourceEvent::Claim { source, grid, min, size } => {
						registry.forget_others(source, grid, min, size);
						chunk_changed.write(ChunkChanged {
							grid,
							min,
							size,
							kind: ChunkChangeKind::Changed,
							from_save: false,
						});
					}
					SourceEvent::Unavailable { grid, min, size } => {
						chunk_changed.write(ChunkChanged {
							grid,
							min,
							size,
							kind: ChunkChangeKind::Removed,
							from_save: false,
						});
					}
				}
				source_events.write(event);
			}
			SourceMessage::PresenceLoaded(event) => {
				if registry.finish_presence_load(event.grid) {
					presence_loaded.write(event);
				}
			}
			SourceMessage::Chunk(result) => {
				chunk_writer.write(ChunkLoaded { grid: result.grid, chunk: result.chunk, voxels: result.voxels });
			}
			SourceMessage::Lod(result) => {
				let Some((requests, lod, voxels)) = registry.take_pending_lod_completion(result) else { continue };
				for request in requests {
					lod_writer.write(LodLoaded {
						grid: request.grid,
						requester: request.requester,
						key: LodKey { min: request.key.min, size: request.key.size, lod: lod.max(0.0).floor() as u8 },
						priority: request.priority,
						generation: request.generation,
						voxels: voxels.clone(),
						entity: None,
					});
				}
			}
		}
	}
}
