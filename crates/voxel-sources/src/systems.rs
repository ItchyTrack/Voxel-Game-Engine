use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

use crate::handle::{SourceHandle, SourceMessage};
use crate::registry::SourceRegistry;
use crate::source::SourceId;
use crate::{ChunkChangeKind, ChunkChanged, ChunkLoaded, ChunkPresenceLoaded, LodLoaded, LodKey, SourceEvent};

pub(crate) fn init_sources(registry: ResMut<SourceRegistry>) {
	let messages = registry.message_tx.clone();
	let lod_generators = registry.generators.clone();
	for (i, source) in registry.sources.iter().enumerate() {
		source.init(SourceHandle {
			id: SourceId(i),
			messages: messages.clone(),
			lod_generators: lod_generators.clone(),
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
						let generation = registry.next_generation(grid);
						chunk_changed.write(ChunkChanged {
							grid,
							min,
							size,
							kind: ChunkChangeKind::Changed { generation },
							from_save: false,
						});
					}
					SourceEvent::Unavailable { grid, min, size } => {
						let generation = registry.next_generation(grid);
						chunk_changed.write(ChunkChanged {
							grid,
							min,
							size,
							kind: ChunkChangeKind::Removed { generation },
							from_save: false,
						});
					}
				}
				source_events.write(event);
			}
			SourceMessage::ChunkChanged(event) => {
				chunk_changed.write(event);
			}
			SourceMessage::PresenceLoaded(event) => {
				if registry.finish_presence_load(event.grid) {
					presence_loaded.write(event);
				}
			}
			SourceMessage::Chunk(result) => {
				if registry.complete_chunk(result.grid, result.chunk, result.generation) {
					chunk_writer.write(ChunkLoaded { grid: result.grid, chunk: result.chunk, generation: result.generation, voxels: result.voxels });
				}
			}
			SourceMessage::Lod(result) => {
				let Some((requests, lod, voxels)) = registry.take_pending_lod_completion(result) else { continue };
				for request in requests {
					lod_writer.write(LodLoaded {
						grid: request.request.grid,
						requester: request.request.requester,
						key: LodKey { min: request.request.key.min, size: request.request.key.size, lod: lod.max(0.0).floor() as u8 },
						priority: request.request.priority,
						generation: request.generation,
						voxels: voxels.clone(),
						entity: None,
					});
				}
			}
		}
	}
}
