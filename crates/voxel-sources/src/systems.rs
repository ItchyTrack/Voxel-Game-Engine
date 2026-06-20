use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

use crate::handle::SourceHandle;
use crate::registry::SourceRegistry;
use crate::source::SourceId;
use crate::{ChunkLoaded, LodLoaded, LodKey};

pub(crate) fn init_sources(registry: ResMut<SourceRegistry>) {
	let events = registry.event_tx.clone();
	let results = registry.result_tx.clone();
	let lod_results = registry.lod_result_tx.clone();
	for (i, source) in registry.sources.iter().enumerate() {
		source.init(SourceHandle {
			id: SourceId(i),
			events: events.clone(),
			results: results.clone(),
			lod_results: lod_results.clone(),
		});
	}
}

pub(crate) fn publish_chunk_results(registry: Res<SourceRegistry>, mut writer: MessageWriter<ChunkLoaded>) {
	while let Some(result) = registry.try_recv_result() {
		writer.write(ChunkLoaded { grid: result.grid, chunk: result.chunk, voxels: result.voxels });
	}
}

pub(crate) fn publish_lod_results(registry: Res<SourceRegistry>, mut writer: MessageWriter<LodLoaded>) {
	while let Some(result) = registry.try_recv_lod_result() {
		let Some((requests, lod, voxels)) = registry.take_pending_lod_completion(result) else { continue };
		for request in requests {
			writer.write(LodLoaded {
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
