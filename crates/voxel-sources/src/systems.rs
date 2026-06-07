use bevy::prelude::*;

use voxel_streaming::{
	ChunkLoaderChannel, ChunkLoadResult, ChunkRequestChannel, ChunkSaveChannel, ChunkState,
	GridStreaming,
};

use crate::handle::{SourceEvent, SourceHandle};
use crate::registry::SourceRegistry;
use crate::source::{GridKey, SourceId};

pub(crate) fn init_sources(mut registry: ResMut<SourceRegistry>) {
	let events = registry.event_tx.clone();
	let results = registry.result_tx.clone();
	for (i, source) in registry.sources.iter_mut().enumerate() {
		source.init(SourceHandle { id: SourceId(i), events: events.clone(), results: results.clone() });
	}
}

pub(crate) fn sync_grid_keys(mut registry: ResMut<SourceRegistry>, grids: Query<(Entity, &GridKey)>) {
	registry.keys.clear();
	for (entity, key) in &grids {
		registry.keys.insert(*key, entity);
	}
}

/// Drains source events before any load/save is routed, so edits and
/// availability land first.
pub(crate) fn apply_source_events(
	mut registry: ResMut<SourceRegistry>,
	mut grids: Query<&mut GridStreaming>,
) {
	let rx = registry.event_rx.clone();
	while let Ok(event) = rx.try_recv() {
		match event {
			SourceEvent::Available { grid, chunk } => {
				let Some(entity) = registry.entity(grid) else { continue };
				if let Ok(mut s) = grids.get_mut(entity) {
					if s.presence().state(chunk).is_none() {
						s.presence_mut().mark_present(chunk);
					}
				}
			}
			SourceEvent::AvailableArea { grid, min, size } => {
				let Some(entity) = registry.entity(grid) else { continue };
				if let Ok(mut s) = grids.get_mut(entity) {
					s.presence_mut().mark_present_area(min, size);
				}
			}
			SourceEvent::Unavailable { grid, chunk } => {
				let Some(entity) = registry.entity(grid) else { continue };
				if let Ok(mut s) = grids.get_mut(entity) {
					if matches!(s.presence().state(chunk), Some(ChunkState::Available)) {
						s.presence_mut().clear_present(chunk);
					}
				}
			}
			SourceEvent::Edited { source, grid, chunk } => {
				let entity = registry.entity(grid);
				registry.forget_others(source, grid, chunk);
				if let Some(entity) = entity {
					if let Ok(mut s) = grids.get_mut(entity) {
						s.presence_mut().set_state(chunk, ChunkState::ExternalDirty);
					}
				}
			}
		}
	}
}

pub(crate) fn serve_requests(
	requests: Res<ChunkRequestChannel>,
	mut registry: ResMut<SourceRegistry>,
	grids: Query<&GridKey>,
) {
	while let Some(request) = requests.try_recv() {
		let Ok(&key) = grids.get(request.grid) else { continue };
		if let Some(id) = registry.cheapest(key, request.chunk) {
			registry.sources[id.0].request_load(key, request.chunk);
		}
	}
}

pub(crate) fn serve_saves(
	saves: Res<ChunkSaveChannel>,
	mut registry: ResMut<SourceRegistry>,
	grids: Query<&GridKey>,
) {
	while let Some(save) = saves.try_recv() {
		let Ok(&key) = grids.get(save.grid) else { continue };
		for source in registry.sources.iter_mut() {
			if source.can_save() {
				source.save(key, save.chunk, &save.voxels);
			} else {
				source.forget(key, save.chunk);
			}
		}
	}
}

pub(crate) fn drain_source_results(registry: Res<SourceRegistry>, loader: Res<ChunkLoaderChannel>) {
	while let Ok(result) = registry.result_rx.try_recv() {
		if let Some(grid) = registry.entity(result.grid) {
			loader.report(ChunkLoadResult { grid, chunk: result.chunk, voxels: result.voxels });
		}
	}
}
