use bevy::prelude::*;

use voxel_streaming::{
	ChunkLoaderChannel, ChunkLoadResult, ChunkSaveChannel, ChunkState,
	GridStreaming, LodLoaderChannel, LodLoadResult,
};

use crate::handle::{SourceEvent, SourceHandle};
use crate::registry::SourceRegistry;
use crate::source::{GridKey, SourceId};

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

pub(crate) fn sync_grid_keys(mut registry: ResMut<SourceRegistry>, grids: Query<(Entity, &GridKey)>) {
	let grid_keys = registry.grid_keys.clone();
	let mut grid_keys = grid_keys.write().unwrap();
	registry.keys.clear();
	grid_keys.clear();
	for (entity, key) in &grids {
		registry.keys.insert(*key, entity);
		grid_keys.insert(entity, *key);
	}
}

/// Drains source events before any load/save is routed, so edits and
/// availability land first.
pub(crate) fn apply_source_events(
	registry: Res<SourceRegistry>,
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
						s.mark_external_dirty(chunk);
					}
				}
			}
		}
	}
}

pub(crate) fn drain_source_lod_results(
	registry: Res<SourceRegistry>,
	loader: Res<LodLoaderChannel>,
) {
	let rx = registry.lod_result_rx.clone();
	let mut pending_lod = registry.pending_lod.lock().unwrap();
	while let Ok(result) = rx.try_recv() {
		let Some(grid) = registry.entity(result.grid) else { continue };
		let matched = pending_lod.iter().position(|r| {
			r.grid == grid
				&& r.min == result.min
				&& r.size == result.size
				&& r.lod.to_bits() == result.lod.to_bits()
		});
		let Some(pos) = matched else { continue };
		let request = pending_lod.swap_remove(pos);
		loader.report(LodLoadResult {
			grid,
			requester: request.requester,
			min: result.min,
			size: result.size,
			lod: result.lod,
			priority: request.priority,
			voxels: result.voxels,
		});
	}
}

pub(crate) fn serve_saves(
	saves: Res<ChunkSaveChannel>,
	registry: Res<SourceRegistry>,
	grids: Query<&GridKey>,
) {
	while let Some(save) = saves.try_recv() {
		let Ok(&key) = grids.get(save.grid) else { continue };
		for source in registry.sources.iter() {
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
