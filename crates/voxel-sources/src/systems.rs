use bevy::ecs::message::MessageWriter;
use bevy::prelude::*;

use voxel_data::voxels::Voxels;
use voxel_streaming::{
	ChunkBecamePresent, ChunkLoaderChannel, ChunkLoadResult, ChunkSaveChannel,
	ChunkState, GridStreaming, LodLoaderChannel, LodLoadResult,
};

use crate::handle::{SourceEvent, SourceHandle};
use crate::registry::{LodRequestKey, PendingLodJob, SourceRegistry};
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
	mut present_events: MessageWriter<ChunkBecamePresent>,
) {
	let rx = registry.event_rx.clone();
	while let Ok(event) = rx.try_recv() {
		match event {
			SourceEvent::Available { grid, chunk } => {
				let Some(entity) = registry.entity(grid) else { continue };
				if let Ok(mut s) = grids.get_mut(entity) {
					if s.presence().state(chunk).is_none() {
						present_events.write(ChunkBecamePresent { grid: entity, chunk });
						s.presence_mut().mark_present(chunk);
					}
				}
			}
			SourceEvent::AvailableArea { grid, min, size } => {
				let Some(entity) = registry.entity(grid) else { continue };
				if let Ok(mut s) = grids.get_mut(entity) {
					for x in min.x..min.x + size.x {
						for y in min.y..min.y + size.y {
							for z in min.z..min.z + size.z {
								let chunk = IVec3::new(x, y, z);
								if s.presence().state(chunk).is_none() {
									present_events.write(ChunkBecamePresent { grid: entity, chunk });
								}
							}
						}
					}
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
	while let Ok(result) = rx.try_recv() {
		let Some(grid) = registry.entity(result.grid) else { continue };
		let key = LodRequestKey::new(result.grid, result.min, result.size, result.lod);
		let completed = {
			let mut pending_lod = registry.pending_lod.lock().unwrap();
			let Some(job) = pending_lod.get_mut(&key) else { continue };
			match job {
				PendingLodJob::Direct { requests } => {
					let requests = std::mem::take(requests);
					pending_lod.remove(&key);
					Some((requests, result.lod, result.voxels))
				}
				PendingLodJob::Composite { requests, expected, received, final_lod, intermediate_lod } => {
					received.insert(result.source, result.voxels);
					if !expected.iter().all(|source| received.contains_key(source)) {
						None
					} else {
						let requests = std::mem::take(requests);
						let final_lod = *final_lod;
						let intermediate_lod = *intermediate_lod;
						let parts: Vec<_> = received.values().filter_map(Clone::clone).collect();
						pending_lod.remove(&key);
						let merged = merge_voxels(parts);
						let voxels = if final_lod <= intermediate_lod {
							(!merged.is_empty()).then_some(merged)
						} else {
							registry.lod_generator.generate(&merged, final_lod - intermediate_lod)
						};
						Some((requests, final_lod, voxels))
					}
				}
			}
		};
		let Some((requests, lod, voxels)) = completed else { continue };
		for request in requests {
			loader.report(LodLoadResult {
				grid,
				requester: request.requester,
				key: voxel_streaming::LodKey { min: request.key.min, size: request.key.size, lod: lod.max(0.0).floor() as u8 },
				priority: request.priority,
				generation: request.generation,
				voxels: voxels.clone(),
				entity: None,
			});
		}
	}
}

fn merge_voxels(parts: Vec<Voxels>) -> Voxels {
	let mut merged = Voxels::new();
	for voxels in parts {
		let areas: Vec<_> = voxels
			.grid_tree()
			.iter()
			.map(|(pos, size, id)| (pos, bevy::math::I16Vec3::splat(size as i16), id))
			.collect();
		merged.add_palette_areas(&areas, voxels.palette());
	}
	merged
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
