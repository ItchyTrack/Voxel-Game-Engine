use std::collections::HashSet;

use bevy::prelude::*;
use tracy_client::span;
use voxel_gpu::{VoxelGpuFormat, VoxelGpuState};
use voxel_data::grid::Grid;
use voxel_data::grid_tree::GridRegion;
use voxel_data::subgrid::SubGrid;
use voxel_streaming::{ChunkConsumer, GridStreaming, LodKey, VoxelSourceRequestApi};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::lod_policy::tile_has_present_source;
use crate::replacement_graph::DependencyRecord;
use crate::subgrid_interface::resolve_chunk_source_if_ready;
use crate::types::{SourceResolution, SourceState, TileKey, TileRecord, TileStatus};
use crate::CameraVoxelLoaderConsumer;

// Deciding what the player gets happens in `scheduling`; the desired set is already chosen by the
// time these run. Everything here only makes an already-decided (un)desire happen against the
// streaming layer and the coverage bookkeeping.

pub(crate) fn want_source(
	camera_voxel_loader: &mut CameraVoxelLoader,
	format: VoxelGpuFormat,
	streaming: &mut GridStreaming,
	requests: &impl VoxelSourceRequestApi,
	grid: &Grid,
	subgrids: &Query<&SubGrid>,
	gpu_state: &Query<&VoxelGpuState>,
	camera_entity: Entity,
	key: TileKey,
) {
	if key.is_chunk() {
		request_source(camera_voxel_loader, key);
		streaming.fetch(key.grid, requests, key.min);
		if matches!(streaming.state(key.min), Some(voxel_streaming::ChunkState::Loaded | voxel_streaming::ChunkState::InternalDirty)) {
			let ready = resolve_chunk_source_if_ready(camera_voxel_loader, format, grid, subgrids, gpu_state, key);
			retire_sources(camera_voxel_loader, streaming, camera_entity, ready);
		}
		return;
	}
	let Some(record) = camera_voxel_loader.tiles.get_mut(&key) else {
		if !matches!(camera_voxel_loader.coverage_sources.get(&key), Some(SourceState::Desired(SourceResolution::Empty))) {
			camera_voxel_loader.tiles.insert(key, TileRecord::queued());
			request_source(camera_voxel_loader, key);
			camera_voxel_loader.queue.push_back(key);
		}
		return;
	};
	if record.status == TileStatus::Retiring {
		record.status = TileStatus::Ready;
	}
	if record.status == TileStatus::Ready {
		if let Some(entity) = record.entity {
			request_source(camera_voxel_loader, key);
			resolve_and_retire(camera_voxel_loader, streaming, camera_entity, key, SourceResolution::Visible(entity));
		}
	}
}

pub(crate) fn unwant_source(
	camera_voxel_loader: &mut CameraVoxelLoader,
	streaming: &mut GridStreaming,
	camera_entity: Entity,
	key: TileKey,
) {
	if tile_has_present_source(streaming, key) {
		// Data is still valid: retire through the no-gap path so a visible tile stays on screen until replacement coverage is ready.
		let ready = handle_non_desired_tile(camera_voxel_loader, key);
		retire_sources(camera_voxel_loader, streaming, camera_entity, ready);
		return;
	}
	// Data is gone: resolve the coverage empty (unblocking dependents) and throw the tile away.
	let ready = resolve_empty(camera_voxel_loader, key);
	retire_sources(camera_voxel_loader, streaming, camera_entity, ready.into_iter().chain([key]));
}

pub(crate) fn retire_sources(
	camera_voxel_loader: &mut CameraVoxelLoader,
	streaming: &mut GridStreaming,
	requester: Entity,
	sources: impl IntoIterator<Item = TileKey>,
) {
	for source in sources {
		remove_source(camera_voxel_loader, source);
		if source.is_chunk() {
			streaming.release(source.min);
		} else {
			streaming.release_lod(requester, lod_key(source));
			camera_voxel_loader.tiles.remove(&source);
		}
	}
}

pub(crate) fn resolve_and_retire(
	camera_voxel_loader: &mut CameraVoxelLoader,
	streaming: &mut GridStreaming,
	requester: Entity,
	source: TileKey,
	resolution: SourceResolution,
) {
	let ready = resolve_source(camera_voxel_loader, source, resolution);
	retire_sources(camera_voxel_loader, streaming, requester, ready);
}

fn handle_non_desired_tile(camera_voxel_loader: &mut CameraVoxelLoader, key: TileKey) -> Vec<TileKey> {
	if key.is_chunk() {
		return undesire_source(camera_voxel_loader, key);
	}
	let Some(status) = camera_voxel_loader.tiles.get(&key).map(|record| record.status) else { return Vec::new() };
	match status {
		// Already-renderable tiles must stay visible until replacement coverage is ready.
		TileStatus::Ready | TileStatus::Retiring => {
			if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
				record.status = TileStatus::Retiring;
			}
			undesire_source(camera_voxel_loader, key)
		}
		// In-flight work may still produce valid voxel data. Keep the record so the
		// late result can be applied, then retire it through the same no-gap path.
		TileStatus::Loading => {
			camera_voxel_loader.unresolved_tiles.remove(key);
			Vec::new()
		}
		// Queued work has not been sent yet, so it can be safely dropped.
		TileStatus::Queued => {
			camera_voxel_loader.tiles.remove(&key);
			remove_source(camera_voxel_loader, key);
			Vec::new()
		}
	}
}

pub(crate) fn undesire_source(loader: &mut CameraVoxelLoader, source: TileKey) -> Vec<TileKey> {
	match loader.coverage_sources.get(&source) {
		None | Some(SourceState::RetiringVisible(_)) => Vec::new(),
		Some(SourceState::Desired(SourceResolution::Requested | SourceResolution::Empty)) => {
			remove_source(loader, source);
			Vec::new()
		}
		Some(SourceState::Desired(SourceResolution::Visible(entity))) => {
			let replacements = unresolved_replacements_for(loader, source);
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::RetiringVisible(*entity);
			if replacements.is_empty() {
				vec![source]
			} else {
				loader.replacement_graph.add_record(DependencyRecord::new(source, replacements));
				Vec::new()
			}
		}
	}
}

pub(crate) fn lod_key(key: TileKey) -> LodKey {
	LodKey { min: key.min, size: key.size(), lod: key.lod }
}

pub(crate) fn receive_camera_voxel_loader_results(
	mut camera_voxel_loaders: Query<&mut CameraVoxelLoader>,
	mut consumers: Query<&mut CameraVoxelLoaderConsumer>,
	mut grids: Query<&mut GridStreaming>,
) {
	for mut consumer in &mut consumers {
		let results = consumer.drain_lod();
		if results.is_empty() {
			continue;
		}
		// The consumer lives on the same entity as the controller.
		// Query iteration order is not reliable, so use the requester embedded in each result.
		for result in results {
			let Ok(mut camera_voxel_loader) = camera_voxel_loaders.get_mut(result.requester) else { continue };
			let key = TileKey { grid: result.grid, lod: result.key.lod, min: result.key.min };
			let Some(_) = camera_voxel_loader.tiles.get(&key) else { continue };
			let Ok(streaming) = grids.get_mut(result.grid) else { continue };
			let streaming = streaming.into_inner();
			match result.entity {
				Some(entity) => {
					if let Some(record) = camera_voxel_loader.tiles.get_mut(&key) {
						record.entity = Some(entity);
						record.status = TileStatus::Ready;
					}
					resolve_and_retire(&mut camera_voxel_loader, streaming, result.requester, key, SourceResolution::Visible(entity));
					if !camera_voxel_loader.desired_tiles.contains(&key) {
						let ready = handle_non_desired_tile(&mut camera_voxel_loader, key);
						retire_sources(&mut camera_voxel_loader, streaming, result.requester, ready);
					}
				}
				None => {
					camera_voxel_loader.tiles.remove(&key);
					resolve_and_retire(&mut camera_voxel_loader, streaming, result.requester, key, SourceResolution::Empty);
					if !camera_voxel_loader.desired_tiles.contains(&key) {
						retire_sources(&mut camera_voxel_loader, streaming, result.requester, [key]);
					}
				}
			}
		}
	}
}

// Coverage bookkeeping: the state machine that tracks each source's resolution and drives the
// no-gap replacement graph. Called by the executors above.

pub(crate) fn request_source(loader: &mut CameraVoxelLoader, source: TileKey) {
	let _span = span!();
	match loader.coverage_sources.get(&source) {
		None => {
			loader.coverage_sources.insert(source, SourceState::Desired(SourceResolution::Requested));
			loader.unresolved_tiles.insert(source);
		}
		Some(SourceState::RetiringVisible(entity)) => {
			loader.replacement_graph.cancel_record(source);
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::Desired(SourceResolution::Visible(*entity));
		}
		_ => {}
	}
}

pub(crate) fn resolve_empty(loader: &mut CameraVoxelLoader, source: TileKey) -> Vec<TileKey> {
	resolve_source(loader, source, SourceResolution::Empty)
}

pub(crate) fn resolve_visible(loader: &mut CameraVoxelLoader, source: TileKey, entity: Entity) -> Vec<TileKey> {
	resolve_source(loader, source, SourceResolution::Visible(entity))
}

fn resolve_source(loader: &mut CameraVoxelLoader, source: TileKey, resolution: SourceResolution) -> Vec<TileKey> {
	let ready = match loader.coverage_sources.get(&source) {
		None => return Vec::new(),
		Some(SourceState::Desired(_)) => {
			*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::Desired(resolution);
			loader.replacement_graph.apply_satisfied(source)
		}
		Some(SourceState::RetiringVisible(_)) => {
			if let SourceResolution::Visible(entity) = resolution {
				*loader.coverage_sources.get_mut(&source).unwrap() = SourceState::RetiringVisible(entity);
			}
			let mut ready = loader.replacement_graph.apply_satisfied(source);
			if matches!(resolution, SourceResolution::Empty) {
				ready.push(source);
			}
			ready
		}
	};
	loader.unresolved_tiles.remove(source);
	ready
}

pub(crate) fn remove_source(loader: &mut CameraVoxelLoader, source: TileKey) {
	loader.coverage_sources.remove(&source);
	loader.unresolved_tiles.remove(source);
	loader.replacement_graph.remove_source(source);
}

fn unresolved_replacements_for(loader: &CameraVoxelLoader, source: TileKey) -> HashSet<TileKey> {
	let Some(region) = GridRegion::from_min_size(source.min, source.size()) else { return HashSet::new() };
	let mut replacements = HashSet::new();
	loader.unresolved_tiles.for_each_in_region(source.grid, region, loader.settings.max_lod, Some(source.lod), |candidate| {
		replacements.insert(candidate);
	});
	replacements
}

#[cfg(test)]
mod tests {
	use super::*;

	fn grid(bits: u64) -> Entity { Entity::from_bits(bits) }
	fn tile(grid: Entity, lod: u8, min: IVec3) -> TileKey { TileKey { grid, lod, min } }

	#[test]
	fn undesired_loading_lod_waits_for_result_until_dependency_retirement() {
		let grid = Entity::from_bits(1);
		let key = TileKey { grid, lod: 1, min: IVec3::ZERO };
		let mut loader = CameraVoxelLoader::default();
		loader.tiles.insert(key, TileRecord { status: TileStatus::Loading, entity: None });
		request_source(&mut loader, key);

		let ready = handle_non_desired_tile(&mut loader, key);

		assert!(ready.is_empty());
		assert!(loader.tiles.contains_key(&key), "loading LOD records stay until the in-flight result resolves because release_lod is delayed until dependency retirement");
		assert!(loader.coverage_sources.contains_key(&key), "coverage remains requested so the late result can satisfy replacement dependencies");
		let mut unresolved = Vec::new();
		loader.unresolved_tiles.for_each_in_region(key.grid, GridRegion::from_min_size(key.min, key.size()).unwrap(), loader.settings.max_lod, None, |candidate| unresolved.push(candidate));
		assert!(unresolved.is_empty(), "undesired in-flight tiles must not block other retirements");
	}

	#[test]
	fn same_grid_replacements_do_not_wait_on_other_grids() {
		let source_grid = grid(1);
		let other_grid = grid(2);
		let source = tile(source_grid, 1, IVec3::ZERO);
		let replacement = tile(source_grid, 0, IVec3::ZERO);
		let other_grid_replacement = tile(other_grid, 0, IVec3::ZERO);
		let mut loader = CameraVoxelLoader::default();
		loader.insert_desired_tile(replacement);
		loader.insert_desired_tile(other_grid_replacement);

		request_source(&mut loader, source);
		resolve_visible(&mut loader, source, Entity::from_bits(10));
		request_source(&mut loader, replacement);

		assert!(undesire_source(&mut loader, source).is_empty());
		assert_eq!(resolve_visible(&mut loader, replacement, Entity::from_bits(11)), vec![source]);
	}
}
