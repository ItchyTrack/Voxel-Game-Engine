use std::collections::{HashMap, HashSet};

use bevy::prelude::*;

mod types {
	use bevy::prelude::*;
	use voxel_data::grid::GridId;

	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct ChunkKey { pub(crate) grid: GridId, pub(crate) chunk: IVec3 }

	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct TileKey { pub(crate) grid: GridId, pub(crate) lod: u8, pub(crate) min: IVec3 }
	impl TileKey { pub(crate) fn size(self) -> IVec3 { IVec3::splat(1i32 << self.lod) } }
}

mod camera_voxel_loader {
	use super::*;
	use crate::coverage::{CoverageCell, CoverageRecord, CoverageSource};
	use crate::types::ChunkKey;

	#[derive(Default)]
	pub(crate) struct CameraVoxelLoader {
		pub(crate) desired_chunks: HashSet<ChunkKey>,
		pub(crate) coverage_sources: HashMap<CoverageSource, CoverageRecord>,
		pub(crate) coverage_cells: HashMap<ChunkKey, CoverageCell>,
	}
}

#[path = "../src/coverage.rs"]
mod coverage;

use camera_voxel_loader::CameraVoxelLoader;
use coverage::{chunks_in_bounds, coverage_cell_replacement_state, ready_retiring_sources, remove_source, request_source, resolve_empty, resolve_visible, retiring_visible_chunks, undesire_source, CoverageCellReplacementState, CoverageSource, SourceResolution, SourceState};
use types::{ChunkKey, TileKey};

#[test]
fn subgrid_upload_must_not_resolve_chunks_outside_uploaded_voxel_bounds() {
	let grid = Entity::PLACEHOLDER;
	let subgrid_origin = IVec3::splat(57);
	let uploaded_bounds_min = subgrid_origin;
	let uploaded_bounds_max_exclusive = uploaded_bounds_min + IVec3::ONE;
	let chunks = chunks_in_bounds(grid, subgrid_origin, subgrid_origin);

	assert!(uploaded_bounds_max_exclusive.x <= 64, "control setup should occupy only streaming chunk x=0");
	assert!(
		!chunks.contains(&ChunkKey { grid, chunk: IVec3::new(1, 0, 0) }),
		"subgrid upload coverage must use uploaded voxel bounds, not the full fixed subgrid extent"
	);
}

#[test]
fn subgrid_upload_covers_every_chunk_in_uploaded_bounds() {
	let grid = Entity::PLACEHOLDER;
	let chunks = chunks_in_bounds(grid, IVec3::splat(63), IVec3::splat(64));

	assert!(chunks.contains(&ChunkKey { grid, chunk: IVec3::ZERO }));
	assert!(
		chunks.contains(&ChunkKey { grid, chunk: IVec3::new(1, 1, 1) }),
		"upload completion must resolve all chunks covered by the uploaded voxel bounds, not just the origin chunk"
	);
}

#[test]
fn reentering_already_gpu_visible_chunk_without_new_upload_event_should_replace_lod() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));

	// This simulates coming back to a chunk whose subgrid is already resident and
	// GPU-visible from an earlier visit. No ChunkLoadResolved or VoxelGpuUploadFinished
	// event will fire, because nothing loaded/uploaded this frame; the request path
	// must synchronously resolve the already-visible source from current world state.
	request_source(&mut loader, chunk);
	resolve_visible(&mut loader, chunk, Entity::from_bits(2));
	undesire_source(&mut loader, old_tile);

	assert_eq!(
		ready_retiring_sources(&loader),
		vec![old_tile],
		"an already resident/GPU-visible LOD0 chunk can be reintroduced without any new event; old LOD must still retire"
	);
}

#[test]
fn loaded_chunk_that_produces_no_subgrid_should_resolve_as_empty_coverage() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, chunk);
	undesire_source(&mut loader, old_tile);

	// A loaded chunk can still produce no visible subgrid. The streaming event should
	// report that as not visible, allowing it to resolve as empty coverage.
	let chunk_load_resolved_visible = false;
	if !chunk_load_resolved_visible {
		resolve_empty(&mut loader, chunk);
	}

	assert_eq!(
		ready_retiring_sources(&loader),
		vec![old_tile],
		"a chunk load that resolves successfully but produces no subgrid should count as empty replacement coverage"
	);
}

#[test]
fn coverage_cell_replacement_state_matches_retirement_decisions() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let replacement = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let chunk = ChunkKey { grid, chunk: IVec3::ZERO };
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	undesire_source(&mut loader, old_tile);
	let cell = loader.coverage_cells.get(&chunk).expect("retiring source should still cover its cells");
	assert_eq!(coverage_cell_replacement_state(&loader, cell, old_tile), CoverageCellReplacementState::NoDesiredCoverage);
	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);

	request_source(&mut loader, replacement);
	let cell = loader.coverage_cells.get(&chunk).expect("replacement request should cover the cell");
	assert_eq!(coverage_cell_replacement_state(&loader, cell, old_tile), CoverageCellReplacementState::Waiting);
	assert!(ready_retiring_sources(&loader).is_empty());

	resolve_empty(&mut loader, replacement);
	let cell = loader.coverage_cells.get(&chunk).expect("empty replacement should still cover the cell");
	assert_eq!(coverage_cell_replacement_state(&loader, cell, old_tile), CoverageCellReplacementState::Replaced);
	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn desired_requested_replacement_does_not_count_even_if_a_retiring_source_is_visible() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let retiring_replacement = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let pending_next_replacement = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let chunk = ChunkKey { grid, chunk: IVec3::ZERO };
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, retiring_replacement);
	resolve_visible(&mut loader, retiring_replacement, Entity::from_bits(2));
	request_source(&mut loader, pending_next_replacement);
	undesire_source(&mut loader, old_tile);
	undesire_source(&mut loader, retiring_replacement);

	let cell = loader.coverage_cells.get(&chunk).expect("all sources overlap the origin cell");
	assert_eq!(
		coverage_cell_replacement_state(&loader, cell, old_tile),
		CoverageCellReplacementState::Waiting,
		"retiring visible sources must not satisfy replacement coverage while the desired replacement is only requested"
	);
	assert!(ready_retiring_sources(&loader).is_empty());
}

#[test]
fn retiring_lod_waits_for_chunk_replacement_resolution_even_before_subgrid_exists() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, chunk);
	undesire_source(&mut loader, old_tile);

	assert!(ready_retiring_sources(&loader).is_empty());

	resolve_empty(&mut loader, chunk);

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn retiring_lod_waits_for_subgrid_gpu_visible_replacement() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, chunk);
	undesire_source(&mut loader, old_tile);

	assert!(ready_retiring_sources(&loader).is_empty());

	resolve_visible(&mut loader, chunk, Entity::from_bits(2));

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn retiring_lod_to_smaller_lod_only_waits_for_still_desired_overlap() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 2, min: IVec3::ZERO });
	let new_tile = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, new_tile);
	undesire_source(&mut loader, old_tile);

	assert!(ready_retiring_sources(&loader).is_empty());

	resolve_visible(&mut loader, new_tile, Entity::from_bits(2));

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn retiring_lod_to_subgrid_only_waits_for_still_desired_chunk_overlap() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 2, min: IVec3::ZERO });
	let chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, chunk);
	undesire_source(&mut loader, old_tile);

	assert!(ready_retiring_sources(&loader).is_empty());

	resolve_visible(&mut loader, chunk, Entity::from_bits(2));

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn late_old_lod_resolution_must_not_erase_already_resolved_lod_replacement() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 2, min: IVec3::ZERO });
	let replacement_tile = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	// Old LOD was requested first but has not resolved yet.
	request_source(&mut loader, old_tile);
	// Camera moves / policy changes and requests a smaller replacement LOD.
	request_source(&mut loader, replacement_tile);
	resolve_visible(&mut loader, replacement_tile, Entity::from_bits(2));

	// The old request finishes late. This must not wipe out the replacement coverage
	// when the old source is then marked undesired/retiring.
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	undesire_source(&mut loader, old_tile);

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn late_old_lod_resolution_must_not_erase_already_resolved_subgrid_replacement() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 2, min: IVec3::ZERO });
	let replacement_chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	// Old LOD was requested first but has not resolved yet.
	request_source(&mut loader, old_tile);
	// Camera moves close enough that one covered chunk is now replaced by LOD0.
	request_source(&mut loader, replacement_chunk);
	resolve_visible(&mut loader, replacement_chunk, Entity::from_bits(2));

	// The old LOD request/upload completes late. Retiring it should keep the already
	// visible subgrid coverage intact for the still-desired chunk.
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	undesire_source(&mut loader, old_tile);

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}

#[test]
fn retiring_lod0_chunk_waiting_for_lod1_tile_must_still_be_rendered() {
	let grid = Entity::PLACEHOLDER;
	let retiring_chunk_key = ChunkKey { grid, chunk: IVec3::ZERO };
	let retiring_chunk = CoverageSource::Chunk(retiring_chunk_key);
	let pending_lod1 = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();
	loader.desired_chunks.insert(retiring_chunk_key);

	request_source(&mut loader, retiring_chunk);
	resolve_visible(&mut loader, retiring_chunk, Entity::from_bits(1));
	request_source(&mut loader, pending_lod1);
	undesire_source(&mut loader, retiring_chunk);
	loader.desired_chunks.remove(&retiring_chunk_key);

	let chunks_rendered_by_refresh_visibility: Vec<_> = loader
		.desired_chunks
		.iter()
		.copied()
		.chain(retiring_visible_chunks(&loader))
		.collect();

	assert!(
		chunks_rendered_by_refresh_visibility.contains(&retiring_chunk_key),
		"a retiring visible LOD0 chunk waiting for a pending LOD1 tile must stay in the render list"
	);
}

#[test]
fn two_retiring_visible_sources_must_not_release_each_other_while_next_replacement_is_pending() {
	let grid = Entity::PLACEHOLDER;
	let retiring_chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let retiring_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let pending_replacement = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, retiring_chunk);
	resolve_visible(&mut loader, retiring_chunk, Entity::from_bits(1));
	request_source(&mut loader, retiring_tile);
	resolve_visible(&mut loader, retiring_tile, Entity::from_bits(2));
	request_source(&mut loader, pending_replacement);

	undesire_source(&mut loader, retiring_chunk);
	undesire_source(&mut loader, retiring_tile);

	assert!(
		ready_retiring_sources(&loader).is_empty(),
		"two retiring visible sources currently count each other as replacement coverage, so both can release before the pending desired replacement is visible"
	);
}

#[test]
fn policy_transition_from_subgrid_to_lod_must_not_retire_chunk_before_tile_request_is_registered() {
	let grid = Entity::PLACEHOLDER;
	let chunk_key = ChunkKey { grid, chunk: IVec3::ZERO };
	let chunk = CoverageSource::Chunk(chunk_key);
	let replacement_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, chunk);
	resolve_visible(&mut loader, chunk, Entity::from_bits(1));

	// The policy transition must register replacement coverage before retiring the
	// old chunk, so the chunk waits for the tile to resolve instead of releasing early.
	request_source(&mut loader, replacement_tile);
	undesire_source(&mut loader, chunk);

	assert!(
		!ready_retiring_sources(&loader).contains(&chunk),
		"LOD0 chunk became ready to release even though the replacement LOD tile is only requested, not resolved"
	);
	assert!(ready_retiring_sources(&loader).is_empty());
}

#[test]
fn removing_visible_source_clears_its_cell_coverage() {
	let grid = Entity::PLACEHOLDER;
	let chunk = ChunkKey { grid, chunk: IVec3::ZERO };
	let source = CoverageSource::Chunk(chunk);
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, source);
	resolve_visible(&mut loader, source, Entity::from_bits(1));
	remove_source(&mut loader, source);

	assert!(
		loader.coverage_cells.get(&chunk).is_none_or(|cell| !cell.visible.contains(&source)),
		"remove_source left stale visible coverage from a removed source"
	);
}

#[test]
fn re_desiring_a_retiring_visible_source_cancels_retirement_and_restores_coverage() {
	let grid = Entity::PLACEHOLDER;
	let source = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, source);
	resolve_visible(&mut loader, source, Entity::from_bits(1));
	undesire_source(&mut loader, source);
	request_source(&mut loader, source);

	let record = loader.coverage_sources.get(&source).expect("source should still be tracked");
	assert!(
		matches!(record.state, SourceState::Desired(SourceResolution::Visible(_))),
		"re-desiring a still-visible retiring source should make it desired-visible instead of leaving it retiring"
	);
	assert!(
		ready_retiring_sources(&loader).is_empty(),
		"a source that became desired again must not be returned as ready to retire"
	);
}

#[test]
fn retiring_visible_replacement_still_counts_while_next_replacement_is_pending() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let visible_replacement = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let pending_replacement = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, visible_replacement);
	resolve_visible(&mut loader, visible_replacement, Entity::from_bits(2));
	request_source(&mut loader, pending_replacement);

	undesire_source(&mut loader, old_tile);
	assert!(ready_retiring_sources(&loader).contains(&old_tile));

	undesire_source(&mut loader, visible_replacement);

	assert!(
		!ready_retiring_sources(&loader).contains(&old_tile),
		"a retiring visible replacement must not count as stable replacement coverage for an older retiring source while the next replacement is pending"
	);
}

#[test]
fn removing_one_overlapping_replacement_does_not_forget_another_visible_replacement() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 1, min: IVec3::ZERO });
	let replacement_chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let overlapping_replacement_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, replacement_chunk);
	request_source(&mut loader, overlapping_replacement_tile);
	undesire_source(&mut loader, old_tile);

	resolve_visible(&mut loader, replacement_chunk, Entity::from_bits(2));
	resolve_visible(&mut loader, overlapping_replacement_tile, Entity::from_bits(3));
	undesire_source(&mut loader, overlapping_replacement_tile);

	assert!(
		ready_retiring_sources(&loader).contains(&old_tile),
		"removing one resolved overlapping replacement cleared the cell coverage even though another desired replacement source is still visible"
	);
}

#[test]
fn retiring_lod_does_not_get_stuck_after_replacement_resolved_empty() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = CoverageSource::Tile(TileKey { grid, lod: 0, min: IVec3::ZERO });
	let chunk = CoverageSource::Chunk(ChunkKey { grid, chunk: IVec3::ZERO });
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, chunk);
	undesire_source(&mut loader, old_tile);
	resolve_empty(&mut loader, chunk);

	// Re-running policy/visibility work should not re-add an already-resolved dependency.
	request_source(&mut loader, chunk);
	resolve_empty(&mut loader, chunk);

	assert_eq!(ready_retiring_sources(&loader), vec![old_tile]);
}
