use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::CHUNK_SIZE;

mod types {
	use bevy::prelude::*;
	use voxel_data::grid::GridId;

	#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
	pub(crate) struct TileKey {
		pub(crate) grid: GridId,
		pub(crate) lod: u8,
		pub(crate) min: IVec3,
	}
	impl TileKey {
		pub(crate) fn size(self) -> IVec3 {
			IVec3::splat(1i32 << self.lod)
		}
	}
}

#[allow(dead_code)]
#[path = "../src/replacement_graph.rs"]
mod replacement_graph;
#[allow(dead_code)]
#[path = "../src/unresolved_tile_index.rs"]
mod unresolved_tile_index;

mod camera_voxel_loader {
	use super::*;
	use crate::coverage::SourceState;
	use crate::replacement_graph::ReplacementGraph;
	use crate::types::TileKey;
	use crate::unresolved_tile_index::UnresolvedTileIndex;

	#[derive(Debug, Clone)]
	pub(crate) struct CameraVoxelLoaderSettings {
		pub(crate) max_lod: u8,
	}
	impl Default for CameraVoxelLoaderSettings {
		fn default() -> Self {
			Self { max_lod: 6 }
		}
	}

	#[derive(Default)]
	pub(crate) struct CameraVoxelLoader {
		pub(crate) settings: CameraVoxelLoaderSettings,
		pub(crate) desired_tiles: HashSet<TileKey>,
		pub(crate) coverage_sources: HashMap<TileKey, SourceState>,
		pub(crate) unresolved_tiles: UnresolvedTileIndex,
		pub(crate) replacement_graph: ReplacementGraph,
	}
}

#[allow(dead_code)]
#[path = "../src/coverage.rs"]
mod coverage;

use camera_voxel_loader::CameraVoxelLoader;
use coverage::{
	remove_source, request_source, resolve_empty, resolve_visible, retiring_visible_chunks, undesire_source, SourceResolution, SourceState,
};
use types::TileKey;

fn chunk(grid: GridId, min: IVec3) -> TileKey {
	TileKey { grid, lod: 0, min }
}
fn tile(grid: GridId, lod: u8, min: IVec3) -> TileKey {
	TileKey { grid, lod, min }
}

fn chunks_in_bounds(grid: GridId, min: IVec3, max: IVec3) -> Vec<TileKey> {
	let min = min.div_euclid(IVec3::splat(CHUNK_SIZE));
	let max = max.div_euclid(IVec3::splat(CHUNK_SIZE));
	let mut chunks = Vec::new();
	for x in min.x..=max.x {
		for y in min.y..=max.y {
			for z in min.z..=max.z {
				chunks.push(chunk(grid, IVec3::new(x, y, z)));
			}
		}
	}
	chunks
}

#[test]
fn subgrid_upload_must_not_resolve_chunks_outside_uploaded_voxel_bounds() {
	let grid = Entity::PLACEHOLDER;
	let subgrid_origin = IVec3::splat(57);
	let uploaded_bounds_max_exclusive = subgrid_origin + IVec3::ONE;
	let chunks = chunks_in_bounds(grid, subgrid_origin, subgrid_origin);

	assert!(uploaded_bounds_max_exclusive.x <= 64);
	assert!(!chunks.contains(&chunk(grid, IVec3::new(1, 0, 0))));
}

#[test]
fn subgrid_upload_covers_every_chunk_in_uploaded_bounds() {
	let grid = Entity::PLACEHOLDER;
	let chunks = chunks_in_bounds(grid, IVec3::splat(63), IVec3::splat(64));

	assert!(chunks.contains(&chunk(grid, IVec3::ZERO)));
	assert!(chunks.contains(&chunk(grid, IVec3::new(1, 1, 1))));
}

#[test]
fn visible_replacement_allows_retiring_source_to_remove() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = tile(grid, 1, IVec3::ZERO);
	let replacement = chunk(grid, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(replacement);

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, replacement);
	assert!(undesire_source(&mut loader, old_tile).is_empty());
	assert_eq!(resolve_visible(&mut loader, replacement, Entity::from_bits(2)), vec![old_tile]);
}

#[test]
fn empty_replacement_allows_retiring_source_to_remove() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = tile(grid, 1, IVec3::ZERO);
	let replacement = chunk(grid, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(replacement);

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, replacement);
	assert!(undesire_source(&mut loader, old_tile).is_empty());
	assert_eq!(resolve_empty(&mut loader, replacement), vec![old_tile]);
}

#[test]
fn already_visible_replacement_removes_immediately_without_waiting_for_event() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = tile(grid, 1, IVec3::ZERO);
	let replacement = chunk(grid, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(replacement);

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, replacement);
	resolve_visible(&mut loader, replacement, Entity::from_bits(2));
	assert_eq!(undesire_source(&mut loader, old_tile), vec![old_tile]);
}

#[test]
fn re_desired_retiring_source_then_undesired_again_can_retire_again() {
	let grid = Entity::PLACEHOLDER;
	let old_tile = tile(grid, 1, IVec3::ZERO);
	let replacement = chunk(grid, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(replacement);

	request_source(&mut loader, old_tile);
	resolve_visible(&mut loader, old_tile, Entity::from_bits(1));
	request_source(&mut loader, replacement);
	undesire_source(&mut loader, old_tile);
	request_source(&mut loader, old_tile);

	assert!(undesire_source(&mut loader, old_tile).is_empty());
	assert_eq!(resolve_visible(&mut loader, replacement, Entity::from_bits(2)), vec![old_tile]);
}

#[test]
fn removing_ready_source_cleans_record() {
	let grid = Entity::PLACEHOLDER;
	let source = tile(grid, 1, IVec3::ZERO);
	let replacement = chunk(grid, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(replacement);

	request_source(&mut loader, source);
	resolve_visible(&mut loader, source, Entity::from_bits(1));
	request_source(&mut loader, replacement);
	resolve_empty(&mut loader, replacement);
	let ready = undesire_source(&mut loader, source);
	assert_eq!(ready, vec![source]);

	remove_source(&mut loader, source);
	assert!(!loader.coverage_sources.contains_key(&source));
}

#[test]
fn retiring_chunks_are_reported_for_rendering() {
	let grid = Entity::PLACEHOLDER;
	let chunk = chunk(grid, IVec3::ZERO);
	let replacement = tile(grid, 1, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();
	loader.desired_tiles.insert(replacement);

	request_source(&mut loader, chunk);
	resolve_visible(&mut loader, chunk, Entity::from_bits(1));
	request_source(&mut loader, replacement);
	undesire_source(&mut loader, chunk);

	assert_eq!(retiring_visible_chunks(&loader), vec![chunk]);
}

#[test]
fn source_state_records_visibility_and_empty() {
	let grid = Entity::PLACEHOLDER;
	let visible = chunk(grid, IVec3::ZERO);
	let empty = tile(grid, 1, IVec3::ZERO);
	let mut loader = CameraVoxelLoader::default();

	request_source(&mut loader, visible);
	resolve_visible(&mut loader, visible, Entity::from_bits(3));
	request_source(&mut loader, empty);
	resolve_empty(&mut loader, empty);

	assert_eq!(loader.coverage_sources[&visible], SourceState::Desired(SourceResolution::Visible(Entity::from_bits(3))));
	assert_eq!(loader.coverage_sources[&empty], SourceState::Desired(SourceResolution::Empty));
}
