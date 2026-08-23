use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_sources::SourceManager;
use voxel_streaming::{ChunkConsumer, GridStreaming, TileLoadUpdate};

#[derive(Default)]
struct TestConsumer {
	needed: HashMap<GridId, HashSet<IVec3>>,
	outstanding: usize,
	tile_updates: Vec<TileLoadUpdate>,
}

impl ChunkConsumer for TestConsumer {
	fn needed(&self) -> &HashMap<GridId, HashSet<IVec3>> { &self.needed }
	fn needed_mut(&mut self) -> &mut HashMap<GridId, HashSet<IVec3>> { &mut self.needed }
	fn outstanding(&self) -> usize { self.outstanding }
	fn outstanding_mut(&mut self) -> &mut usize { &mut self.outstanding }
	fn push_tile(&mut self, update: TileLoadUpdate) { self.tile_updates.push(update); }
	fn drain_tiles(&mut self) -> Vec<TileLoadUpdate> { std::mem::take(&mut self.tile_updates) }
}

#[test]
fn chunk_request_releases_cleanly_after_load_when_no_longer_needed() {
	let grid = Entity::PLACEHOLDER;
	let chunk = IVec3::new(2, 0, -1);
	let mut sources = SourceManager::default();
	let mut streaming = GridStreaming::default();
	let mut consumer = TestConsumer::default();

	streaming.mark_present_area(NonZeroChunkRegion::from_single(chunk));
	streaming.fetch_needed(&mut sources, grid, &mut consumer, chunk);

	assert_eq!(streaming.state(chunk), Some(voxel_streaming::ChunkState::InFlight));
	assert_eq!(streaming.presence().request_count(chunk), 1);
	assert!(consumer.needed().get(&grid).is_some_and(|chunks| chunks.contains(&chunk)));

	streaming.presence_mut().set_state(chunk, voxel_streaming::ChunkState::Loaded);
	streaming.release_needed(&mut sources, grid, &mut consumer, chunk);

	assert_eq!(streaming.presence().request_count(chunk), 0);
	assert!(!consumer.needed().get(&grid).is_some_and(|chunks| chunks.contains(&chunk)));
}

#[test]
fn releasing_the_last_inflight_request_restores_available_state() {
	let grid = Entity::PLACEHOLDER;
	let chunk = IVec3::new(2, 0, -1);
	let mut sources = SourceManager::default();
	let mut streaming = GridStreaming::default();
	let mut consumer = TestConsumer::default();

	streaming.mark_present_area(NonZeroChunkRegion::from_single(chunk));
	streaming.fetch_needed(&mut sources, grid, &mut consumer, chunk);
	streaming.release_needed(&mut sources, grid, &mut consumer, chunk);

	assert_eq!(streaming.state(chunk), Some(voxel_streaming::ChunkState::Available));
	assert_eq!(consumer.outstanding(), 0);
}

#[test]
fn fetch_needed_for_already_loaded_chunk_does_not_enqueue_another_source_request() {
	let grid = Entity::PLACEHOLDER;
	let chunk = IVec3::new(2, 0, -1);
	let mut sources = SourceManager::default();
	let mut streaming = GridStreaming::default();
	let mut consumer = TestConsumer::default();

	streaming.mark_present_area(NonZeroChunkRegion::from_single(chunk));
	streaming.presence_mut().set_state(chunk, voxel_streaming::ChunkState::Loaded);
	streaming.fetch_needed(&mut sources, grid, &mut consumer, chunk);

	assert_eq!(streaming.state(chunk), Some(voxel_streaming::ChunkState::Loaded));
	assert_eq!(streaming.presence().request_count(chunk), 1);
	assert_eq!(consumer.outstanding(), 0);
}
