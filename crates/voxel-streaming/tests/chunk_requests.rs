use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_sources::{ChunkLoadRequest, LodLoadRequest, PresenceLoadRequest, VoxelSourceRequestApi};
use voxel_streaming::{ChunkConsumer, GridStreaming, LodLoadResult};

#[derive(Default)]
struct TestConsumer {
	needed: HashMap<GridId, HashSet<IVec3>>,
	outstanding: usize,
	lod_results: Vec<LodLoadResult>,
}

impl ChunkConsumer for TestConsumer {
	fn needed(&self) -> &HashMap<GridId, HashSet<IVec3>> { &self.needed }
	fn needed_mut(&mut self) -> &mut HashMap<GridId, HashSet<IVec3>> { &mut self.needed }
	fn outstanding(&self) -> usize { self.outstanding }
	fn outstanding_mut(&mut self) -> &mut usize { &mut self.outstanding }
	fn push_lod(&mut self, result: LodLoadResult) { self.lod_results.push(result); }
	fn drain_lod(&mut self) -> Vec<LodLoadResult> { std::mem::take(&mut self.lod_results) }
}

#[derive(Default)]
struct TestRequests {
	chunk_sent: AtomicU64,
}

impl VoxelSourceRequestApi for TestRequests {
	fn request_presence(&self, _request: PresenceLoadRequest) {}
	fn request_chunk(&self, _request: ChunkLoadRequest) {
		self.chunk_sent.fetch_add(1, Ordering::Relaxed);
	}
	fn request_lod(&self, _request: LodLoadRequest) {}
	fn chunk_requests_sent(&self) -> u64 { self.chunk_sent.load(Ordering::Relaxed) }
	fn lod_requests_sent(&self) -> u64 { 0 }
}

#[test]
fn chunk_request_releases_cleanly_after_load_when_no_longer_needed() {
	let grid = Entity::PLACEHOLDER;
	let chunk = IVec3::new(2, 0, -1);
	let requests = TestRequests::default();
	let mut streaming = GridStreaming::default();
	let mut consumer = TestConsumer::default();

	streaming.presence_mut().mark_present(chunk);
	streaming.fetch_needed(grid, &mut consumer, &requests, chunk);

	assert_eq!(requests.chunk_requests_sent(), 1);
	assert_eq!(streaming.presence().request_count(chunk), 1);
	assert!(consumer.needed().get(&grid).is_some_and(|chunks| chunks.contains(&chunk)));

	streaming.presence_mut().set_state(chunk, voxel_streaming::ChunkState::Loaded);
	streaming.release_needed(grid, &mut consumer, chunk);

	assert_eq!(streaming.presence().request_count(chunk), 0);
	assert!(!consumer.needed().get(&grid).is_some_and(|chunks| chunks.contains(&chunk)));
}

#[test]
fn fetch_needed_for_already_loaded_chunk_does_not_enqueue_another_source_request() {
	let grid = Entity::PLACEHOLDER;
	let chunk = IVec3::new(2, 0, -1);
	let requests = TestRequests::default();
	let mut streaming = GridStreaming::default();
	let mut consumer = TestConsumer::default();

	streaming.presence_mut().mark_present(chunk);
	streaming.presence_mut().set_state(chunk, voxel_streaming::ChunkState::Loaded);
	streaming.fetch_needed(grid, &mut consumer, &requests, chunk);

	assert_eq!(requests.chunk_requests_sent(), 0);
	assert_eq!(streaming.presence().request_count(chunk), 1);
	assert_eq!(consumer.outstanding(), 0);
}
