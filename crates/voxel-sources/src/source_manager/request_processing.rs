use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, RwLock};

use bevy::math::IVec3;
use tile_data::NonZeroChunkRegion;
use crossbeam_channel::Sender;
use tracy_client::span;
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, VoxelTypeInfo, Voxels};
use voxel_tasks::{AsyncTaskPusher, CancellationToken, PriorityTask};

use super::handle::{SourceChunkResult, SourceId, SourceMessage, SourceVoxelsResult};
use super::source::{SharedSource, SourceCoverage};
use super::worker::SourceWork;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VoxelRequestKey {
	grid: GridId,
	region: NonZeroChunkRegion,
	lod_bits: u32,
	voxel_type: VoxelTypeId,
}

impl VoxelRequestKey {
	fn new(grid: GridId, region: NonZeroChunkRegion, lod: f32, voxel_type: VoxelTypeId) -> Self {
		Self { grid, region, lod_bits: lod.to_bits(), voxel_type }
	}
}

struct PendingVoxelJob {
	required_generation: u64,
	cancellation: CancellationToken,
	kind: PendingVoxelJobKind,
}

enum PendingVoxelJobKind {
	Composite {
		expected: Option<HashSet<SourceId>>,
		received: HashMap<SourceId, (u64, Option<Voxels>)>,
	},
}

#[derive(Clone, Default)]
pub(super) struct RequestState {
	pending_voxels: Arc<Mutex<HashMap<VoxelRequestKey, PendingVoxelJob>>>,
	active_presence_loads: Arc<Mutex<HashMap<GridId, u32>>>,
}

impl RequestState {
	pub(super) fn retain_active(&self) {
		self.pending_voxels.lock().unwrap().retain(|_, job| !job.cancellation.is_cancelled());
	}

	pub(super) fn finish_presence_load(&self, grid: GridId) -> bool {
		let mut loads = self.active_presence_loads.lock().unwrap();
		let Some(count) = loads.get_mut(&grid) else { return false };
		*count = count.saturating_sub(1);
		if *count == 0 {
			loads.remove(&grid);
			true
		} else {
			false
		}
	}

	pub(super) fn take_voxels_completion(&self, result: SourceVoxelsResult) -> Option<(f32, Option<Voxels>, u64)> {
		let key = VoxelRequestKey::new(result.grid, result.region, result.lod, result.voxel_type);
		let mut pending = self.pending_voxels.lock().unwrap();
		let job = pending.get_mut(&key)?;
		if job.cancellation.is_cancelled() {
			pending.remove(&key);
			return None;
		}
		if result.generation < job.required_generation { return None; }
		match &mut job.kind {
			PendingVoxelJobKind::Composite { expected, received } => {
				if result.source != SourceId(usize::MAX) {
					received.insert(result.source, (result.generation, result.voxels));
				}
				let Some(expected) = expected else { return None };
				if !expected.iter().all(|source| received.contains_key(source)) { return None; }
				let generation = received.values().map(|(generation, _)| *generation).max().unwrap_or(result.generation);
				let parts: Vec<_> = received.values().filter_map(|(_, voxels)| voxels.clone()).collect();
				pending.remove(&key);
				let voxels = parts.first().map(|first| first.voxel_type_info()).map(|info| merge_voxels(info, parts));
				Some((result.lod, voxels.filter(|voxels| !voxels.is_empty()), generation))
			}
		}
	}
}

pub(super) struct RequestProcessor {
	sources: Arc<[SharedSource]>,
	state: RequestState,
	routing: Arc<RwLock<()>>,
	pusher: AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
}

impl RequestProcessor {
	pub(super) fn new(
		sources: Arc<[SharedSource]>,
		state: RequestState,
		routing: Arc<RwLock<()>>,
		pusher: AsyncTaskPusher,
		message_tx: Sender<SourceMessage>,
	) -> Self {
		Self { sources, state, routing, pusher, message_tx }
	}

	pub(super) fn process(&self, request: SourceWork) {
		match request {
			SourceWork::Presence { grid } => {
				request_presence(
					grid,
					&self.sources,
					&self.state.active_presence_loads,
					&self.pusher,
					&self.message_tx,
				)
			}
			SourceWork::Chunk { grid, chunk, generation, cancellation } => {
				request_chunk(
					grid,
					chunk,
					generation,
					cancellation,
					self.sources.clone(),
					self.routing.clone(),
					&self.pusher,
					self.message_tx.clone(),
				)
			}
			SourceWork::Voxels { grid, region, lod, voxel_type, priority, generation, cancellation } => {
				request_voxels(
					grid,
					region,
					lod,
					voxel_type,
					priority,
					generation,
					cancellation,
					self.sources.clone(),
					self.routing.clone(),
					self.state.pending_voxels.clone(),
					&self.pusher,
					self.message_tx.clone(),
				)
			}
		}
	}
}

fn request_presence(
	grid: GridId,
	sources: &[SharedSource],
	active_presence_loads: &Arc<Mutex<HashMap<GridId, u32>>>,
	pusher: &AsyncTaskPusher,
	message_tx: &Sender<SourceMessage>,
) {
	let count = sources.len() as u32;
	if count == 0 {
		*active_presence_loads.lock().unwrap().entry(grid).or_default() += 1;
		let _ = message_tx.send(SourceMessage::PresenceLoaded(grid));
		return;
	}
	*active_presence_loads.lock().unwrap().entry(grid).or_default() += count;
	for source in sources {
		let source = source.clone();
		pusher.push(PriorityTask::new(0.0, async move {
			source.request_available_area(grid);
		}));
	}
}

fn request_chunk(
	grid: GridId,
	chunk: bevy::math::IVec3,
	generation: u64,
	cancellation: CancellationToken,
	sources: Arc<[SharedSource]>,
	routing: Arc<RwLock<()>>,
	pusher: &AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
) {
	pusher.push(PriorityTask::new(0.0, async move {
		if cancellation.is_cancelled() { return; }
		let _routing = routing.read().unwrap();
		let mut owner = None;
		for (index, source) in sources.iter().enumerate() {
			if cancellation.is_cancelled() { return; }
			let _zone = span!("source request_load chunk");
			if source.request_load(grid, chunk, generation, cancellation.clone()).has_any() {
				assert!(owner.replace(SourceId(index)).is_none(), "multiple voxel sources own {grid:?} chunk {chunk:?}");
			}
		}
		if owner.is_none() && !cancellation.is_cancelled() {
			let _ = message_tx.send(SourceMessage::Chunk(SourceChunkResult { grid, chunk, generation, voxels: None }));
		}
	}));
}

pub(super) fn take_area(
	taker: SourceId,
	grid: GridId,
	region: NonZeroChunkRegion,
	generation: u64,
	sources: &[SharedSource],
	routing: &Arc<RwLock<()>>,
	pusher: &AsyncTaskPusher,
) {
	assert!(taker.0 < sources.len(), "invalid taker source ID");
	let jobs = {
		let _routing = routing.write().unwrap();
		let target = &sources[taker.0];
		let claimed: HashSet<_> = target.begin_take(grid, region).into_iter().collect();
		let mut owners = HashMap::new();
		let mut jobs = Vec::new();

		for (index, source) in sources.iter().enumerate() {
			let source_id = SourceId(index);
			if source_id == taker { continue; }
			for job in source.take(grid, region) {
				let chunk = job.chunk();
				assert!(claimed.contains(&chunk), "a voxel source transferred a chunk already owned by the destination");
				assert!(owners.insert(chunk, source_id).is_none(), "multiple voxel sources own {grid:?} chunk {chunk:?}");
				jobs.push(job);
			}
		}
		for chunk in claimed {
			if !owners.contains_key(&chunk) { target.receive_taken(grid, chunk, None); }
		}
		jobs
	};
	for job in jobs {
		pusher.push(PriorityTask::new(0.0, async move { job.run(); }));
	}
}

fn request_voxels(
	grid: GridId,
	region: NonZeroChunkRegion,
	lod: f32,
	voxel_type: VoxelTypeId,
	priority: f32,
	generation: u64,
	cancellation: CancellationToken,
	sources: Arc<[SharedSource]>,
	routing: Arc<RwLock<()>>,
	pending_voxels: Arc<Mutex<HashMap<VoxelRequestKey, PendingVoxelJob>>>,
	pusher: &AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
) {
	pusher.push(PriorityTask::new(priority, async move {
		if cancellation.is_cancelled() { return; }
		let _routing = routing.read().unwrap();
		let key = VoxelRequestKey::new(grid, region, lod, voxel_type);
		{
			let mut pending = pending_voxels.lock().unwrap();
			if pending.get(&key).is_some_and(|job| job.required_generation >= generation) { return; }
			if let Some(previous) = pending.remove(&key) { previous.cancellation.cancel(); }
			pending.insert(key, PendingVoxelJob {
				required_generation: generation,
				cancellation: cancellation.clone(),
				kind: PendingVoxelJobKind::Composite { expected: None, received: Default::default() },
			});
		}

		let mut expected = HashSet::new();
		let mut complete_source = None;
		for (index, source) in sources.iter().enumerate() {
			if cancellation.is_cancelled() { return; }
			let _zone = span!("source request_voxel_area");
			let coverage = source.request_voxel_area(grid, region, lod, voxel_type, generation, cancellation.clone());
			if coverage.has_any() { expected.insert(SourceId(index)); }
			if coverage == SourceCoverage::All {
				assert!(complete_source.replace(SourceId(index)).is_none(), "multiple voxel sources fully own the same area");
			}
		}
		if complete_source.is_some() {
			assert_eq!(expected.len(), 1, "a fully owned voxel area also has partial owners");
		}
		if let Some(job) = pending_voxels.lock().unwrap().get_mut(&key)
			&& let PendingVoxelJobKind::Composite { expected: job_expected, .. } = &mut job.kind {
			*job_expected = Some(expected);
		}
		let _ = message_tx.send(SourceMessage::Voxels(SourceVoxelsResult {
			source: SourceId(usize::MAX), grid, region, lod, voxel_type, generation, voxels: None,
		}));
	}));
}

fn merge_voxels(voxel_type_info: VoxelTypeInfo, parts: Vec<Voxels>) -> Voxels {
	let mut parts = parts.into_iter();
	let Some(mut merged) = parts.next() else {
		return Voxels::new_with_type(voxel_type_info);
	};
	for voxels in parts {
		merged.merge_from(&voxels, IVec3::ZERO);
	}
	merged
}

#[cfg(test)]
mod tests {
	use bevy::math::UVec3;

use super::*;

	fn voxel_result(source: SourceId, generation: u64) -> SourceVoxelsResult {
		SourceVoxelsResult {
			source,
			grid: GridId::PLACEHOLDER,
			region: NonZeroChunkRegion::new(IVec3::ZERO, UVec3::ONE).unwrap(),
			lod: 0.0,
			voxel_type: VoxelTypeId(1),
			generation,
			voxels: None,
		}
	}

	#[test]
	fn direct_voxel_results_must_meet_the_jobs_generation() {
		let state = RequestState::default();
		let key = VoxelRequestKey::new(GridId::PLACEHOLDER, NonZeroChunkRegion::new(IVec3::ZERO, UVec3::ONE).unwrap(), 0.0, VoxelTypeId(1));
		state.pending_voxels.lock().unwrap().insert(key, PendingVoxelJob {
			required_generation: 5,
			cancellation: CancellationToken::new(),
			kind: PendingVoxelJobKind::Composite {
				expected: Some(HashSet::from([SourceId(0)])),
				received: HashMap::new(),
			},
		});

		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 4)).is_none());
		assert!(state.pending_voxels.lock().unwrap().contains_key(&key));
		assert_eq!(state.take_voxels_completion(voxel_result(SourceId(0), 6)).unwrap().2, 6);
	}

	#[test]
	fn composite_voxel_results_reject_old_contributors_and_publish_the_latest_generation() {
		let state = RequestState::default();
		let key = VoxelRequestKey::new(GridId::PLACEHOLDER, NonZeroChunkRegion::new(IVec3::ZERO, UVec3::ONE).unwrap(), 0.0, VoxelTypeId(1));
		state.pending_voxels.lock().unwrap().insert(key, PendingVoxelJob {
			required_generation: 5,
			cancellation: CancellationToken::new(),
			kind: PendingVoxelJobKind::Composite {
				expected: Some(HashSet::from([SourceId(0), SourceId(1)])),
				received: HashMap::new(),
			},
		});

		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 4)).is_none());
		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 6)).is_none());
		assert_eq!(state.take_voxels_completion(voxel_result(SourceId(1), 7)).unwrap().2, 7);
	}
}
