use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use bevy::math::IVec3;
use crossbeam_channel::Sender;
use tracy_client::span;
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, VoxelTypeInfo, Voxels};
use voxel_tasks::{AsyncTaskPusher, CancellationToken, PriorityTask};

use super::handle::{SourceChunkResult, SourceId, SourceMessage, SourceVoxelsResult};
use super::source::SharedSource;
use super::worker::SourceWork;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VoxelRequestKey {
	grid: GridId,
	min: IVec3,
	size: IVec3,
	lod_bits: u32,
	voxel_type: VoxelTypeId,
}

impl VoxelRequestKey {
	fn new(grid: GridId, min: IVec3, size: IVec3, lod: f32, voxel_type: VoxelTypeId) -> Self {
		Self { grid, min, size, lod_bits: lod.to_bits(), voxel_type }
	}
}

struct PendingVoxelJob {
	required_generation: u64,
	cancellation: CancellationToken,
	kind: PendingVoxelJobKind,
}

enum PendingVoxelJobKind {
	Direct,
	Composite {
		expected: HashSet<SourceId>,
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
		let key = VoxelRequestKey::new(result.grid, result.min, result.size, result.lod, result.voxel_type);
		let mut pending = self.pending_voxels.lock().unwrap();
		let job = pending.get_mut(&key)?;
		if job.cancellation.is_cancelled() {
			pending.remove(&key);
			return None;
		}
		if result.generation < job.required_generation { return None; }
		match &mut job.kind {
			PendingVoxelJobKind::Direct => {
				pending.remove(&key);
				Some((result.lod, result.voxels, result.generation))
			}
			PendingVoxelJobKind::Composite { expected, received } => {
				received.insert(result.source, (result.generation, result.voxels));
				if !expected.iter().all(|source| received.contains_key(source)) {
					return None;
				}
				let generation = received.values().map(|(generation, _)| *generation).min().unwrap();
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
	pusher: AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
}

impl RequestProcessor {
	pub(super) fn new(
		sources: Arc<[SharedSource]>,
		state: RequestState,
		pusher: AsyncTaskPusher,
		message_tx: Sender<SourceMessage>,
	) -> Self {
		Self { sources, state, pusher, message_tx }
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
					&self.sources,
					&self.pusher,
					&self.message_tx,
				)
			}
			SourceWork::Voxels { grid, min, size, lod, voxel_type, priority, generation, cancellation } => {
				request_voxels(
					grid,
					min,
					size,
					lod,
					voxel_type,
					priority,
					generation,
					cancellation,
					&self.sources,
					&self.state.pending_voxels,
					&self.pusher,
					&self.message_tx,
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
	sources: &[SharedSource],
	pusher: &AsyncTaskPusher,
	message_tx: &Sender<SourceMessage>,
) {
	if cancellation.is_cancelled() { return; }
	if let Some(id) = cheapest(sources, grid, chunk) {
		let source = sources[id.0].clone();
		pusher.push(PriorityTask::new(0.0, async move {
			if cancellation.is_cancelled() { return; }
			let _zone = span!("source request_load chunk");
			source.request_load(grid, chunk, generation, cancellation);
		}));
	} else if !cancellation.is_cancelled() {
		let _ = message_tx.send(SourceMessage::Chunk(SourceChunkResult {
			grid,
			chunk,
			generation,
			voxels: None,
		}));
	}
}

fn request_voxels(
	grid: GridId,
	min: bevy::math::IVec3,
	size: bevy::math::IVec3,
	lod: f32,
	voxel_type: VoxelTypeId,
	priority: f32,
	generation: u64,
	cancellation: CancellationToken,
	sources: &[SharedSource],
	pending_voxels: &Arc<Mutex<HashMap<VoxelRequestKey, PendingVoxelJob>>>,
	pusher: &AsyncTaskPusher,
	message_tx: &Sender<SourceMessage>,
) {
	if cancellation.is_cancelled() { return; }
	let source_ids = voxel_sources_for_region(sources, grid, min, size, lod, voxel_type);
	let key = VoxelRequestKey::new(grid, min, size, lod, voxel_type);

	{
		let mut pending = pending_voxels.lock().unwrap();
		if pending.get(&key).is_some_and(|job| generation <= job.required_generation) {
			return;
		}

		if let Some(previous) = pending.remove(&key) {
			previous.cancellation.cancel();
		}

		let kind = if source_ids.len() > 1 {
			PendingVoxelJobKind::Composite {
				expected: source_ids.iter().copied().collect(),
				received: Default::default(),
			}
		} else {
			PendingVoxelJobKind::Direct
		};
		pending.insert(key, PendingVoxelJob {
			required_generation: generation,
			cancellation: cancellation.clone(),
			kind,
		});
	}

	match source_ids.as_slice() {
		[] => {
			let _ = message_tx.send(SourceMessage::Voxels(SourceVoxelsResult {
				source: SourceId(usize::MAX),
				grid,
				min,
				size,
				lod,
				voxel_type,
				generation,
				voxels: None,
			}));
		}
		[id] => {
			let source = sources[id.0].clone();
			pusher.push(PriorityTask::new(priority, async move {
				if cancellation.is_cancelled() { return; }
				let _zone = span!("source request_voxel_area direct");
				source.request_voxel_area(grid, min, size, lod, voxel_type, generation, cancellation);
			}));
		}
		_ => {
			for id in source_ids {
				let source = sources[id.0].clone();
				let cancellation = cancellation.clone();
				pusher.push(PriorityTask::new(priority, async move {
					if cancellation.is_cancelled() { return; }
					let _zone = span!("source request_voxel_area composite part");
					source.request_voxel_area(grid, min, size, lod, voxel_type, generation, cancellation);
				}));
			}
		}
	}
}

fn cheapest(sources: &[SharedSource], grid: GridId, chunk: bevy::math::IVec3) -> Option<SourceId> {
	sources
		.iter()
		.enumerate()
		.filter_map(|(i, source)| source.cost(grid, chunk).map(|cost| (cost, SourceId(i))))
		.min_by_key(|(cost, _)| *cost)
		.map(|(_, id)| id)
}

fn voxel_sources_for_region(
	sources: &[SharedSource],
	grid: GridId,
	min: bevy::math::IVec3,
	size: bevy::math::IVec3,
	lod: f32,
	voxel_type: VoxelTypeId,
) -> Vec<SourceId> {
	sources
		.iter()
		.enumerate()
		.filter_map(|(i, source)| source.cost_voxels(grid, min, size, lod, voxel_type).is_some().then_some(SourceId(i)))
		.collect()
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
	use super::*;

	fn voxel_result(source: SourceId, generation: u64) -> SourceVoxelsResult {
		SourceVoxelsResult {
			source,
			grid: GridId::PLACEHOLDER,
			min: IVec3::ZERO,
			size: IVec3::ONE,
			lod: 0.0,
			voxel_type: VoxelTypeId(1),
			generation,
			voxels: None,
		}
	}

	#[test]
	fn direct_voxel_results_must_meet_the_jobs_generation() {
		let state = RequestState::default();
		let key = VoxelRequestKey::new(GridId::PLACEHOLDER, IVec3::ZERO, IVec3::ONE, 0.0, VoxelTypeId(1));
		state.pending_voxels.lock().unwrap().insert(key, PendingVoxelJob {
			required_generation: 5,
			cancellation: CancellationToken::new(),
			kind: PendingVoxelJobKind::Direct,
		});

		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 4)).is_none());
		assert!(state.pending_voxels.lock().unwrap().contains_key(&key));
		assert_eq!(state.take_voxels_completion(voxel_result(SourceId(0), 6)).unwrap().2, 6);
	}

	#[test]
	fn composite_voxel_results_reject_old_contributors_and_publish_the_minimum_generation() {
		let state = RequestState::default();
		let key = VoxelRequestKey::new(GridId::PLACEHOLDER, IVec3::ZERO, IVec3::ONE, 0.0, VoxelTypeId(1));
		state.pending_voxels.lock().unwrap().insert(key, PendingVoxelJob {
			required_generation: 5,
			cancellation: CancellationToken::new(),
			kind: PendingVoxelJobKind::Composite {
				expected: HashSet::from([SourceId(0), SourceId(1)]),
				received: HashMap::new(),
			},
		});

		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 4)).is_none());
		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 6)).is_none());
		assert_eq!(state.take_voxels_completion(voxel_result(SourceId(1), 7)).unwrap().2, 6);
	}
}
