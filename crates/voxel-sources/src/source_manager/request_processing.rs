use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use bevy::math::IVec3;
use tile_data::ChunkRegion;
use crossbeam_channel::Sender;
use tracy_client::span;
use voxel_data::grid::GridId;
use voxel_data::voxels::{VoxelTypeId, VoxelTypeInfo, Voxels};
use voxel_tasks::{AsyncTaskPusher, CancellationToken, PriorityTask};

use super::handle::{ChunksBorrowed, SourceChunkResult, SourceId, SourceMessage, SourceVoxelsResult};
use super::source::{LendResult, SharedSource};
use super::worker::SourceWork;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct VoxelRequestKey {
	grid: GridId,
	region: ChunkRegion,
	lod_bits: u32,
	voxel_type: VoxelTypeId,
}

impl VoxelRequestKey {
	fn new(grid: GridId, region: ChunkRegion, lod: f32, voxel_type: VoxelTypeId) -> Self {
		Self { grid, region, lod_bits: lod.to_bits(), voxel_type }
	}
}

struct PendingVoxelJob {
	required_edit_index: u64,
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
		let key = VoxelRequestKey::new(result.grid, result.region, result.lod, result.voxel_type);
		let mut pending = self.pending_voxels.lock().unwrap();
		let job = pending.get_mut(&key)?;
		if job.cancellation.is_cancelled() {
			pending.remove(&key);
			return None;
		}
		if result.edit_index < job.required_edit_index { return None; }
		match &mut job.kind {
			PendingVoxelJobKind::Direct => {
				pending.remove(&key);
				Some((result.lod, result.voxels, result.edit_index))
			}
			PendingVoxelJobKind::Composite { expected, received } => {
				received.insert(result.source, (result.edit_index, result.voxels));
				if !expected.iter().all(|source| received.contains_key(source)) {
					return None;
				}
				let edit_index = received.values().map(|(edit_index, _)| *edit_index).min().unwrap();
				let parts: Vec<_> = received.values().filter_map(|(_, voxels)| voxels.clone()).collect();
				pending.remove(&key);
				let voxels = parts.first().map(|first| first.voxel_type_info()).map(|info| merge_voxels(info, parts));
				Some((result.lod, voxels.filter(|voxels| !voxels.is_empty()), edit_index))
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
			SourceWork::Chunk { grid, chunk, edit_index, cancellation } => {
				request_chunk(
					grid,
					chunk,
					edit_index,
					cancellation,
					self.sources.clone(),
					&self.pusher,
					self.message_tx.clone(),
				)
			}
			SourceWork::Borrow { request, borrower, grid, min, size, edit_index, cancellation } => {
				borrow_area(request, borrower, grid, min, size, edit_index, cancellation, &self.sources, &self.message_tx)
			}
			SourceWork::Return { grid, min, size } => {
				return_area(grid, min, size, &self.sources);
			}
			SourceWork::Voxels { grid, min, size, lod, voxel_type, priority, edit_index, cancellation } => {
				request_voxels(
					grid,
					min,
					size,
					lod,
					voxel_type,
					priority,
					edit_index,
					cancellation,
					self.sources.clone(),
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
	edit_index: u64,
	cancellation: CancellationToken,
	sources: Arc<[SharedSource]>,
	pusher: &AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
) {
	pusher.push(PriorityTask::new(0.0, async move {
		if cancellation.is_cancelled() { return; }
		if let Some(id) = cheapest(&sources, grid, chunk) {
			let _zone = span!("source request_load chunk");
			if sources[id.0].request_load(grid, chunk, edit_index, cancellation.clone()) { return; }
			// Ownership changed after routing. Re-evaluate once without retaining
			// any manager lock across the source call.
			if let Some(retry) = cheapest(&sources, grid, chunk)
				&& sources[retry.0].request_load(grid, chunk, edit_index, cancellation.clone()) { return; }
		}
		if !cancellation.is_cancelled() {
			let _ = message_tx.send(SourceMessage::Chunk(SourceChunkResult { grid, chunk, edit_index, voxels: None }));
		}
	}));
}

fn borrow_area(
	request: u64,
	borrower: SourceId,
	grid: GridId,
	min: IVec3,
	size: IVec3,
	edit_index: u64,
	cancellation: CancellationToken,
	sources: &[SharedSource],
	message_tx: &Sender<SourceMessage>,
) {
	let target = &sources[borrower.0];
	let mut transferred: Vec<(IVec3, Option<SourceId>)> = Vec::new();
	let mut success = true;

	'area: for z in min.z..min.z + size.z {
		for y in min.y..min.y + size.y {
			for x in min.x..min.x + size.x {
				if cancellation.is_cancelled() { success = false; break 'area; }
				let chunk = IVec3::new(x, y, z);
				if target.cost(grid, chunk).is_some() { continue; }
				let owners: Vec<_> = sources
					.iter()
					.enumerate()
					.filter(|(index, source)| *index != borrower.0 && source.cost(grid, chunk).is_some())
					.map(|(index, _)| SourceId(index))
					.collect();
				assert!(owners.len() <= 1, "multiple voxel sources own {grid:?} chunk {chunk:?}");
				let (voxels, owner) = match owners.first().copied() {
					Some(owner) => match sources[owner.0].lend(grid, chunk, cancellation.clone()) {
						LendResult::Borrowed(voxels) => (voxels, Some(owner)),
						LendResult::Unavailable => { success = false; break 'area; }
					},
					None => (None, None),
				};
				let accepted = if owner.is_some() {
					target.accept_borrow(grid, chunk, voxels)
				} else {
					target.create_owned(grid, chunk)
				};
				if !accepted {
					if let Some(owner) = owner { sources[owner.0].return_area(grid, chunk, IVec3::ONE); }
					success = false;
					break 'area;
				}
				transferred.push((chunk, owner));
			}
		}
	}

	if !success {
		for (chunk, owner) in transferred {
			target.forget(grid, chunk);
			if let Some(owner) = owner { sources[owner.0].return_area(grid, chunk, IVec3::ONE); }
		}
	}
	let _ = message_tx.send(SourceMessage::Borrowed(ChunksBorrowed {
		request,
		borrower,
		grid,
		region: ChunkRegion::new(min, size.as_uvec3()),
		edit_index,
		success,
	}));
}

fn return_area(grid: GridId, min: IVec3, size: IVec3, sources: &[SharedSource]) {
	for source in sources { source.return_area(grid, min, size); }
}

fn request_voxels(
	grid: GridId,
	min: IVec3,
	size: IVec3,
	lod: f32,
	voxel_type: VoxelTypeId,
	priority: f32,
	edit_index: u64,
	cancellation: CancellationToken,
	sources: Arc<[SharedSource]>,
	pending_voxels: Arc<Mutex<HashMap<VoxelRequestKey, PendingVoxelJob>>>,
	pusher: &AsyncTaskPusher,
	message_tx: Sender<SourceMessage>,
) {
	pusher.push(PriorityTask::new(priority, async move {
		if cancellation.is_cancelled() { return; }
		let source_ids = voxel_sources_for_region(&sources, grid, min, size, lod, voxel_type);
		let key = VoxelRequestKey::new(grid, ChunkRegion::new(min, size.as_uvec3()), lod, voxel_type);
		{
			let mut pending = pending_voxels.lock().unwrap();
			if pending.get(&key).is_some_and(|job| edit_index <= job.required_edit_index) { return; }
			if let Some(previous) = pending.remove(&key) { previous.cancellation.cancel(); }
			let kind = if source_ids.len() > 1 {
				PendingVoxelJobKind::Composite {
					expected: source_ids.iter().copied().collect(),
					received: Default::default(),
				}
			} else {
				PendingVoxelJobKind::Direct
			};
			pending.insert(key, PendingVoxelJob { required_edit_index: edit_index, cancellation: cancellation.clone(), kind });
		}

		if source_ids.is_empty() {
			let _ = message_tx.send(SourceMessage::Voxels(SourceVoxelsResult {
				source: SourceId(usize::MAX), grid, region: ChunkRegion::new(min, size.as_uvec3()), lod, voxel_type, edit_index, voxels: None,
			}));
			return;
		}
		for id in source_ids {
			if cancellation.is_cancelled() { return; }
			let _zone = span!("source request_voxel_area");
			sources[id.0].request_voxel_area(grid, min, size, lod, voxel_type, edit_index, cancellation.clone());
		}
	}));
}

fn cheapest(sources: &[SharedSource], grid: GridId, chunk: bevy::math::IVec3) -> Option<SourceId> {
	let owners: Vec<_> = sources
		.iter()
		.enumerate()
		.filter_map(|(i, source)| source.cost(grid, chunk).map(|cost| (cost, SourceId(i))))
		.collect();
	assert!(owners.len() <= 1, "multiple voxel sources own {grid:?} chunk {chunk:?}");
	owners.into_iter().min_by_key(|(cost, _)| *cost).map(|(_, id)| id)
}

fn voxel_sources_for_region(
	sources: &[SharedSource],
	grid: GridId,
	min: bevy::math::IVec3,
	size: bevy::math::IVec3,
	lod: f32,
	voxel_type: VoxelTypeId,
) -> Vec<SourceId> {
	for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
		let chunk = min + IVec3::new(x, y, z);
		let owner_count = sources.iter().filter(|source| source.cost(grid, chunk).is_some()).count();
		assert!(owner_count <= 1, "multiple voxel sources own {grid:?} chunk {chunk:?}");
	}}}
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

	fn voxel_result(source: SourceId, edit_index: u64) -> SourceVoxelsResult {
		SourceVoxelsResult {
			source,
			grid: GridId::PLACEHOLDER,
			region: ChunkRegion::new(IVec3::ZERO, UVec3::ONE),
			lod: 0.0,
			voxel_type: VoxelTypeId(1),
			edit_index,
			voxels: None,
		}
	}

	#[test]
	fn direct_voxel_results_must_meet_the_jobs_edit_index() {
		let state = RequestState::default();
		let key = VoxelRequestKey::new(GridId::PLACEHOLDER, ChunkRegion::new(IVec3::ZERO, UVec3::ONE), 0.0, VoxelTypeId(1));
		state.pending_voxels.lock().unwrap().insert(key, PendingVoxelJob {
			required_edit_index: 5,
			cancellation: CancellationToken::new(),
			kind: PendingVoxelJobKind::Direct,
		});

		assert!(state.take_voxels_completion(voxel_result(SourceId(0), 4)).is_none());
		assert!(state.pending_voxels.lock().unwrap().contains_key(&key));
		assert_eq!(state.take_voxels_completion(voxel_result(SourceId(0), 6)).unwrap().2, 6);
	}

	#[test]
	fn composite_voxel_results_reject_old_contributors_and_publish_the_minimum_edit_index() {
		let state = RequestState::default();
		let key = VoxelRequestKey::new(GridId::PLACEHOLDER, ChunkRegion::new(IVec3::ZERO, UVec3::ONE), 0.0, VoxelTypeId(1));
		state.pending_voxels.lock().unwrap().insert(key, PendingVoxelJob {
			required_edit_index: 5,
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
