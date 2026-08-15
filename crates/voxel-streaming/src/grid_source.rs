use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock, RwLock};

use bevy::prelude::*;
use crossbeam_channel::{Sender, bounded};
use voxel_data::grid::{Grid, GridId, reconcile_subgrids};
use voxel_data::grid_tree::NonZeroVoxelRegion;
use voxel_data::splat::{GridSplat, splat_voxels_blocking};
use voxel_data::subgrid::SubGrid;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_edit::{GridEdit, apply_grid_edit};
use voxel_sources::{CancellationToken, ChunkSource, SourceCoverage, SourceHandle, TakeJob};

use tile_data::{ChunkRegion, chunk_origin};
use tile_data::CHUNK_SIZE;

struct ChunkRequest {
	grid: GridId,
	chunk: IVec3,
	generation: u64,
	cancellation: CancellationToken,
}

struct TransferRequest {
	destination: voxel_sources::SourceId,
	grid: GridId,
	chunk: IVec3,
	generation: u64,
	response: Sender<()>,
}

struct ReceiveTakenRequest {
	grid: GridId,
	chunk: IVec3,
	voxels: Option<Voxels>,
}

struct VoxelRequest {
	grid: GridId,
	region: ChunkRegion,
	lod: f32,
	voxel_type: VoxelTypeId,
	generation: u64,
	cancellation: CancellationToken,
}

struct GridSourceState {
	handle: OnceLock<SourceHandle>,
	owned: RwLock<HashSet<(GridId, IVec3)>>,
	served: RwLock<HashSet<(GridId, IVec3)>>,
	chunk_requests: Mutex<VecDeque<ChunkRequest>>,
	transfer_requests: Mutex<VecDeque<TransferRequest>>,
	voxel_requests: Mutex<VecDeque<VoxelRequest>>,
	receive_taken_requests: Mutex<VecDeque<ReceiveTakenRequest>>,
	pending_edits: Mutex<HashMap<(GridId, IVec3), Vec<GridEdit>>>,
}

impl Default for GridSourceState {
	fn default() -> Self {
		Self {
			handle: OnceLock::new(),
			owned: RwLock::new(HashSet::new()),
			served: RwLock::new(HashSet::new()),
			chunk_requests: Mutex::new(VecDeque::new()),
			transfer_requests: Mutex::new(VecDeque::new()),
			voxel_requests: Mutex::new(VecDeque::new()),
			receive_taken_requests: Mutex::new(VecDeque::new()),
			pending_edits: Mutex::new(HashMap::new()),
		}
	}
}

/// The single source backed directly by resident ECS [`Grid`] components.
/// Voxel bytes remain in the Grid; this resource only stores routing state and
/// synchronized requests for the Bevy system that may access those bytes.
#[derive(Clone, Default, Resource)]
pub(crate) struct StreamingGridSource {
	state: Arc<GridSourceState>,
}

impl StreamingGridSource {
	pub(crate) fn handle(&self) -> Option<&SourceHandle> {
		self.state.handle.get()
	}

	pub(crate) fn is_ready(&self, grid: GridId, chunk: IVec3) -> bool {
		self.state.served.read().unwrap().contains(&(grid, chunk))
	}

	pub(crate) fn presence(&self, grid: GridId, min: IVec3, size: IVec3) {
		self.state.handle.get().expect("streaming grid source was not initialized").presence(grid, min, size);
	}

	pub(crate) fn queue_edit(&self, grid: GridId, chunk: IVec3, edit: GridEdit) {
		self.state.pending_edits.lock().unwrap().entry((grid, chunk)).or_default().push(edit);
	}

	pub(crate) fn edited(&self, grid: GridId, min: IVec3, size: IVec3, edit: GridEdit) {
		let handle = self.state.handle.get().expect("streaming grid source was not initialized");
		handle.edited(grid, min, size, edit);
	}

}

impl ChunkSource for StreamingGridSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.state.handle.set(handle);
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64, cancellation: CancellationToken) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		if !self.state.owned.read().unwrap().contains(&(grid, chunk)) { return SourceCoverage::None; }
		self.state.chunk_requests.lock().unwrap().push_back(ChunkRequest { grid, chunk, generation, cancellation });
		SourceCoverage::All
	}

	fn request_voxel_area(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		generation: u64,
		cancellation: CancellationToken,
	) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }
		let owned = self.state.owned.read().unwrap();
		let mut owned_count = 0;
		for z in 0..size.z { for y in 0..size.y { for x in 0..size.x {
			owned_count += owned.contains(&(grid, min + IVec3::new(x, y, z))) as usize;
		}}}
		let coverage = SourceCoverage::from_count(owned_count, (size.x * size.y * size.z) as usize);
		if coverage == SourceCoverage::None { return coverage; }
		self.state.voxel_requests.lock().unwrap().push_back(VoxelRequest {
			grid,
			region: ChunkRegion::new(min, size.as_uvec3()),
			lod,
			voxel_type,
			generation,
			cancellation,
		});
		coverage
	}

	fn request_available_area(&self, grid: GridId) {
		if let Some(handle) = self.state.handle.get() { handle.presence_loaded(grid); }
	}

	fn take(&self, destination: voxel_sources::SourceId, grid: GridId, min: IVec3, size: IVec3, generation: u64) -> Vec<TakeJob> {
		let chunks = {
			let mut owned = self.state.owned.write().unwrap();
			let mut served = self.state.served.write().unwrap();
			let mut chunks = Vec::new();
			for z in min.z..min.z + size.z { for y in min.y..min.y + size.y { for x in min.x..min.x + size.x {
				let chunk = IVec3::new(x, y, z);
				if owned.remove(&(grid, chunk)) {
					served.remove(&(grid, chunk));
					chunks.push(chunk);
				}
			}}}
			chunks
		};
		let source = self.clone();
		chunks.into_iter().map(|chunk| TakeJob::new(chunk, {
			let source = source.clone();
			move || {
				let (response, received) = bounded(1);
				source.state.transfer_requests.lock().unwrap().push_back(TransferRequest {
					destination, grid, chunk, generation, response,
				});
				let _ = received.recv();
			}
		})).collect()
	}

	fn begin_take(&self, grid: GridId, min: IVec3, size: IVec3) -> Vec<IVec3> {
		let mut owned = self.state.owned.write().unwrap();
		let mut claimed = Vec::new();
		for z in min.z..min.z + size.z { for y in min.y..min.y + size.y { for x in min.x..min.x + size.x {
			let chunk = IVec3::new(x, y, z);
			if owned.insert((grid, chunk)) { claimed.push(chunk); }
		}}}
		claimed
	}

	fn receive_taken(&self, grid: GridId, chunk: IVec3, voxels: Option<Voxels>) {
		assert!(self.state.owned.read().unwrap().contains(&(grid, chunk)), "received data for a chunk the grid source does not own");
		self.state.receive_taken_requests.lock().unwrap().push_back(ReceiveTakenRequest { grid, chunk, voxels });
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		let key = (grid, chunk);
		self.state.owned.write().unwrap().remove(&key);
		self.state.served.write().unwrap().remove(&key);
		self.state.pending_edits.lock().unwrap().remove(&key);
	}
}

fn chunk_voxel_region(chunk: IVec3) -> NonZeroVoxelRegion {
	NonZeroVoxelRegion::from_min_size(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE)).unwrap()
}

pub(crate) fn serve_grid_source_requests(
	mut commands: Commands,
	source: Res<StreamingGridSource>,
	mut grids: Query<(&mut Grid, &mut crate::GridStreaming)>,
	mut sub_grids: Query<&mut SubGrid>,
	mut consumers: Query<&mut dyn crate::ChunkConsumer>,
	mut chunk_resolved: MessageWriter<crate::ChunkLoadResolved>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let mut touched_by_grid: HashMap<GridId, HashSet<IVec3>> = HashMap::new();
	let mut resolved_chunks = Vec::new();

	let received: Vec<_> = source.state.receive_taken_requests.lock().unwrap().drain(..).collect();
	for request in received {
		let mut pending = source.state.pending_edits.lock().unwrap();
		let (mut grid, mut streaming) = grids.get_mut(request.grid).expect("grid source took a chunk for a missing grid");
		let touched = match request.voxels {
			Some(voxels) => {
				let replace = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::splat(CHUNK_SIZE)).unwrap();
				let splat = GridSplat { grid: 0, base: chunk_origin(request.chunk), voxels: &voxels, replace: Some(replace) };
				let mut touched = splat_voxels_blocking(std::slice::from_mut(grid.as_mut()), std::slice::from_ref(&splat));
				touched.remove(&0).unwrap_or_default()
			}
			None => grid.set_area(chunk_origin(request.chunk), IVec3::splat(CHUNK_SIZE), None),
		};
		touched_by_grid.entry(request.grid).or_default().extend(touched);
		if let Some(edits) = pending.remove(&(request.grid, request.chunk)) {
			for edit in edits {
				if let Some(edit) = edit.clipped_to(chunk_voxel_region(request.chunk)) {
					touched_by_grid.entry(request.grid).or_default().extend(apply_grid_edit(grid.as_mut(), &edit));
				}
			}
			let was_inflight = matches!(streaming.state(request.chunk), Some(crate::ChunkState::InFlight));
			if was_inflight { streaming.finish_chunk_request(request.chunk); }
			streaming.presence.set_state(request.chunk, crate::ChunkState::InternalDirty);
			streaming.newly_dirty.push(request.chunk);
			if streaming.pending_newly_present_edits.remove(&request.chunk) {
				streaming.newly_present_dirty.push(request.chunk);
			}
			if was_inflight { resolved_chunks.push((request.grid, request.chunk)); }
		}
		drop(pending);
		source.state.served.write().unwrap().insert((request.grid, request.chunk));
	}

	for (grid_id, touched) in touched_by_grid {
		let Ok((mut grid, _)) = grids.get_mut(grid_id) else { continue };
		reconcile_subgrids(grid_id, grid.as_mut(), touched, &mut commands, &mut sub_grids);
	}
	for (grid_id, chunk) in resolved_chunks {
		for mut entity_consumers in consumers.iter_mut() {
			for mut consumer in &mut entity_consumers {
				if consumer.needed().get(&grid_id).is_some_and(|set| set.contains(&chunk)) {
					*consumer.outstanding_mut() = consumer.outstanding().saturating_sub(1);
				}
			}
		}
		let visible = grids.get(grid_id).is_ok_and(|(grid, _)| {
			!grid.read_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE)).is_empty()
		});
		chunk_resolved.write(crate::ChunkLoadResolved { grid: grid_id, chunk, visible });
	}

	for request in source.state.transfer_requests.lock().unwrap().drain(..) {
		let voxels = grids.get(request.grid).ok().map(|(grid, _)| grid.read_area(chunk_origin(request.chunk), IVec3::splat(CHUNK_SIZE)));
		handle.transferred(request.destination, request.grid, request.chunk, request.generation, voxels);
		let _ = request.response.send(());
	}

	let mut chunk_requests = source.state.chunk_requests.lock().unwrap();
	for _ in 0..chunk_requests.len() {
		let request = chunk_requests.pop_front().unwrap();
		if request.cancellation.is_cancelled() { continue; }
		if !source.state.served.read().unwrap().contains(&(request.grid, request.chunk)) {
			chunk_requests.push_back(request);
			continue;
		}
		let voxels = grids.get(request.grid).ok().map(|(grid, _)| grid.read_area(chunk_origin(request.chunk), IVec3::splat(CHUNK_SIZE)));
		let actual = handle.region_generation(request.grid, request.chunk, IVec3::ONE).max(request.generation);
		handle.loaded(request.grid, request.chunk, actual, voxels);
	}
	drop(chunk_requests);

	let mut voxel_requests = source.state.voxel_requests.lock().unwrap();
	for _ in 0..voxel_requests.len() {
		let request = voxel_requests.pop_front().unwrap();
		if request.cancellation.is_cancelled() { continue; }
		let owned = source.state.owned.read().unwrap();
		let served = source.state.served.read().unwrap();
		let all_owned_ready = (request.region.min().z..request.region.end().z).all(|z| {
			(request.region.min().y..request.region.end().y).all(|y| {
				(request.region.min().x..request.region.end().x).all(|x| {
					let key = (request.grid, IVec3::new(x, y, z));
					!owned.contains(&key) || served.contains(&key)
				})
			})
		});
		if !all_owned_ready {
			drop(served);
			drop(owned);
			voxel_requests.push_back(request);
			continue;
		}
		let voxels = grids.get(request.grid).ok().and_then(|(grid, _)| {
			if request.lod <= 0.0 && grid.voxel_type_info().id == request.voxel_type {
				let mut out = Voxels::new_with_type(grid.voxel_type_info());
				for z in 0..request.region.size().z { for y in 0..request.region.size().y { for x in 0..request.region.size().x {
					let offset = UVec3::new(x, y, z).as_ivec3();
					let chunk = request.region.min() + offset;
					if served.contains(&(request.grid, chunk)) {
						let voxels = grid.read_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE));
						out.merge_from(&voxels, offset * CHUNK_SIZE);
					}
				}}}
				return (!out.is_empty()).then_some(out);
			}
			let generator = handle.voxel_lod_generator(grid.voxel_type_info().id, request.voxel_type)?;
			generator.generate(request.region.min(), request.region.size().as_ivec3(), request.lod, &|chunk| {
				served.contains(&(request.grid, chunk)).then(|| grid.read_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE)))
			})
		});
		drop(served);
		drop(owned);
		if !request.cancellation.is_cancelled() {
			let actual = handle.region_generation(request.grid, request.region.min(), request.region.size().as_ivec3()).max(request.generation);
			handle.voxels_loaded(
				request.grid,
				request.region.min(),
				request.region.size().as_ivec3(),
				request.lod,
				request.voxel_type,
				actual,
				voxels,
			);
		}
	}
}
