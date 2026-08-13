use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock, RwLock};

use bevy::prelude::*;
use crossbeam_channel::{Sender, bounded};
use voxel_data::grid::{Grid, GridId, reconcile_subgrids};
use voxel_data::grid_tree::NonZeroVoxelRegion;
use voxel_data::splat::{GridSplat, splat_voxels_blocking};
use voxel_data::subgrid::SubGrid;
use voxel_data::voxels::{VoxelTypeId, Voxels};
use voxel_sources::{CancellationToken, ChunkSource, LendResult, SourceHandle};

use tile_data::{ChunkRegion, chunk_origin};
use tile_data::CHUNK_SIZE;

struct ChunkRequest {
	grid: GridId,
	chunk: IVec3,
	response: Sender<Option<Voxels>>,
}

struct LendRequest {
	grid: GridId,
	chunk: IVec3,
	response: Sender<LendResult>,
}

struct AcceptRequest {
	grid: GridId,
	chunk: IVec3,
	voxels: Option<Voxels>,
	response: Sender<bool>,
}

struct VoxelRequest {
	grid: GridId,
	region: ChunkRegion,
	lod: f32,
	voxel_type: VoxelTypeId,
	response: Sender<Option<Voxels>>,
}

#[derive(Default)]
struct GridSourceState {
	handle: OnceLock<SourceHandle>,
	served: RwLock<HashSet<(GridId, IVec3)>>,
	lent: RwLock<HashSet<(GridId, IVec3)>>,
	borrowed: RwLock<HashSet<(GridId, IVec3)>>,
	chunk_requests: Mutex<VecDeque<ChunkRequest>>,
	voxel_requests: Mutex<VecDeque<VoxelRequest>>,
	lend_requests: Mutex<VecDeque<LendRequest>>,
	accept_requests: Mutex<VecDeque<AcceptRequest>>,
}

/// The single source backed directly by resident ECS [`Grid`] components.
/// Voxel bytes remain in the Grid; this resource only stores routing state and
/// synchronized requests for the Bevy system that may access those bytes.
#[derive(Clone, Default, Resource)]
pub(crate) struct StreamingGridSource {
	state: Arc<GridSourceState>,
}

impl StreamingGridSource {
	pub(crate) fn id(&self) -> Option<voxel_sources::SourceId> {
		self.state.handle.get().map(SourceHandle::id)
	}

	pub(crate) fn presence(&self, grid: GridId, min: IVec3, size: IVec3) {
		self.state.handle.get().expect("streaming grid source was not initialized").presence(grid, min, size);
	}

	pub(crate) fn edited(&self, grid: GridId, min: IVec3, size: IVec3, edit: voxel_edit::GridEdit) {
		let served = self.state.served.read().unwrap();
		for z in min.z..min.z + size.z { for y in min.y..min.y + size.y { for x in min.x..min.x + size.x {
			assert!(served.contains(&(grid, IVec3::new(x, y, z))), "grid source edited a chunk it does not own or borrow");
		}}}
		drop(served);
		let handle = self.state.handle.get().expect("streaming grid source was not initialized");
		handle.edited(grid, min, size, edit);
	}

}

impl ChunkSource for StreamingGridSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.state.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		self.state.served.read().unwrap().contains(&(grid, chunk)).then_some(0)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, edit_index: u64, cancellation: CancellationToken) -> bool {
		if cancellation.is_cancelled() { return false; }
		let served = self.state.served.read().unwrap();
		if !served.contains(&(grid, chunk)) { return false; }
		let (response, received) = bounded(1);
		self.state.chunk_requests.lock().unwrap().push_back(ChunkRequest { grid, chunk, response });
		drop(served);
		let Ok(voxels) = received.recv() else { return false };
		if !cancellation.is_cancelled() { self.state.handle.get().unwrap().loaded(grid, chunk, edit_index, voxels); }
		true
	}

	fn cost_voxels(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32, _voxel_type: VoxelTypeId) -> Option<u32> {
		let served = self.state.served.read().unwrap();
		(0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| served.contains(&(grid, min + IVec3::new(x, y, z)))))).then_some(0)
	}

	fn request_voxel_area(
		&self,
		grid: GridId,
		min: IVec3,
		size: IVec3,
		lod: f32,
		voxel_type: VoxelTypeId,
		edit_index: u64,
		cancellation: CancellationToken,
	) -> bool {
		if cancellation.is_cancelled() { return false; }
		let served = self.state.served.read().unwrap();
		let owns_any = (0..size.z).any(|z| (0..size.y).any(|y| (0..size.x).any(|x| {
			served.contains(&(grid, min + IVec3::new(x, y, z)))
		})));
		if !owns_any { return false; }
		let (response, received) = bounded(1);
		self.state.voxel_requests.lock().unwrap().push_back(VoxelRequest { grid, region: ChunkRegion::new(min, size.as_uvec3()), lod, voxel_type, response });
		drop(served);
		let Ok(voxels) = received.recv() else { return false };
		if !cancellation.is_cancelled() {
			self.state.handle.get().unwrap().voxels_loaded(grid, min, size, lod, voxel_type, edit_index, voxels);
		}
		true
	}

	fn request_available_area(&self, grid: GridId) {
		if let Some(handle) = self.state.handle.get() { handle.presence_loaded(grid); }
	}

	fn lend(&self, grid: GridId, chunk: IVec3, _cancellation: CancellationToken) -> LendResult {
		if !self.state.served.write().unwrap().remove(&(grid, chunk)) { return LendResult::Unavailable; }
		self.state.lent.write().unwrap().insert((grid, chunk));
		let (response, received) = bounded(1);
		self.state.lend_requests.lock().unwrap().push_back(LendRequest { grid, chunk, response });
		match received.recv() {
			Ok(LendResult::Borrowed(voxels)) => LendResult::Borrowed(voxels),
			_ => {
				self.state.lent.write().unwrap().remove(&(grid, chunk));
				self.state.served.write().unwrap().insert((grid, chunk));
				LendResult::Unavailable
			}
		}
	}

	fn accept_borrow(&self, grid: GridId, chunk: IVec3, voxels: Option<Voxels>) -> bool {
		let key = (grid, chunk);
		if self.state.borrowed.read().unwrap().contains(&key) { return true; }
		let (response, received) = bounded(1);
		self.state.accept_requests.lock().unwrap().push_back(AcceptRequest { grid, chunk, voxels, response });
		let accepted = received.recv().unwrap_or(false);
		if accepted { self.state.borrowed.write().unwrap().insert(key); }
		accepted
	}

	fn create_owned(&self, grid: GridId, chunk: IVec3) -> bool {
		let key = (grid, chunk);
		if self.state.served.read().unwrap().contains(&key) { return true; }
		let (response, received) = bounded(1);
		self.state.accept_requests.lock().unwrap().push_back(AcceptRequest { grid, chunk, voxels: None, response });
		received.recv().unwrap_or(false)
	}

	fn return_area(&self, grid: GridId, min: IVec3, size: IVec3) {
		let mut lent = self.state.lent.write().unwrap();
		let mut borrowed = self.state.borrowed.write().unwrap();
		let mut served = self.state.served.write().unwrap();
		for z in min.z..min.z + size.z { for y in min.y..min.y + size.y { for x in min.x..min.x + size.x {
			let key = (grid, IVec3::new(x, y, z));
			if lent.remove(&key) { served.insert(key); }
			if borrowed.remove(&key) { served.remove(&key); }
		}}}
	}

	fn forget(&self, grid: GridId, chunk: IVec3) {
		let key = (grid, chunk);
		self.state.served.write().unwrap().remove(&key);
		self.state.lent.write().unwrap().remove(&key);
		self.state.borrowed.write().unwrap().remove(&key);
	}
}

pub(crate) fn serve_grid_source_requests(
	mut commands: Commands,
	source: Res<StreamingGridSource>,
	mut grids: Query<&mut Grid>,
	mut sub_grids: Query<&mut SubGrid>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	for request in source.state.lend_requests.lock().unwrap().drain(..) {
		let result = grids
			.get(request.grid)
			.ok()
			.map(|grid| LendResult::Borrowed(Some(grid.read_area(chunk_origin(request.chunk), IVec3::splat(CHUNK_SIZE)))))
			.unwrap_or(LendResult::Unavailable);
		let _ = request.response.send(result);
	}

	for request in source.state.accept_requests.lock().unwrap().drain(..) {
		let success = if let Ok(mut grid) = grids.get_mut(request.grid) {
			let touched = match request.voxels {
				Some(voxels) => {
					let replace = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::splat(CHUNK_SIZE)).unwrap();
					let splat = GridSplat { grid: 0, base: chunk_origin(request.chunk), voxels: &voxels, replace: Some(replace) };
					let mut touched = splat_voxels_blocking(std::slice::from_mut(grid.as_mut()), std::slice::from_ref(&splat));
					touched.remove(&0).unwrap_or_default()
				}
				None => grid.set_area(chunk_origin(request.chunk), IVec3::splat(CHUNK_SIZE), None),
			};
			reconcile_subgrids(request.grid, grid.as_mut(), touched, &mut commands, &mut sub_grids);
			source.state.served.write().unwrap().insert((request.grid, request.chunk));
			true
		} else { false };
		let _ = request.response.send(success);
	}

	let served = source.state.served.read().unwrap();
	for request in source.state.chunk_requests.lock().unwrap().drain(..) {
		let voxels = if served.contains(&(request.grid, request.chunk)) {
			grids.get(request.grid).ok().map(|grid| grid.read_area(chunk_origin(request.chunk), IVec3::splat(CHUNK_SIZE)))
		} else {
			None
		};
		let _ = request.response.send(voxels);
	}

	for request in source.state.voxel_requests.lock().unwrap().drain(..) {
		let voxels = grids.get(request.grid).ok().and_then(|grid| {
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
		let _ = request.response.send(voxels);
	}
}
