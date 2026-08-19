use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex, OnceLock, RwLock};

use bevy::prelude::*;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_data::{
	grid::{Grid, GridId},
	voxels::VoxelTypeId,
};
use voxel_sources::{ChunkSource, RequestId, SourceCoverage, SourceHandle};
use voxel_tasks::CancellationToken;

struct VoxelRequest {
	request_id: RequestId,
	grid: GridId,
	region: NonZeroChunkRegion,
	cancellation: CancellationToken,
}

struct GridSourceState {
	handle: OnceLock<SourceHandle>,
	owned: RwLock<HashSet<(GridId, IVec3)>>,
	served: RwLock<HashSet<(GridId, IVec3)>>,
	voxel_requests: Mutex<VecDeque<VoxelRequest>>,
}

impl Default for GridSourceState {
	fn default() -> Self {
		Self {
			handle: OnceLock::new(),
			owned: RwLock::new(HashSet::new()),
			served: RwLock::new(HashSet::new()),
			voxel_requests: Mutex::new(VecDeque::new()),
		}
	}
}

#[derive(Clone, Default, Resource)]
pub(crate) struct StreamingGridSource {
	state: Arc<GridSourceState>,
}

impl StreamingGridSource {
	pub(crate) fn handle(&self) -> Option<&SourceHandle> {
		self.state.handle.get()
	}

	pub(crate) fn claim(&self, grid: GridId, region: NonZeroChunkRegion) {
		let mut owned = self.state.owned.write().unwrap();
		let mut served = self.state.served.write().unwrap();
		for chunk in chunks(region) {
			owned.insert((grid, chunk));
			served.insert((grid, chunk));
		}
	}
}

impl ChunkSource for StreamingGridSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.state.handle.set(handle);
	}

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		_lod: u8,
		_voxel_type: Option<VoxelTypeId>,
	) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None; }

		let owned = self.state.owned.read().unwrap();
		let owned_count = chunks(region).filter(|chunk| owned.contains(&(grid, *chunk))).count();
		let total = region.size().x as usize * region.size().y as usize * region.size().z as usize;
		let coverage = match owned_count {
			0 => SourceCoverage::None,
			count if count == total => SourceCoverage::All,
			_ => SourceCoverage::Some,
		};
		if coverage != SourceCoverage::None {
			self.state.voxel_requests.lock().unwrap().push_back(VoxelRequest {
				request_id,
				grid,
				region,
				cancellation: cancellation.clone(),
			});
		}
		coverage
	}

	fn request_presence(
		&self,
		request_id: RequestId,
		_cancellation: CancellationToken,
		grid: GridId,
	) {
		let Some(handle) = self.state.handle.get() else { return };
		let owned: Vec<_> = self.state.owned.read().unwrap().iter()
			.filter_map(|(owned_grid, chunk)| (*owned_grid == grid).then_some(*chunk))
			.collect();
		for chunk in owned {
			handle.presence(request_id, grid, NonZeroChunkRegion::from_single(chunk));
		}
		handle.presence_loaded(request_id);
	}

	fn take_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		let mut owned = self.state.owned.write().unwrap();
		let mut served = self.state.served.write().unwrap();
		for chunk in chunks(region) {
			owned.remove(&(grid, chunk));
			served.remove(&(grid, chunk));
		}
	}
}

fn chunks(region: NonZeroChunkRegion) -> impl Iterator<Item = IVec3> {
	(region.min().z..region.end().z).flat_map(move |z| {
		(region.min().y..region.end().y).flat_map(move |y| {
			(region.min().x..region.end().x).map(move |x| IVec3::new(x, y, z))
		})
	})
}

pub(crate) fn serve_grid_source_requests(
	source: Res<StreamingGridSource>,
	grids: Query<(&Grid, &crate::GridStreaming)>,
) {
	let Some(handle) = source.state.handle.get() else { return };
	let mut requests = source.state.voxel_requests.lock().unwrap();
	for _ in 0..requests.len() {
		let request = requests.pop_front().unwrap();
		if request.cancellation.is_cancelled() { continue; }

		let owned = source.state.owned.read().unwrap();
		let served = source.state.served.read().unwrap();
		let ready = chunks(request.region).all(|chunk| {
			!owned.contains(&(request.grid, chunk)) || served.contains(&(request.grid, chunk))
		});
		if !ready {
			drop(served);
			drop(owned);
			requests.push_back(request);
			continue;
		}

		if let Ok((grid, streaming)) = grids.get(request.grid) {
			for chunk in chunks(request.region) {
				if !served.contains(&(request.grid, chunk)) { continue; }
				handle.voxels(
					request.request_id,
					request.grid,
					NonZeroChunkRegion::from_single(chunk),
					0,
					streaming.chunk_generation(chunk),
					grid.read_area(chunk_origin(chunk), IVec3::splat(CHUNK_SIZE)),
				);
			}
		}
		drop(served);
		drop(owned);
		if !request.cancellation.is_cancelled() {
			handle.voxels_loaded(request.request_id);
		}
	}
}
