use std::{
	collections::HashMap,
	sync::{Arc, OnceLock, RwLock},
};

use bevy::prelude::*;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunks_covering_nonzero_voxel_region};
use voxel_data::{grid::GridId, voxels::{VoxelTypeId, VoxelTypeInfo, Voxels}};
use voxel_sources::{
	ChunkSource, RequestId, SourceCoverage, SourceHandle, SourceId, SourceManager,
	SourceResult, SourceResultData, VoxelSourcesAppExt,
	edit::{GridEdit, GridGeneration},
};
use voxel_tasks::{AsyncPriorityTaskPool, CancellationToken};

use super::grid_store::{ChunkOwnership, ChunkVersion, GridStore};

#[derive(Default)]
struct StoreState {
	grids: HashMap<GridId, GridStore>,
	acquisitions: HashMap<RequestId, Acquisition>,
	waiting_requests: Vec<ServingRequest>,
}

struct Acquisition {
	grid: GridId,
	chunk: IVec3,
	voxel_type: VoxelTypeInfo,
	generation: GridGeneration,
	baseline: Option<Voxels>,
	queued_edits: Vec<(GridGeneration, Arc<dyn GridEdit>)>,
}

struct ServingRequest {
	request_id: RequestId,
	cancellation: CancellationToken,
	grid: GridId,
	generation: GridGeneration,
	chunks: Vec<(IVec3, Option<Arc<ChunkVersion>>)>,
}

#[derive(Default)]
struct VoxelStoreSourceInner {
	state: RwLock<StoreState>,
	handle: OnceLock<SourceHandle>,
}

#[derive(Resource, Clone, Default)]
pub struct VoxelStoreSource {
	inner: Arc<VoxelStoreSourceInner>,
}

impl VoxelStoreSource {
	/// Inserts unchanged LOD0 chunk data as generation zero owned by this source.
	pub fn insert_chunk_data(&self, grid: GridId, chunk_data: HashMap<IVec3, Voxels>) {
		let mut state = self.inner.state.write().unwrap();
		let store = state.grids.entry(grid).or_default();
		for (chunk, voxels) in chunk_data {
			store.save_chunk(chunk, GridGeneration::default(), &voxels);
		}
	}

	pub fn source_id(&self) -> SourceId {
		self.inner.handle.get().expect("voxel store source was not initialized").id()
	}

	fn grid_available_area(&self, grid: GridId) -> Option<NonZeroChunkRegion> {
		self.inner.state.read().unwrap().grids.get(&grid)?.available_area()
	}

	pub(crate) fn apply_edit(
		&self,
		sources: &mut SourceManager,
		grid: GridId,
		voxel_type: VoxelTypeInfo,
		previous_generation: GridGeneration,
		generation: GridGeneration,
		edit: Arc<dyn GridEdit>,
	) {
		let affected_chunks = chunks_covering_nonzero_voxel_region(edit.affected_region());
		for chunk in chunks(affected_chunks) {
			let unowned = self.inner.state.read().unwrap().grids
				.get(&grid)
				.map_or(true, |store| store.ownership(chunk) == ChunkOwnership::Unowned);
			if !unowned { continue; }

			let region = NonZeroChunkRegion::from_single(chunk);
			let request_id = sources.request_voxels(grid, region, 0, Some(voxel_type.id), previous_generation);
			sources.transfer_ownership(self.source_id(), grid, region);

			let mut state = self.inner.state.write().unwrap();
			state.grids.entry(grid).or_default().begin_acquiring(chunk, request_id);
			state.acquisitions.insert(request_id, Acquisition {
				grid,
				chunk,
				voxel_type,
				generation: previous_generation,
				baseline: None,
				queued_edits: Vec::new(),
			});
		}

		let mut state = self.inner.state.write().unwrap();
		for chunk in chunks(affected_chunks) {
			let ownership = state.grids.entry(grid).or_default().ownership(chunk);
			match ownership {
				ChunkOwnership::Owned => state.grids.get_mut(&grid).unwrap().apply_edit(chunk, voxel_type, generation, edit.as_ref()),
				ChunkOwnership::Acquiring(request_id) => {
					if let Some(acquisition) = state.acquisitions.get_mut(&request_id) {
						acquisition.queued_edits.push((generation, edit.clone()));
					}
				}
				ChunkOwnership::Unowned => {}
			}
		}
	}

	fn receive_acquisition_result(&self, result: &SourceResult) {
		let mut ready = Vec::new();
		{
			let mut state = self.inner.state.write().unwrap();
			match &result.data {
				SourceResultData::Voxels { region, lod, voxels, .. } => {
					let Some(acquisition) = state.acquisitions.get_mut(&result.request_id) else { return };
					if *lod != 0 || !region.contains(acquisition.chunk) { return; }
					let offset = (region.min() - acquisition.chunk) * CHUNK_SIZE as i32;
					acquisition.baseline.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info()))
						.merge_from(voxels, offset);
					return;
				}
				SourceResultData::VoxelsLoaded { .. } => {
					let Some(acquisition) = state.acquisitions.remove(&result.request_id) else { return };
					state.grids.entry(acquisition.grid).or_default().complete_acquisition(
						acquisition.chunk,
						result.request_id,
						acquisition.generation,
						acquisition.baseline.as_ref(),
						acquisition.voxel_type,
						&acquisition.queued_edits,
					);
				}
				SourceResultData::Presence { .. } | SourceResultData::PresenceLoaded => return,
			}

			let mut still_waiting = Vec::new();
			let waiting_requests = std::mem::take(&mut state.waiting_requests);
			for mut request in waiting_requests {
				let mut waiting = false;
				for (chunk, version) in &mut request.chunks {
					if version.is_some() { continue; }
					let Some(store) = state.grids.get(&request.grid) else { waiting = true; continue };
					if matches!(store.ownership(*chunk), ChunkOwnership::Acquiring(_)) {
						waiting = true;
					} else {
						*version = store.version_for(*chunk, request.generation);
					}
				}
				if waiting { still_waiting.push(request) } else { ready.push(request) }
			}
			state.waiting_requests = still_waiting;
			for store in state.grids.values_mut() {
				store.retain_current_versions();
			}
		}
		for request in ready {
			self.dispatch(request);
		}
	}

	fn dispatch(&self, request: ServingRequest) {
		let handle = self.inner.handle.get().expect("voxel store source was not initialized").clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("VoxelStoreSource build").entered();
			for (chunk, version) in request.chunks {
				if request.cancellation.is_cancelled() { break; }
				let Some(voxels) = version.and_then(|version| version.voxels()) else { continue };
				handle.voxels(
					request.request_id,
					request.grid,
					NonZeroChunkRegion::from_single(chunk),
					0,
					request.generation,
					voxels,
				);
			}
			handle.voxels_loaded(request.request_id, request.generation);
		});
	}
}

impl ChunkSource for VoxelStoreSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.inner.handle.set(handle);
	}

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		_lod: u8,
		_voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> SourceCoverage {
		if cancellation.is_cancelled() { return SourceCoverage::None }

		let mut request = ServingRequest {
			request_id,
			cancellation: cancellation.clone(),
			grid,
			generation,
			chunks: Vec::new(),
		};
		let mut waits_for_acquisition = false;
		{
			let state = self.inner.state.read().unwrap();
			let Some(store) = state.grids.get(&grid) else { return SourceCoverage::None };
			for chunk in chunks(region) {
				match store.ownership(chunk) {
					ChunkOwnership::Unowned => {}
					ChunkOwnership::Owned => request.chunks.push((chunk, store.version_for(chunk, generation))),
					ChunkOwnership::Acquiring(_) => {
						waits_for_acquisition = true;
						request.chunks.push((chunk, None));
					}
				}
			}
		}
		if request.chunks.is_empty() { return SourceCoverage::None }
		let coverage = if request.chunks.len() == region.area() as usize { SourceCoverage::All } else { SourceCoverage::Some };
		if waits_for_acquisition {
			self.inner.state.write().unwrap().waiting_requests.push(request);
		} else {
			self.dispatch(request);
		}
		coverage
	}

	fn request_presence(&self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let handle = self.inner.handle.get().expect("voxel store source was not initialized");
		if let Some(region) = self.grid_available_area(grid) {
			handle.presence(request_id, grid, region);
		}
		handle.presence_loaded(request_id);
	}

	fn acquire_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		let mut state = self.inner.state.write().unwrap();
		for chunk in chunks(region) {
			let pending = state.acquisitions.iter()
				.filter_map(|(request_id, acquisition)| {
					(acquisition.grid == grid && acquisition.chunk == chunk).then_some(*request_id)
				})
				.max();
			let store = state.grids.entry(grid).or_default();
			if let Some(request_id) = pending {
				store.begin_acquiring(chunk, request_id);
			} else {
				store.acquire(chunk);
			}
		}
	}

	fn relinquish_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		let mut state = self.inner.state.write().unwrap();
		let Some(store) = state.grids.get_mut(&grid) else { return };
		for chunk in chunks(region) {
			store.relinquish(chunk);
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

pub fn complete_voxel_store_acquisitions(
	store: Res<VoxelStoreSource>,
	mut results: MessageReader<SourceResult>,
) {
	for result in results.read() {
		store.receive_acquisition_result(result);
	}
}

#[derive(Default)]
pub struct VoxelStoreSourcePlugin;

impl Plugin for VoxelStoreSourcePlugin {
	fn build(&self, app: &mut App) {
		let source = VoxelStoreSource::default();
		app.insert_resource(source.clone());
		app.register_voxel_source(source)
			.add_systems(PreUpdate, complete_voxel_store_acquisitions);
	}
}
