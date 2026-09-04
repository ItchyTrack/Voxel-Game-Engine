use std::{collections::HashMap, sync::Arc};

use bevy::prelude::*;
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_data::{grid::GridId, voxels::{VoxelTypeId, VoxelTypeInfo, Voxels}};
use voxel_mass::{MassError, MassProperties, SourceMassChange, SourceMassState, VoxelMassReaders, mass_properties_of_voxels};
use voxel_sources::{
	ChunkSource, RequestId, SourceCoverage, SourceHandle, SourceId,
	SourceResult, SourceResultData, edit::{GridEdit, GridGeneration},
};
use voxel_tasks::AsyncPriorityTaskPool;

use super::grid_store::{ChunkOwnership, GridStore, ServingRequest};

struct Acquisition {
	grid: GridId,
	chunk: IVec3,
	voxel_type: VoxelTypeInfo,
	generation: GridGeneration,
	baseline: Option<Voxels>,
	queued_edits: Vec<(GridGeneration, Arc<dyn GridEdit>, MassError)>,
}

#[derive(Default)]
pub struct VoxelStoreSource {
	handle: Option<SourceHandle>,
	grids: HashMap<GridId, GridStore>,
	acquisitions: HashMap<RequestId, Acquisition>,
	mass_state: SourceMassState,
	mass_readers: VoxelMassReaders,
}

impl VoxelStoreSource {
	pub(crate) fn new(mass_readers: VoxelMassReaders) -> Self {
		Self { mass_readers, ..Default::default() }
	}

	pub fn insert_chunk_data(&mut self, grid: GridId, chunk_data: HashMap<IVec3, Voxels>) {
		self.mass_state.initialize_grid_once(grid, MassProperties::ZERO, MassError::ZERO);
		let store = self.grids.entry(grid).or_default();
		for (chunk, voxels) in chunk_data {
			let previous = store.get_chunk(chunk).map(|version| {
				version.voxels()
					.as_ref()
					.and_then(|voxels| mass_properties_of_voxels(&self.mass_readers, voxels, chunk_origin(chunk)))
					.unwrap_or(MassProperties::ZERO)
			});
			let exact = mass_properties_of_voxels(&self.mass_readers, &voxels, chunk_origin(chunk))
				.unwrap_or(MassProperties::ZERO);
			match previous {
				Some(previous) => self.mass_state.apply_exact_edit(grid, previous, exact, MassError::ZERO),
				None => {
					self.mass_state.reconcile_generated_chunk(
						grid,
						chunk,
						MassProperties::ZERO,
						MassError::ZERO,
						exact,
					);
				}
			}
			store.save_chunk(chunk, GridGeneration::default(), &voxels);
		}
	}

	pub(crate) fn set_mass_readers(&mut self, mass_readers: VoxelMassReaders) {
		self.mass_readers = mass_readers;
	}

	pub(crate) fn source_id(&self) -> SourceId {
		self.handle.as_ref().expect("voxel store source was not initialized").id()
	}

	fn grid_available_area(&self, grid: GridId) -> Option<NonZeroChunkRegion> {
		self.grids.get(&grid)?.available_area()
	}

	pub(crate) fn chunk_ownership(&self, grid: GridId, chunk: IVec3) -> ChunkOwnership {
		self.grids.get(&grid).map_or(ChunkOwnership::Unowned, |store| store.ownership(chunk))
	}

	pub(crate) fn begin_acquisition(
		&mut self,
		grid: GridId,
		chunk: IVec3,
		request_id: RequestId,
		voxel_type: VoxelTypeInfo,
		baseline_generation: GridGeneration,
	) {
		self.grids.entry(grid).or_default().begin_acquiring(chunk, request_id);
		self.acquisitions.insert(request_id, Acquisition {
			grid,
			chunk,
			voxel_type,
			generation: baseline_generation,
			baseline: None,
			queued_edits: Vec::new(),
		});
	}

	pub(crate) fn apply_edit_or_queue(
		&mut self,
		grid: GridId,
		chunk: IVec3,
		voxel_type: VoxelTypeInfo,
		generation: GridGeneration,
		edit: Arc<dyn GridEdit>,
		reserved_error: MassError,
	) {
		self.mass_state.initialize_grid_once(grid, MassProperties::ZERO, MassError::ZERO);
		match self.chunk_ownership(grid, chunk) {
			ChunkOwnership::Owned => {
				assert_eq!(reserved_error, MassError::ZERO, "owned edits cannot carry a mass reservation");
				let (before, after) = self.grids.entry(grid).or_default().apply_edit(
					chunk,
					voxel_type,
					generation,
					edit.as_ref(),
					&self.mass_readers,
				);
				self.mass_state.apply_exact_edit(grid, before, after, MassError::ZERO);
			}
			ChunkOwnership::Acquiring(request_id) => {
				if let Some(acquisition) = self.acquisitions.get_mut(&request_id) {
					self.mass_state.reserve_edit_error(grid, reserved_error);
					acquisition.queued_edits.push((generation, edit, reserved_error));
				}
			}
			ChunkOwnership::Unowned => {}
		}
	}

	pub(crate) fn receive_acquisition_result(&mut self, result: &SourceResult) {
		let mut ready: Vec<(GridId, ServingRequest)> = Vec::new();
		match &result.data {
			SourceResultData::Voxels { region, lod, voxels, .. } => {
				let Some(acquisition) = self.acquisitions.get_mut(&result.request_id) else { return };
				if *lod != 0 || !region.contains(acquisition.chunk) { return; }
				let offset = (region.min() - acquisition.chunk) * CHUNK_SIZE as i32;
				acquisition.baseline.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info()))
					.merge_from(voxels, offset);
				return;
			}
			SourceResultData::VoxelsLoaded { .. } => {
				let Some(acquisition) = self.acquisitions.remove(&result.request_id) else { return };
				let mass_changes = self.grids.entry(acquisition.grid).or_default().complete_acquisition(
					acquisition.chunk,
					result.request_id,
					acquisition.generation,
					acquisition.baseline.as_ref(),
					acquisition.voxel_type,
					&acquisition.queued_edits,
					&self.mass_readers,
				);
				for (before, after, reserved_error) in mass_changes {
					self.mass_state.apply_exact_edit(acquisition.grid, before, after, reserved_error);
				}
				for (&grid, store) in self.grids.iter_mut() {
					store.retain_current_versions();
					for request in store.poll_waiting_requests() {
						ready.push((grid, request));
					}
				}
			}
			SourceResultData::Presence { .. } | SourceResultData::PresenceLoaded => return,
		}
		for (grid, request) in ready {
			self.dispatch(grid, request);
		}
	}

	fn dispatch(&self, grid: GridId, request: ServingRequest) {
		let handle = self.handle.as_ref().expect("voxel store source was not initialized").clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("VoxelStoreSource build").entered();
			for (chunk, version) in request.chunks {
				if request.cancellation.is_cancelled() { break; }
				let Some(voxels) = version.and_then(|version| version.voxels()) else { continue };
				handle.voxels(
					request.request_id,
					grid,
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
	fn init(&mut self, handle: SourceHandle) {
		self.mass_state.set_source_id(handle.id());
		self.handle = Some(handle);
	}

	fn request_voxels(
		&mut self,
		request_id: RequestId,
		cancellation: &voxel_tasks::CancellationToken,
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
			generation,
			chunks: Vec::new(),
		};
		let mut waits_for_acquisition = false;
		{
			let Some(store) = self.grids.get(&grid) else { return SourceCoverage::None };
			for chunk in chunks(region) {
				match store.ownership(chunk) {
					ChunkOwnership::Unowned => {}
					ChunkOwnership::Owned => request.chunks.push((chunk, store.get_chunk(chunk))),
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
			self.grids.entry(grid).or_default().push_waiting_request(request);
		} else {
			self.dispatch(grid, request);
		}
		coverage
	}

	fn request_presence(&mut self, request_id: RequestId, _cancellation: voxel_tasks::CancellationToken, grid: GridId) {
		let handle = self.handle.as_ref().expect("voxel store source was not initialized");
		if let Some(region) = self.grid_available_area(grid) {
			handle.presence(request_id, grid, region);
		}
		handle.presence_loaded(request_id);
	}

	fn acquire_ownership(&mut self, grid: GridId, region: NonZeroChunkRegion) {
		for chunk in chunks(region) {
			let pending = self.acquisitions.iter()
				.filter_map(|(request_id, acquisition)| {
					(acquisition.grid == grid && acquisition.chunk == chunk).then_some(*request_id)
				})
				.max();
			let store = self.grids.entry(grid).or_default();
			if let Some(request_id) = pending {
				store.begin_acquiring(chunk, request_id);
			} else {
				store.acquire(chunk);
			}
		}
	}

	fn relinquish_ownership(&mut self, grid: GridId, region: NonZeroChunkRegion) {
		let Some(store) = self.grids.get_mut(&grid) else { return };
		for chunk in chunks(region) {
			store.relinquish(chunk);
		}
	}
}

pub(crate) fn chunks(region: NonZeroChunkRegion) -> impl Iterator<Item = IVec3> {
	(region.min().z..region.end().z).flat_map(move |z| {
		(region.min().y..region.end().y).flat_map(move |y| {
			(region.min().x..region.end().x).map(move |x| IVec3::new(x, y, z))
		})
	})
}

pub fn complete_voxel_store_acquisitions(
	mut sources: ResMut<voxel_sources::SourceManager>,
	mut results: MessageReader<SourceResult>,
) {
	for result in results.read() {
		if let Some(store) = sources.get_source_mut::<VoxelStoreSource>() {
			store.receive_acquisition_result(result);
		}
	}
}

pub fn drain_voxel_store_mass_changes(
	mut sources: ResMut<voxel_sources::SourceManager>,
	mass_readers: Res<VoxelMassReaders>,
	mut changes: MessageWriter<SourceMassChange>,
) {
	let Some(store) = sources.get_source_mut::<VoxelStoreSource>() else { return };
	store.mass_readers = mass_readers.clone();
	changes.write_batch(store.mass_state.drain_changes());
}
