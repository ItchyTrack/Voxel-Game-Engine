use std::{collections::HashMap, sync::Arc};

use bevy::math::{IVec3, UVec3};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_trees::region::NonZeroVoxelRegion;
use voxel_data::{
	compressed_voxels::CompressedVoxels,
	voxels::{VoxelTypeInfo, Voxels},
};
use voxel_mass::{MassError, MassProperties, VoxelMassReaders, mass_properties_of_voxels};
use voxel_sources::{RequestId, edit::{GridEdit, GridGeneration}};
use voxel_tasks::CancellationToken;

#[derive(Debug)]
pub(crate) struct ChunkVersion {
	pub(crate) generation: GridGeneration,
	voxels: Option<CompressedVoxels>,
}

impl ChunkVersion {
	pub(crate) fn voxels(&self) -> Option<Voxels> {
		self.voxels.as_ref()?.decompress().ok()
	}

	fn empty(generation: GridGeneration) -> Self {
		Self { generation, voxels: None }
	}

	fn from_voxels(generation: GridGeneration, voxels: &Voxels) -> Self {
		let voxels = (!voxels.is_empty()).then(|| CompressedVoxels::new(voxels, 0).expect("compressing voxels into memory cannot fail"));
		Self { generation, voxels }
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ChunkOwnership {
	Owned,
	Acquiring(RequestId),
	Unowned,
}

struct StoredChunk {
	ownership: ChunkOwnership,
	versions: Vec<Arc<ChunkVersion>>,
}

impl StoredChunk {
	fn new(ownership: ChunkOwnership) -> Self {
		Self { ownership, versions: Vec::new() }
	}

	fn push_version(&mut self, version: ChunkVersion) -> Arc<ChunkVersion> {
		self.versions.retain(|existing| existing.generation != version.generation);
		let version = Arc::new(version);
		self.versions.push(version.clone());
		self.versions.sort_by_key(|version| version.generation);
		version
	}

	fn get(&self) -> Option<Arc<ChunkVersion>> {
		self.versions.last().cloned()
	}

	fn retain_current_version(&mut self) {
		if let Some(current) = self.versions.pop() {
			self.versions.clear();
			self.versions.push(current);
		}
	}
}

pub(crate) struct ServingRequest {
	pub(crate) request_id: RequestId,
	pub(crate) cancellation: CancellationToken,
	pub(crate) generation: GridGeneration,
	pub(crate) chunks: Vec<(IVec3, Option<Arc<ChunkVersion>>)>,
}

#[derive(Default)]
pub struct GridStore {
	chunks: HashMap<IVec3, StoredChunk>,
	waiting_requests: Vec<ServingRequest>,
}

impl GridStore {
	pub fn available_area(&self) -> Option<NonZeroChunkRegion> {
		let mut iter = self.chunks.keys().copied();
		let first = iter.next()?;
		let (mut min, mut max) = (first, first);
		for chunk in iter {
			min = min.min(chunk);
			max = max.max(chunk);
		}
		NonZeroChunkRegion::from_min_max(min, max)
	}

	pub fn save_chunk(&mut self, chunk: IVec3, generation: GridGeneration, voxels: &Voxels) {
		let stored = self.chunks.entry(chunk).or_insert_with(|| StoredChunk::new(ChunkOwnership::Owned));
		stored.ownership = ChunkOwnership::Owned;
		stored.push_version(ChunkVersion::from_voxels(generation, voxels));
	}

	pub(crate) fn ownership(&self, chunk: IVec3) -> ChunkOwnership {
		self.chunks.get(&chunk).map_or(ChunkOwnership::Unowned, |chunk| chunk.ownership)
	}

	pub(crate) fn acquire(&mut self, chunk: IVec3) {
		let stored = self.chunks.entry(chunk).or_insert_with(|| StoredChunk::new(ChunkOwnership::Owned));
		stored.ownership = ChunkOwnership::Owned;
		if stored.versions.is_empty() {
			stored.push_version(ChunkVersion::empty(GridGeneration::default()));
		}
	}

	pub(crate) fn begin_acquiring(&mut self, chunk: IVec3, request_id: RequestId) {
		let stored = self.chunks.entry(chunk).or_insert_with(|| StoredChunk::new(ChunkOwnership::Acquiring(request_id)));
		stored.ownership = ChunkOwnership::Acquiring(request_id);
	}

	pub(crate) fn relinquish(&mut self, chunk: IVec3) {
		if let Some(stored) = self.chunks.get_mut(&chunk) {
			stored.ownership = ChunkOwnership::Unowned;
		}
	}

	pub(crate) fn get_chunk(&self, chunk: IVec3) -> Option<Arc<ChunkVersion>> {
		self.chunks.get(&chunk)?.get()
	}

	pub(crate) fn complete_acquisition(
		&mut self,
		chunk: IVec3,
		request_id: RequestId,
		baseline_generation: GridGeneration,
		baseline: Option<&Voxels>,
		voxel_type: VoxelTypeInfo,
		edits: &[(GridGeneration, Arc<dyn GridEdit>, MassError)],
		mass_readers: &VoxelMassReaders,
	) -> Vec<(MassProperties, MassProperties, MassError)> {
		let stored = self.chunks.entry(chunk).or_insert_with(|| StoredChunk::new(ChunkOwnership::Unowned));
		let baseline = match baseline {
			Some(voxels) => ChunkVersion::from_voxels(baseline_generation, voxels),
			None => ChunkVersion::empty(baseline_generation),
		};
		let mut current = stored.push_version(baseline);
		let mut mass_changes = Vec::with_capacity(edits.len());
		for (generation, edit, reserved_error) in edits {
			let before = current.voxels().unwrap_or_else(|| Voxels::new_with_type(voxel_type));
			let before_mass = chunk_mass(mass_readers, chunk, &before);
			let after = edited_chunk_voxels(before, chunk, voxel_type, edit.as_ref());
			let after_mass = chunk_mass(mass_readers, chunk, &after);
			current = stored.push_version(ChunkVersion::from_voxels(*generation, &after));
			mass_changes.push((before_mass, after_mass, *reserved_error));
		}
		if stored.ownership == ChunkOwnership::Acquiring(request_id) {
			stored.ownership = ChunkOwnership::Owned;
		}
		mass_changes
	}

	pub(crate) fn apply_edit(
		&mut self,
		chunk: IVec3,
		voxel_type: VoxelTypeInfo,
		generation: GridGeneration,
		edit: &dyn GridEdit,
		mass_readers: &VoxelMassReaders,
	) -> (MassProperties, MassProperties) {
		let stored = self.chunks.entry(chunk).or_insert_with(|| StoredChunk::new(ChunkOwnership::Owned));
		let before = stored.get()
			.and_then(|version| version.voxels())
			.unwrap_or_else(|| Voxels::new_with_type(voxel_type));
		let before_mass = chunk_mass(mass_readers, chunk, &before);
		let after = edited_chunk_voxels(before, chunk, voxel_type, edit);
		let after_mass = chunk_mass(mass_readers, chunk, &after);
		stored.push_version(ChunkVersion::from_voxels(generation, &after));
		stored.retain_current_version();
		(before_mass, after_mass)
	}

	pub(crate) fn retain_current_versions(&mut self) {
		for chunk in self.chunks.values_mut() {
			chunk.retain_current_version();
		}
	}

	pub(crate) fn push_waiting_request(&mut self, request: ServingRequest) {
		self.waiting_requests.push(request);
	}

	pub(crate) fn poll_waiting_requests(&mut self) -> Vec<ServingRequest> {
		let mut ready = Vec::new();
		let mut still_waiting = Vec::new();
		for mut request in std::mem::take(&mut self.waiting_requests) {
			let mut waiting = false;
			for (chunk, version) in &mut request.chunks {
				if version.is_some() { continue; }
				if matches!(self.ownership(*chunk), ChunkOwnership::Acquiring(_)) {
					waiting = true;
				} else {
					*version = self.get_chunk(*chunk);
				}
			}
			if waiting { still_waiting.push(request) } else { ready.push(request) }
		}
		self.waiting_requests = still_waiting;
		ready
	}
}

fn edited_chunk_voxels(
	mut voxels: Voxels,
	chunk: IVec3,
	voxel_type: VoxelTypeInfo,
	edit: &dyn GridEdit,
) -> Voxels {
	edit.apply_to_voxels(-chunk_origin(chunk), &mut voxels);

	let mut chunk_voxels = Voxels::new_with_type(voxel_type);
	let chunk_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::splat(CHUNK_SIZE)).unwrap();
	chunk_voxels.merge_region_from(&voxels, Some(chunk_region), IVec3::ZERO);
	chunk_voxels
}

fn chunk_mass(readers: &VoxelMassReaders, chunk: IVec3, voxels: &Voxels) -> MassProperties {
	mass_properties_of_voxels(readers, voxels, chunk_origin(chunk)).unwrap_or(MassProperties::ZERO)
}
