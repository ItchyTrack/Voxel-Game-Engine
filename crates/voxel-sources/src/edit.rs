use bevy::{ecs::{component::Component, message::Message}, math::IVec3};
use serde::{Deserialize, Serialize};
use voxel_data::{grid::GridId, region::NonZeroVoxelRegion, voxels::{Voxel, Voxels}};

#[typetag::serde(tag = "type")]
pub trait GridEdit: std::fmt::Debug + Send + Sync + 'static {
	fn affected_region(&self) -> NonZeroVoxelRegion;
	fn apply_to_voxels(&self, voxels_position: IVec3, voxels: &mut Voxels);
	fn apply_to_tracking(&self) {} // todo once voxel tracking is added
}

#[derive(Debug, Serialize, Deserialize)]
pub struct AddArea {
	region: NonZeroVoxelRegion,
	voxel: Voxel,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RemoveArea {
	region: NonZeroVoxelRegion,
}

#[typetag::serde]
impl GridEdit for AddArea {
	fn affected_region(&self) -> NonZeroVoxelRegion { self.region }

	fn apply_to_voxels(&self, voxels_position: IVec3, voxels: &mut Voxels) {
		let Some(region) =
			NonZeroVoxelRegion::from_min_end((self.region.min() + voxels_position).max(IVec3::ZERO), self.region.end() + voxels_position)
		else {
			return;
		};
		voxels.add_area(region.min().as_uvec3(), region.size(), self.voxel.get_ref());
	}
}

#[typetag::serde]
impl GridEdit for RemoveArea {
	fn affected_region(&self) -> NonZeroVoxelRegion { self.region }

	fn apply_to_voxels(&self, voxels_position: IVec3, voxels: &mut Voxels) {
		let Some(region) =
			NonZeroVoxelRegion::from_min_end((self.region.min() + voxels_position).max(IVec3::ZERO), self.region.end() + voxels_position)
		else {
			return;
		};
		voxels.remove_area(region.min().as_uvec3(), region.size());
	}
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GridEditId(u64);

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct GridChunkGeneration(u64);

impl GridEditId {
	pub const fn get_next(self) -> GridEditId { GridEditId(self.0 + 1) }

	pub const fn is_next(self, maybe_next_id: GridEditId) -> bool { self.get_next().0 == maybe_next_id.0 }
}

#[derive(Default, Debug, Component)]
pub struct GridEditIdManager {
	current_edit_id: GridEditId,
	current_chunk_generation: GridChunkGeneration,
	// chunk_generations: SignedGridTree<U64Cell> // 95% sure I dont need this as generation invaidation
												  // should happen when the generation of a region changes
												  // due to a edit. Not by querying the region.
}

impl GridEditIdManager {
	pub fn current_id(&self) -> GridEditId {
		self.current_edit_id
	}

	pub fn latest_generation(&self) -> GridChunkGeneration {
		self.current_chunk_generation
	}

	// pub fn latest_region_generation(&self, region: NonZeroChunkRegion) -> GridChunkGeneration {
	// 	let mut max_gen: u64 = 0;
	// 	self.chunk_generations.for_each_in_region(
	// 		NonZeroVoxelRegion::from_min_size(region.min(), region.size()).unwrap(),
	// 		|_, _, chunk_gen| max_gen = max_gen.max(chunk_gen)
	// 	);
	// 	GridChunkGeneration(max_gen)
	// }

	// this function is called to update the generation of the region and to get the new GridEditId and ChunkGeneration
	pub fn apply_edit(&mut self/*, grid_edit: &(impl GridEdit + ?Sized)*/) -> (GridEditId, GridChunkGeneration) {
		// let region = chunks_covering_nonzero_voxel_region(grid_edit.affected_region());
		self.current_chunk_generation.0 += 1;
		// self.chunk_generations.add_area(NonZeroVoxelRegion::from_min_size(region.min(), region.size()).unwrap(), self.current_chunk_generation.0);
		self.current_edit_id = self.current_edit_id.get_next();
		(self.current_edit_id, self.current_chunk_generation)
	}
}

#[derive(Debug, Message)]
pub struct GridEditMessage {
	grid_id: GridId,
	edit_id: GridEditId,
	edit: Box<dyn GridEdit>,
}

impl GridEditMessage {
	pub fn new<T: GridEdit>(grid_id: GridId, grid_edit_id: GridEditId, grid_edit: T) -> Self {
		Self {
			grid_id,
			edit_id: grid_edit_id,
			edit: Box::new(grid_edit),
		}
	}

	pub fn grid_id(&self) -> GridId {
		self.grid_id
	}

	pub fn edit_id(&self) -> GridEditId {
		self.edit_id
	}

	pub fn edit(&self) -> &dyn GridEdit {
		self.edit.as_ref()
	}
}
