use bevy::{ecs::{component::Component, message::Message}, math::IVec3};
use serde::{Deserialize, Serialize};
use voxel_data::{
	grid::GridId, region::NonZeroVoxelRegion, voxels::{Voxel, Voxels},
};

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

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridEditId(u64);

impl GridEditId {
	pub const fn get_next(self) -> GridEditId { GridEditId(self.0 + 1) }

	pub const fn is_next(self, maybe_next_id: GridEditId) -> bool { self.get_next().0 == maybe_next_id.0 }
}

#[derive(Default, Debug, Component)]
pub struct GridEditIdManager {
	current: GridEditId,
}

impl GridEditIdManager {
	pub fn current_id(&self) -> GridEditId {
		self.current
	}

	pub fn bump_id(&mut self) -> GridEditId {
		self.current = self.current.get_next();
		self.current
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
