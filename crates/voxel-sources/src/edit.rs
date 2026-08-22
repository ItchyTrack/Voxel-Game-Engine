use bevy::math::IVec3;
use serde::{Deserialize, Serialize};
use voxel_data::{
	region::NonZeroVoxelRegion,
	voxels::{Voxel, Voxels},
};

#[typetag::serde(tag = "type")]
pub trait GirdEdit {
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
impl GirdEdit for AddArea {
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
impl GirdEdit for RemoveArea {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridEditId(u64);

impl GridEditId {
	pub const fn get_next(self) -> GridEditId { GridEditId(self.0 + 1) }

	pub const fn is_next(self, maybe_next_id: GridEditId) -> bool { self.get_next().0 == maybe_next_id.0 }
}

pub struct GirdEditMessage {
	edit: Box<dyn GirdEdit>,
	grid_edit_id: GridEditId,
}
