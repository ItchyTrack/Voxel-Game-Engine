use std::{any::Any, sync::Arc};

use bevy::{ecs::{component::Component, message::Message}, math::IVec3};
use serde::{Deserialize, Serialize};
use bile_math::NonZeroRegion;
use voxel_data::{grid::GridId, voxels::{Voxel, Voxels}};
use voxel_trees::grid_tree::{GridTree64, U16Cell};

#[typetag::serde(tag = "type")]
pub trait GridEdit: std::fmt::Debug + Send + Sync + GridEditToAny {
	fn affected_region(&self) -> NonZeroRegion;
	fn apply_to_voxels(&self, voxels_position: IVec3, voxels: &mut Voxels);
	fn apply_to_occupancy(&self, tile: NonZeroRegion, occupancy: &mut GridTree64<U16Cell>);
	fn apply_to_tracking(&self) {} // todo once voxel tracking is added
}

pub trait GridEditToAny: 'static {
	fn as_any(&self) -> &dyn Any;
}

impl<T: 'static> GridEditToAny for T {
	fn as_any(&self) -> &dyn Any { self }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct AddArea {
	region: NonZeroRegion,
	voxel: Voxel,
}

impl AddArea {
	pub fn new(region: NonZeroRegion, voxel: Voxel) -> Self { Self { region, voxel } }

	pub fn voxel(&self) -> &Voxel { &self.voxel }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RemoveArea {
	region: NonZeroRegion,
}

impl RemoveArea {
	pub fn new(region: NonZeroRegion) -> Self { Self { region } }
}

#[typetag::serde]
impl GridEdit for AddArea {
	fn affected_region(&self) -> NonZeroRegion { self.region }

	fn apply_to_voxels(&self, voxels_position: IVec3, voxels: &mut Voxels) {
		let Some(region) =
			NonZeroRegion::from_min_end((self.region.min() + voxels_position).max(IVec3::ZERO), self.region.end() + voxels_position)
		else {
			return;
		};
		voxels.add_area(region.min().as_uvec3(), region.size(), self.voxel.get_ref());
	}

	fn apply_to_occupancy(&self, tile: NonZeroRegion, occupancy: &mut GridTree64<U16Cell>) {
		let Some(region) = self.region.intersection(tile) else { return };
		occupancy.add_area(&(region.min() - tile.min()).as_uvec3(), region.size(), 0u16);
	}
}

#[typetag::serde]
impl GridEdit for RemoveArea {
	fn affected_region(&self) -> NonZeroRegion { self.region }

	fn apply_to_voxels(&self, voxels_position: IVec3, voxels: &mut Voxels) {
		let Some(region) =
			NonZeroRegion::from_min_end((self.region.min() + voxels_position).max(IVec3::ZERO), self.region.end() + voxels_position)
		else {
			return;
		};
		voxels.remove_area(region.min().as_uvec3(), region.size());
	}

	fn apply_to_occupancy(&self, tile: NonZeroRegion, occupancy: &mut GridTree64<U16Cell>) {
		let Some(region) = self.region.intersection(tile) else { return };
		occupancy.remove_area(&(region.min() - tile.min()).as_uvec3(), region.size());
	}
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GridEditId(u64);

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct GridGeneration(u64);

impl GridEditId {
	pub const fn get_next(self) -> GridEditId { GridEditId(self.0 + 1) }

	pub const fn is_next(self, maybe_next_id: GridEditId) -> bool { self.get_next().0 == maybe_next_id.0 }
}

#[derive(Default, Debug, Component)]
pub struct GridEditIdManager {
	current_edit_id: GridEditId,
	current_grid_generation: GridGeneration,
}

impl GridEditIdManager {
	pub fn current_id(&self) -> GridEditId { self.current_edit_id }

	pub fn latest_generation(&self) -> GridGeneration { self.current_grid_generation }

	pub fn apply_edit(&mut self) -> (GridEditId, GridGeneration) {
		self.current_grid_generation.0 += 1;
		self.current_edit_id = self.current_edit_id.get_next();
		(self.current_edit_id, self.current_grid_generation)
	}
}

#[derive(Debug, Message)]
pub struct GridEditMessage {
	grid_id: GridId,
	grid_generation: GridGeneration,
	edit_id: GridEditId,
	edit: Arc<dyn GridEdit>,
}

impl GridEditMessage {
	pub fn new<T: GridEdit>(grid_id: GridId, grid_generation: GridGeneration, grid_edit_id: GridEditId, grid_edit: T) -> Self {
		Self::from_shared(grid_id, grid_generation, grid_edit_id, Arc::new(grid_edit))
	}

	pub fn from_shared(
		grid_id: GridId,
		grid_generation: GridGeneration,
		grid_edit_id: GridEditId,
		grid_edit: Arc<dyn GridEdit>,
	) -> Self {
		Self { grid_id, grid_generation, edit_id: grid_edit_id, edit: grid_edit }
	}

	pub fn grid_id(&self) -> GridId { self.grid_id }

	pub fn grid_generation(&self) -> GridGeneration { self.grid_generation }

	pub fn edit_id(&self) -> GridEditId { self.edit_id }

	pub fn edit(&self) -> &dyn GridEdit { self.edit.as_ref() }
}
