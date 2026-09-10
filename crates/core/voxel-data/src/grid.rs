use bevy::prelude::*;

use crate::voxels::{VoxelType, VoxelTypeInfo};

/// A [`Grid`] entity.
pub type GridId = Entity;

/// An integer offset in the parent grid's voxel coordinates.
#[derive(Component, serde::Serialize, serde::Deserialize, Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridLocalTransform(pub IVec3);

#[derive(Debug, Component)]
pub struct Grid {
	voxel_type_info: VoxelTypeInfo,
}

impl Grid {
	pub fn new<T: VoxelType>() -> Self {
		Self::new_with_type(T::TYPE_INFO)
	}

	pub fn new_with_type(voxel_type_info: VoxelTypeInfo) -> Self {
		Self { voxel_type_info }
	}

	pub fn voxel_type_info(&self) -> VoxelTypeInfo { self.voxel_type_info }
}
