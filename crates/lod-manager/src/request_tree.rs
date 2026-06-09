use bevy::prelude::*;
use voxel_data::grid_tree::{CellKind, GridCell, GridTree, I32Coord};

use crate::{LodDestination, LodKey};

/// Packed `GridTree` cell whose data value is a requested LOD level.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct LodRequestCell {
	value: u16,
}

impl GridCell for LodRequestCell {
	type Data = u16;
	const EMPTY: Self = Self { value: u16::MAX };
	const MAX_DATA: u16 = (1 << 15) - 1;
	const MAX_NODE_OFFSET: u32 = (1 << 15) - 2;

	fn data(value: u16) -> Self {
		Self { value }
	}

	fn node(offset: u32) -> Self {
		Self { value: offset as u16 | (1 << 15) }
	}

	fn kind(self) -> CellKind {
		if self.value == u16::MAX {
			CellKind::Empty
		} else if self.value & (1 << 15) == 0 {
			CellKind::Data
		} else {
			CellKind::Node
		}
	}

	fn data_value(self) -> u16 {
		self.value & ((1 << 15) - 1)
	}

	fn node_offset(self) -> u32 {
		(self.value & ((1 << 15) - 1)) as u32
	}
}

/// Sparse chunk-space tree storing requested LOD level per chunk.
///
/// Unrequested chunks are absent from the tree.
pub type LodRequestGridTree = GridTree<LodRequestCell, I32Coord>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LodRequest {
	pub key: LodKey,
	pub priority: f32,
	pub destination: LodDestination,
}

impl LodRequest {
	pub fn gpu(key: LodKey, priority: f32) -> Self {
		Self { key, priority, destination: LodDestination::Gpu }
	}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoadedLodEvent {
	pub key: LodKey,
	pub entity: Entity,
}

pub(crate) fn add_key_to_tree(tree: &mut LodRequestGridTree, key: LodKey) {
	tree.add_area(&key.min, key.size, key.level.min(LodRequestCell::MAX_DATA as u32) as u16);
}

pub(crate) fn remove_key_from_tree(tree: &mut LodRequestGridTree, key: LodKey) {
	tree.remove_area(&key.min, key.size);
}
