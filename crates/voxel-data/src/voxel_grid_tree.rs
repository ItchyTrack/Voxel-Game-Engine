use crate::grid_tree::{CellKind, GridCell, GridTree, GridTreeNode, I16Coord};
use serde::{Deserialize, Serialize};

// If value == 1<<16-1: EMPTY, else if bit 15 is 0: DATA, else: NODE.
// A NODE offset can't be 0x7FFF because that aliases EMPTY.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct PackedCell { value: u16 }

impl GridCell for PackedCell {
	type Data = u16;
	const EMPTY: Self = PackedCell { value: u16::MAX };
	const MAX_DATA: u16 = (1 << 15) - 1;
	const MAX_NODE_OFFSET: u32 = (1 << 15) - 2;
	fn data(value: u16) -> Self { PackedCell { value } }
	fn node(offset: u32) -> Self { PackedCell { value: offset as u16 | (1 << 15) } }
	fn kind(self) -> CellKind {
		if self.value == u16::MAX { CellKind::Empty }
		else if self.value & (1 << 15) == 0 { CellKind::Data }
		else { CellKind::Node }
	}
	fn data_value(self) -> u16 { self.value & ((1 << 15) - 1) }
	fn node_offset(self) -> u32 { (self.value & ((1 << 15) - 1)) as u32 }
}

pub type VoxelGridTree = GridTree<PackedCell, I16Coord>;
pub type PackedNode = GridTreeNode<PackedCell>;
