use bevy::math::IVec3;

use super::{CellKind, GridType, SIZE_USIZE_CUBED};

pub const NODE_HEADER_SIZE: usize = 32;
pub const NODE_DATA_MASK_OFFSET: usize = 0;
pub const NODE_NODE_MASK_OFFSET: usize = 8;
pub const NODE_PARENT_OFFSET_OFFSET: usize = 16;
pub const NODE_USED_CELL_COUNT_OFFSET: usize = 20;

const CHILD_INDEX_BYTES: usize = std::mem::size_of::<u32>();

#[derive(Clone, Debug)]
pub struct RawGridTree {
	buffer: Vec<u8>,
	cell_stride: usize,
	root_pos: IVec3,
	root_depth: u8,
	item_count: u64,
	dead_nodes: usize,
	free_nodes: Vec<u32>,
}

impl RawGridTree {
	pub fn new(data_size_bytes: usize) -> Self {
		let cell_stride = data_size_bytes.max(CHILD_INDEX_BYTES);
		let mut out = Self {
			buffer: Vec::new(),
			cell_stride,
			root_pos: IVec3::ZERO,
			root_depth: 0,
			item_count: 0,
			dead_nodes: 0,
			free_nodes: Vec::new(),
		};
		out.push_zeroed_node();
		out
	}

	pub fn cell_stride(&self) -> usize { self.cell_stride }
	pub fn node_stride(&self) -> usize { NODE_HEADER_SIZE + SIZE_USIZE_CUBED * self.cell_stride }
	pub fn node_count(&self) -> usize { self.buffer.len() / self.node_stride() }
	pub fn root_pos(&self) -> IVec3 { self.root_pos }
	pub fn root_depth(&self) -> u8 { self.root_depth }
	pub fn item_count(&self) -> u64 { self.item_count }
	pub fn is_empty(&self) -> bool { self.item_count == 0 }
	pub fn bytes(&self) -> &[u8] { &self.buffer }
	pub(crate) fn free_node_count(&self) -> usize { self.free_nodes.len() }

	pub(crate) fn set_root(&mut self, pos: IVec3, depth: u8) {
		self.root_pos = pos;
		self.root_depth = depth;
	}
	pub(crate) fn set_item_count(&mut self, value: u64) { self.item_count = value; }
	pub(crate) fn add_item_count(&mut self, value: u64) { self.item_count += value; }
	pub(crate) fn sub_item_count(&mut self, value: u64) { self.item_count -= value; }

	#[inline]
	pub fn node_base(&self, node_index: u32) -> usize { node_index as usize * self.node_stride() }

	#[inline]
	pub fn cell_base(&self, node_index: u32, cell_index: u8) -> usize {
		self.node_base(node_index) + NODE_HEADER_SIZE + cell_index as usize * self.cell_stride
	}

	pub fn cell_bytes(&self, node_index: u32, cell_index: u8) -> &[u8] {
		let start = self.cell_base(node_index, cell_index);
		&self.buffer[start..start + self.cell_stride]
	}

	pub fn cell_bytes_mut(&mut self, node_index: u32, cell_index: u8) -> &mut [u8] {
		let start = self.cell_base(node_index, cell_index);
		&mut self.buffer[start..start + self.cell_stride]
	}

	pub(crate) fn reset_empty_root(&mut self, root_pos: IVec3, root_depth: u8) {
		self.clear_all_nodes();
		self.item_count = 0;
		self.root_pos = root_pos;
		self.root_depth = root_depth;
		self.push_zeroed_node();
	}

	pub(crate) fn clear_all_nodes(&mut self) {
		self.buffer.clear();
		self.dead_nodes = 0;
		self.free_nodes.clear();
	}

	pub(crate) fn push_empty_node(&mut self, parent_offset: u32) -> u32 {
		let index = self.push_zeroed_node();
		self.set_parent_offset(index, parent_offset);
		index
	}

	pub(crate) fn insert_empty_node(&mut self, index: u32, parent_offset: u32) {
		let stride = self.node_stride();
		let start = self.node_base(index);
		let old_len = self.buffer.len();
		self.buffer.resize(old_len + stride, 0);
		self.buffer.copy_within(start..old_len, start + stride);
		self.buffer[start..start + stride].fill(0);
		self.set_parent_offset(index, parent_offset);
	}

	pub(crate) fn push_zeroed_node(&mut self) -> u32 {
		let index = self.node_count() as u32;
		self.buffer.resize(self.buffer.len() + self.node_stride(), 0);
		index
	}

	pub(crate) fn remove_node(&mut self, node_index: u32) {
		if self.used_cell_count(node_index) == 255 { return; }
		self.set_used_cell_count(node_index, 255);
		self.dead_nodes += 1;
		self.free_nodes.push(node_index);
	}

	pub(crate) fn has_free_node_after_parent(&self, parent_node_index: u32, max_offset: u32) -> bool {
		self.free_nodes.iter().any(|&idx| idx > parent_node_index && idx - parent_node_index <= max_offset)
	}

	pub(crate) fn free_node_after_parent(&mut self, parent_node_index: u32, max_offset: u32) -> Option<u32> {
		let free_slot = self.free_nodes.iter().position(|&idx| idx > parent_node_index && idx - parent_node_index <= max_offset)?;
		let node_index = self.free_nodes.swap_remove(free_slot);
		self.dead_nodes -= 1;
		self.clear_node(node_index);
		Some(node_index)
	}

	pub(crate) fn clear_node(&mut self, node_index: u32) {
		let start = self.node_base(node_index);
		let end = start + self.node_stride();
		self.buffer[start..end].fill(0);
	}

	pub(crate) fn parent_offset(&self, node_index: u32) -> u32 {
		self.read_u32_at(self.node_base(node_index) + NODE_PARENT_OFFSET_OFFSET)
	}
	pub(crate) fn set_parent_offset(&mut self, node_index: u32, value: u32) {
		let offset = self.node_base(node_index) + NODE_PARENT_OFFSET_OFFSET;
		self.write_u32_at(offset, value);
	}
	pub(crate) fn used_cell_count(&self, node_index: u32) -> u8 {
		self.read_u8_at(self.node_base(node_index) + NODE_USED_CELL_COUNT_OFFSET)
	}
	pub(crate) fn set_used_cell_count(&mut self, node_index: u32, value: u8) {
		let offset = self.node_base(node_index) + NODE_USED_CELL_COUNT_OFFSET;
		self.write_u8_at(offset, value);
	}
	pub(crate) fn inc_used_cell_count(&mut self, node_index: u32) {
		let next = self.used_cell_count(node_index) + 1;
		assert!(next <= 64);
		self.set_used_cell_count(node_index, next);
	}
	pub(crate) fn dec_used_cell_count(&mut self, node_index: u32) {
		self.set_used_cell_count(node_index, self.used_cell_count(node_index) - 1);
	}

	pub(crate) fn data_mask(&self, node_index: u32) -> u64 {
		self.read_u64_at(self.node_base(node_index) + NODE_DATA_MASK_OFFSET)
	}
	pub(crate) fn node_mask(&self, node_index: u32) -> u64 {
		self.read_u64_at(self.node_base(node_index) + NODE_NODE_MASK_OFFSET)
	}
	fn set_data_mask(&mut self, node_index: u32, mask: u64) {
		let offset = self.node_base(node_index) + NODE_DATA_MASK_OFFSET;
		self.write_u64_at(offset, mask);
	}
	fn set_node_mask(&mut self, node_index: u32, mask: u64) {
		let offset = self.node_base(node_index) + NODE_NODE_MASK_OFFSET;
		self.write_u64_at(offset, mask);
	}

	pub(crate) fn cell_kind(&self, node_index: u32, cell_index: u8) -> CellKind {
		let bit = 1u64 << cell_index;
		match (self.data_mask(node_index) & bit != 0, self.node_mask(node_index) & bit != 0) {
			(false, false) => CellKind::Empty,
			(true, false) => CellKind::Data,
			(false, true) => CellKind::Node,
			(true, true) => unreachable!("cell is marked as both data and node"),
		}
	}

	pub(crate) fn child_index(&self, node_index: u32, cell_index: u8) -> u32 {
		u32::from_le_bytes(self.cell_bytes(node_index, cell_index)[..CHILD_INDEX_BYTES].try_into().expect("child index bytes"))
	}

	pub(crate) fn set_empty(&mut self, node_index: u32, cell_index: u8) {
		let bit = 1u64 << cell_index;
		self.set_data_mask(node_index, self.data_mask(node_index) & !bit);
		self.set_node_mask(node_index, self.node_mask(node_index) & !bit);
		self.cell_bytes_mut(node_index, cell_index).fill(0);
	}

	pub(crate) fn set_child_index(&mut self, node_index: u32, cell_index: u8, child_index: u32) {
		let bit = 1u64 << cell_index;
		self.set_data_mask(node_index, self.data_mask(node_index) & !bit);
		self.set_node_mask(node_index, self.node_mask(node_index) | bit);
		let bytes = self.cell_bytes_mut(node_index, cell_index);
		bytes.fill(0);
		bytes[..CHILD_INDEX_BYTES].copy_from_slice(&child_index.to_le_bytes());
	}

	pub(crate) fn set_data<G: GridType>(&mut self, grid_type: &G, node_index: u32, cell_index: u8, data: G::Data<'_>) {
		let bit = 1u64 << cell_index;
		self.set_data_mask(node_index, self.data_mask(node_index) | bit);
		self.set_node_mask(node_index, self.node_mask(node_index) & !bit);
		let bytes = self.cell_bytes_mut(node_index, cell_index);
		bytes.fill(0);
		grid_type.write_data(data, bytes);
	}

	pub(crate) fn copy_data_cell(&mut self, src_node: u32, src_cell: u8, dst_node: u32, dst_cell: u8) {
		let src_start = self.cell_base(src_node, src_cell);
		let dst_start = self.cell_base(dst_node, dst_cell);
		let stride = self.cell_stride;
		self.buffer.copy_within(src_start..src_start + stride, dst_start);
		let bit = 1u64 << dst_cell;
		self.set_data_mask(dst_node, self.data_mask(dst_node) | bit);
		self.set_node_mask(dst_node, self.node_mask(dst_node) & !bit);
	}

	pub(crate) fn copy_cell_to_all_children(&mut self, src_node: u32, src_cell: u8, dst_node: u32) {
		let src_start = self.cell_base(src_node, src_cell);
		let stride = self.cell_stride;
		for child in 0..SIZE_USIZE_CUBED {
			let dst_start = self.cell_base(dst_node, child as u8);
			self.buffer.copy_within(src_start..src_start + stride, dst_start);
		}
		self.set_data_mask(dst_node, u64::MAX);
		self.set_node_mask(dst_node, 0);
		self.set_used_cell_count(dst_node, super::SIZE_CUBED);
	}

	pub(crate) fn fill_node_with_data<G: GridType>(&mut self, grid_type: &G, node_index: u32, data: G::Data<'_>) {
		self.set_data_mask(node_index, u64::MAX);
		self.set_node_mask(node_index, 0);
		self.set_used_cell_count(node_index, super::SIZE_CUBED);
		for child in 0..super::SIZE_CUBED {
			let bytes = self.cell_bytes_mut(node_index, child);
			bytes.fill(0);
			grid_type.write_data(data, bytes);
		}
	}

	pub(crate) fn increment_child_indices_from(&mut self, start_node: u32, delta: u32) {
		for node in start_node as usize..self.node_count() {
			let node = node as u32;
			let mask = self.node_mask(node);
			for child in 0..super::SIZE_CUBED {
				if (mask & (1u64 << child)) == 0 { continue; }
				let next = self.child_index(node, child) + delta;
				self.cell_bytes_mut(node, child)[..CHILD_INDEX_BYTES].copy_from_slice(&next.to_le_bytes());
			}
		}
		for free in &mut self.free_nodes {
			*free += delta;
		}
	}

	fn read_u64_at(&self, offset: usize) -> u64 {
		u64::from_le_bytes(self.buffer[offset..offset + 8].try_into().expect("u64 field length"))
	}
	fn write_u64_at(&mut self, offset: usize, value: u64) {
		self.buffer[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
	}
	fn read_u32_at(&self, offset: usize) -> u32 {
		u32::from_le_bytes(self.buffer[offset..offset + 4].try_into().expect("u32 field length"))
	}
	fn write_u32_at(&mut self, offset: usize, value: u32) {
		self.buffer[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
	}
	fn read_u8_at(&self, offset: usize) -> u8 { self.buffer[offset] }
	fn write_u8_at(&mut self, offset: usize, value: u8) { self.buffer[offset] = value; }
}

#[derive(Clone, Copy, Debug)]
pub struct GridTreeNode<'a> {
	tree: &'a RawGridTree,
	index: u32,
}

impl<'a> GridTreeNode<'a> {
	pub(crate) fn new(tree: &'a RawGridTree, index: u32) -> Self { Self { tree, index } }
	pub fn index(self) -> u32 { self.index }
	pub fn data_mask(self) -> u64 { self.tree.data_mask(self.index) }
	pub fn node_mask(self) -> u64 { self.tree.node_mask(self.index) }
	pub fn parent_offset(self) -> u32 { self.tree.parent_offset(self.index) }
	pub fn used_cell_count(self) -> u8 { self.tree.used_cell_count(self.index) }
	pub fn kind(self, cell_index: u8) -> CellKind { self.tree.cell_kind(self.index, cell_index) }
	pub fn child_index(self, cell_index: u8) -> Option<u32> {
		(self.kind(cell_index) == CellKind::Node).then(|| self.tree.child_index(self.index, cell_index))
	}
	pub fn cell_bytes(self, cell_index: u8) -> &'a [u8] { self.tree.cell_bytes(self.index, cell_index) }
}
