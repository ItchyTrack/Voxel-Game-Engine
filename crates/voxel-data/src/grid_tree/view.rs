use std::marker::PhantomData;

use bevy::math::IVec3;

use super::{raw::RawGridTree, CellKind, GridCoord, GridRegion, GridTreeNode, GridType, MAX_TREE_DEPTH_USIZE, SIZE_CUBED, child_size, get_child_contents_index, get_child_contents_pos, size};

/// Borrowed, read-only view over a grid tree's raw node arena.
#[derive(Debug)]
pub struct GridTreeView<'a, G: GridType, Co: GridCoord> {
	grid_type: &'a G,
	raw: &'a RawGridTree,
	_coord: PhantomData<Co>,
}

/// A concrete node location in a [`GridTreeView`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodeRef {
	pub index: u32,
	pub depth: u8,
	pub origin: IVec3,
}

/// A concrete child cell location in a [`GridTreeView`].
#[derive(Debug)]
pub struct CellRef<'a, G: GridType> {
	pub parent: NodeRef,
	pub child_index: u8,
	pub origin: IVec3,
	pub size: u32,
	kind: CellKind,
	child_node_index: u32,
	grid_type: &'a G,
	bytes: &'a [u8],
}

impl<'a, G: GridType, Co: GridCoord> Copy for GridTreeView<'a, G, Co> {}
impl<'a, G: GridType, Co: GridCoord> Clone for GridTreeView<'a, G, Co> {
	fn clone(&self) -> Self { *self }
}
impl<'a, G: GridType> Copy for CellRef<'a, G> {}
impl<'a, G: GridType> Clone for CellRef<'a, G> {
	fn clone(&self) -> Self { *self }
}

impl<'a, G: GridType> CellRef<'a, G> {
	#[inline]
	pub fn kind(self) -> CellKind { self.kind }

	#[inline]
	pub fn data_value(self) -> G::Data<'a> { self.grid_type.read_data(self.bytes) }

	#[inline]
	pub fn node_index(self) -> u32 { self.child_node_index }

	#[inline]
	pub fn data_bytes(self) -> &'a [u8] { &self.bytes[..self.grid_type.data_size_bytes()] }
}

impl<'a, G: GridType, Co: GridCoord> GridTreeView<'a, G, Co> {
	#[inline]
	pub(crate) fn new(grid_type: &'a G, raw: &'a RawGridTree) -> Self {
		Self { grid_type, raw, _coord: PhantomData }
	}

	#[inline]
	pub fn grid_type(self) -> &'a G { self.grid_type }

	#[inline]
	pub fn raw(self) -> &'a RawGridTree { self.raw }

	#[inline]
	pub fn nodes(self) -> Vec<GridTreeNode<'a>> {
		(0..self.raw.node_count() as u32).map(|index| GridTreeNode::new(self.raw, index)).collect()
	}

	#[inline]
	pub fn root(self) -> NodeRef {
		NodeRef { index: 0, depth: self.raw.root_depth(), origin: self.raw.root_pos() }
	}

	#[inline]
	pub fn root_pos(self) -> Co::Pos { Co::from_ivec3(self.raw.root_pos()) }

	#[inline]
	pub fn root_origin(self) -> IVec3 { self.raw.root_pos() }

	#[inline]
	pub fn root_depth(self) -> u8 { self.raw.root_depth() }

	#[inline]
	pub fn len(self) -> u64 { self.raw.item_count() }

	#[inline]
	pub fn is_empty(self) -> bool { self.raw.item_count() == 0 }

	#[inline]
	pub fn node(self, node: NodeRef) -> GridTreeNode<'a> { GridTreeNode::new(self.raw, node.index) }

	#[inline]
	pub fn child(self, node: NodeRef, child_index: u8) -> CellRef<'a, G> {
		let size = child_size(node.depth);
		let origin = node.origin + (get_child_contents_pos(child_index).as_uvec3() * size).as_ivec3();
		let kind = self.raw.cell_kind(node.index, child_index);
		let child_node_index = if kind == CellKind::Node { self.raw.child_index(node.index, child_index) } else { 0 };
		CellRef { parent: node, child_index, origin, size, kind, child_node_index, grid_type: self.grid_type, bytes: self.raw.cell_bytes(node.index, child_index) }
	}

	#[inline]
	pub fn child_node(self, cell: CellRef<'a, G>) -> Option<NodeRef> {
		if cell.kind() != CellKind::Node { return None; }
		Some(NodeRef { index: cell.node_index(), depth: cell.parent.depth.saturating_sub(1), origin: cell.origin })
	}

	#[inline]
	pub fn children(self, node: NodeRef) -> ChildCells<'a, G, Co> {
		ChildCells::new(self, node, false)
	}

	#[inline]
	pub fn occupied_children(self, node: NodeRef) -> ChildCells<'a, G, Co> {
		ChildCells::new(self, node, true)
	}

	#[inline]
	pub fn occupied_children_in_region(self, node: NodeRef, region: GridRegion) -> ChildCellsInRegion<'a, G, Co> {
		ChildCellsInRegion::new(self, node, region)
	}

	#[inline]
	pub fn leaves(self) -> LeafCells<'a, G, Co> { LeafCells::new(self) }
}

pub struct ChildCells<'a, G: GridType, Co: GridCoord> {
	view: GridTreeView<'a, G, Co>,
	node: NodeRef,
	next: u8,
	occupied_only: bool,
	data_mask: u64,
	node_mask: u64,
	remaining_mask: u64,
}

impl<'a, G: GridType, Co: GridCoord> ChildCells<'a, G, Co> {
	#[inline]
	fn new(view: GridTreeView<'a, G, Co>, node: NodeRef, occupied_only: bool) -> Self {
		let data_mask = view.raw.data_mask(node.index);
		let node_mask = view.raw.node_mask(node.index);
		Self { view, node, next: 0, occupied_only, data_mask, node_mask, remaining_mask: data_mask | node_mask }
	}

	#[inline]
	fn child(&self, child_index: u8) -> CellRef<'a, G> {
		let bit = 1u64 << child_index;
		let size = child_size(self.node.depth);
		let origin = self.node.origin + (get_child_contents_pos(child_index).as_uvec3() * size).as_ivec3();
		let kind = if self.data_mask & bit != 0 {
			CellKind::Data
		} else if self.node_mask & bit != 0 {
			CellKind::Node
		} else {
			CellKind::Empty
		};
		let child_node_index = if kind == CellKind::Node { self.view.raw.child_index(self.node.index, child_index) } else { 0 };
		CellRef {
			parent: self.node,
			child_index,
			origin,
			size,
			kind,
			child_node_index,
			grid_type: self.view.grid_type,
			bytes: self.view.raw.cell_bytes(self.node.index, child_index),
		}
	}
}

impl<'a, G: GridType, Co: GridCoord> Iterator for ChildCells<'a, G, Co> {
	type Item = CellRef<'a, G>;

	#[inline]
	fn next(&mut self) -> Option<Self::Item> {
		if self.occupied_only {
			let child_index = self.remaining_mask.trailing_zeros() as u8;
			if child_index >= SIZE_CUBED {
				return None;
			}
			self.remaining_mask &= self.remaining_mask - 1;
			return Some(self.child(child_index));
		}
		if self.next >= SIZE_CUBED {
			return None;
		}
		let child_index = self.next;
		self.next += 1;
		Some(self.child(child_index))
	}
}

pub struct ChildCellsInRegion<'a, G: GridType, Co: GridCoord> {
	view: GridTreeView<'a, G, Co>,
	node: NodeRef,
	region: GridRegion,
	data_mask: u64,
	remaining_mask: u64,
}

impl<'a, G: GridType, Co: GridCoord> Copy for ChildCellsInRegion<'a, G, Co> {}
impl<'a, G: GridType, Co: GridCoord> Clone for ChildCellsInRegion<'a, G, Co> {
	fn clone(&self) -> Self { *self }
}

impl<'a, G: GridType, Co: GridCoord> ChildCellsInRegion<'a, G, Co> {
	#[inline]
	fn new(view: GridTreeView<'a, G, Co>, node: NodeRef, region: GridRegion) -> Self {
		let data_mask = view.raw.data_mask(node.index);
		let occupied_mask = data_mask | view.raw.node_mask(node.index);
		let node_region = GridRegion { min: node.origin, end: node.origin + IVec3::splat(size(node.depth) as i32) };
		let Some(overlap) = node_region.intersection(region) else {
			return Self { view, node, region, data_mask, remaining_mask: 0 };
		};
		let cell_size = child_size(node.depth) as i32;
		let min = (overlap.min - node.origin).div_euclid(IVec3::splat(cell_size));
		let max = (overlap.end - node.origin - IVec3::ONE).div_euclid(IVec3::splat(cell_size));
		let mut region_mask = 0u64;
		for z in min.z..=max.z {
			for y in min.y..=max.y {
				for x in min.x..=max.x {
					let child_index = get_child_contents_index(IVec3::new(x, y, z).as_u8vec3());
					region_mask |= 1u64 << child_index;
				}
			}
		}
		Self { view, node, region, data_mask, remaining_mask: occupied_mask & region_mask }
	}
}

impl<'a, G: GridType, Co: GridCoord> Iterator for ChildCellsInRegion<'a, G, Co> {
	type Item = (CellRef<'a, G>, GridRegion);

	#[inline]
	fn next(&mut self) -> Option<Self::Item> {
		let child_index = self.remaining_mask.trailing_zeros() as u8;
		if child_index >= SIZE_CUBED {
			return None;
		}
		self.remaining_mask &= self.remaining_mask - 1;

		let bit = 1u64 << child_index;
		let size = child_size(self.node.depth);
		let origin = self.node.origin + (get_child_contents_pos(child_index).as_uvec3() * size).as_ivec3();
		let kind = if self.data_mask & bit != 0 { CellKind::Data } else { CellKind::Node };
		let child_node_index = if kind == CellKind::Node { self.view.raw.child_index(self.node.index, child_index) } else { 0 };
		let child = CellRef {
			parent: self.node,
			child_index,
			origin,
			size,
			kind,
			child_node_index,
			grid_type: self.view.grid_type,
			bytes: self.view.raw.cell_bytes(self.node.index, child_index),
		};
		let child_region = GridRegion { min: child.origin, end: child.origin + IVec3::splat(child.size as i32) };
		let clipped = child_region.intersection(self.region).expect("masked child intersects region");
		Some((child, clipped))
	}
}

#[derive(Clone, Copy, Debug)]
struct LeafFrame {
	node: NodeRef,
	next_child: u8,
}

/// Depth-first iterator over DATA leaves. Uses a fixed stack sized by the tree depth cap.
pub struct LeafCells<'a, G: GridType, Co: GridCoord> {
	view: GridTreeView<'a, G, Co>,
	stack: [LeafFrame; MAX_TREE_DEPTH_USIZE + 1],
	stack_len: usize,
}

impl<'a, G: GridType, Co: GridCoord> LeafCells<'a, G, Co> {
	#[inline]
	fn new(view: GridTreeView<'a, G, Co>) -> Self {
		let root = view.root();
		Self { view, stack: [LeafFrame { node: root, next_child: 0 }; MAX_TREE_DEPTH_USIZE + 1], stack_len: (view.raw.node_count() != 0) as usize }
	}
}

impl<'a, G: GridType, Co: GridCoord> Iterator for LeafCells<'a, G, Co> {
	type Item = CellRef<'a, G>;

	#[inline]
	fn next(&mut self) -> Option<Self::Item> {
		while self.stack_len > 0 {
			let frame_index = self.stack_len - 1;
			let frame = &mut self.stack[frame_index];
			if frame.next_child >= SIZE_CUBED {
				self.stack_len -= 1;
				continue;
			}

			let child_index = frame.next_child;
			frame.next_child += 1;
			let child = self.view.child(frame.node, child_index);
			match child.kind() {
				CellKind::Empty => {}
				CellKind::Data => return Some(child),
				CellKind::Node => {
					if let Some(node) = self.view.child_node(child) {
						debug_assert!(self.stack_len < self.stack.len());
						self.stack[self.stack_len] = LeafFrame { node, next_child: 0 };
						self.stack_len += 1;
					}
				}
			}
		}
		None
	}
}
