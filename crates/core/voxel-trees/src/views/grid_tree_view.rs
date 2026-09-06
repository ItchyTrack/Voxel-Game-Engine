use bevy::math::{UVec3, I8Vec3, U8Vec3};
use bevy::prelude::*;
use std::fmt::Debug;
use crate::region::NonZeroVoxelRegion;
use super::GridView;
use std::collections::HashSet;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CellKind { Empty, Data, Node }

/// Where a node sits during traversal.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodeRef<H> {
	pub handle: H,
	pub depth: u8,
	pub origin: UVec3,
}

/// A child slot.
#[derive(Clone, Copy, Debug)]
pub struct Cell<'tree, View: GridTreeView> {
	pub depth: u8,
	pub child_index: u8,
	pub origin: UVec3,
	pub kind: CellKind,
	pub child_handle: Option<View::NodeHandle>,
}

impl<'tree, View: GridTreeView> Cell<'tree, View> {
	pub fn size(self) -> u32 {
		View::child_size(self.depth)
	}

	pub fn node_ref(self) -> Option<NodeRef<View::NodeHandle>> {
		if self.kind != CellKind::Node {
			return None;
		}
		self.child_handle.map(|handle| NodeRef { handle, depth: self.depth.saturating_sub(1), origin: self.origin })
	}
}

pub trait GridTreeView: Sized + Debug {
	type NodeHandle: Copy + Clone + Eq + Debug;
	type Data<'d>: Copy where Self: 'd;

	/// log2(node edge size) — 1
	/// Octree: 2
	/// 64Tree: 4
	const BRANCH_LOG2: u8;
	const MAX_DEPTH: u8 = 10;

	fn is_empty(&self) -> bool;

	fn root_depth(&self) -> u8;

	fn root_pos(&self) -> UVec3;

	fn root(&self) -> NodeRef<Self::NodeHandle>;

	fn cell_kind(&self, node: Self::NodeHandle, child_index: u8) -> CellKind;

	fn child_handle(&self, node: Self::NodeHandle, child_index: u8) -> Self::NodeHandle;

	fn cell_data<'tree>(&'tree self, node: Self::NodeHandle, child_index: u8) -> Self::Data<'tree>;

	fn occupancy_mask(&self, node: Self::NodeHandle) -> u64;

	// ---- provided, built only from the required methods above ----

	#[inline]
	fn node_edge_size() -> u8 { 1u8 << Self::BRANCH_LOG2 }

	#[inline]
	fn children_per_node() -> u8 {
		let n = Self::node_edge_size();
		n * n * n
	}

	#[inline]
	fn child_index_of(pos: U8Vec3) -> u8 {
		let n = Self::node_edge_size();
		pos.x + pos.y * n + pos.z * n * n
	}

	#[inline]
	fn child_offset_of(index: u8) -> U8Vec3 {
		let n = Self::node_edge_size();
		U8Vec3::new(index % n, (index / n) % n, index / (n * n))
	}

	#[inline]
	fn size(node_depth: u8) -> u32 { Self::child_size(node_depth + 1) }

	#[inline]
	fn child_size(depth: u8) -> u32 { 1 << (Self::BRANCH_LOG2 * depth) }

	fn child(&self, node: NodeRef<Self::NodeHandle>, child_index: u8) -> Cell<'tree, Self> {
		let size = Self::child_size(node.depth);
		let origin = node.origin + Self::child_offset_of(child_index).as_uvec3() * size;
		let kind = self.cell_kind(node.handle, child_index);
		let child_handle = match kind {
			CellKind::Empty => None,
			CellKind::Data => Some(node.handle),
			CellKind::Node => Some(self.child_handle(node.handle, child_index)),
		};
		Cell { depth: node.depth, child_index, origin, kind, child_handle }
	}

	fn children(self, node: NodeRef<Self::NodeHandle>) -> ChildCells<'tree, Self> { ChildCells::new(self, node, false) }

	fn occupied_children(self, node: NodeRef<Self::NodeHandle>) -> ChildCells<'tree, Self> { ChildCells::new(self, node, true) }

	fn occupied_children_in_region(self, node: NodeRef<Self::NodeHandle>, region: NonZeroVoxelRegion) -> ChildCellsInRegion<'tree, Self> {
		ChildCellsInRegion::new(self, node, region)
	}

	fn leaves(self) -> LeafCells<'tree, Self> { LeafCells::new(self) }

	/// Visit every occupied cell (internal node or data leaf) whose cell box intersects inclusive region `[min, max]`.
	fn for_each_node_in_region<F>(&self, min: UVec3, max: UVec3, mut f: F)
	where
		F: FnMut(UVec3, u32, bool),
	{
		if self.is_empty() {
			return;
		}
		let (min, max) = (min, max);
		node_region_recurse(self, self.root(), min, max, &mut |o, s, is_leaf| f(o, s, is_leaf));
	}

	fn for_each_leaf_in_region<'tree, F>(&'tree self, region: NonZeroVoxelRegion, mut f: F)
	where
		F: FnMut(UVec3, u32, Self::Data<'tree>),
	{
		if self.is_empty() {
			return;
		}
		region_recurse(self, self.root(), region.min().as_uvec3(), region.max().as_uvec3(), &mut f);
	}

	fn for_each_occupied_tile_cover<F>(
		&self,
		min: UVec3,
		max: UVec3,
		tile_size: u32,
		mut f: F,
	) where
		F: FnMut(UVec3),
	{
		if self.is_empty() || tile_size <= 0 {
			return;
		}
		let first = min / UVec3::splat(tile_size);
		let last = max / UVec3::splat(tile_size);
		if first == last {
			if region_any_recurse(self, self.root(), min, max) {
				f(first * tile_size);
			}
			return;
		}
		let mut seen = HashSet::new();
		occupied_tile_cover_recurse(self, self.root(), min, max, tile_size, &mut seen, &mut f);
	}

	fn raycast(&self, transform: &Transform, max_length: Option<f32>) -> Option<(UVec3, I8Vec3, f32)>;
}

fn region_recurse<'tree, View: GridTreeView + 'tree, F>(view: &View, node: NodeRef<View::NodeHandle>, min: UVec3, max: UVec3, f: &mut F)
where
	F: FnMut(UVec3, u32, View::Data<'tree>),
{
	for child in view.occupied_children(node) {
		let child_end = child.origin + UVec3::splat(child.size()); // exclusive
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		match child.kind {
			CellKind::Data => f(child.origin, child.size(), view.cell_data(node.handle, child.child_index)),
			CellKind::Node => region_recurse(view, child.node_ref().expect("node cell has child"), min, max, f),
			CellKind::Empty => unreachable!(),
		}
	}
}

fn node_region_recurse<'tree, View: GridTreeView, F>(view: &View, node: NodeRef<View::NodeHandle>, min: UVec3, max: UVec3, f: &mut F)
where
	F: FnMut(UVec3, u32, bool),
{
	for child in view.occupied_children(node) {
		let child_end = child.origin + UVec3::splat(child.size()); // exclusive
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		let is_leaf = child.kind == CellKind::Data;
		f(child.origin, child.size(), is_leaf);
		if child.kind == CellKind::Node {
			node_region_recurse(view, child.node_ref().expect("node cell has child"), min, max, f);
		}
	}
}

fn region_any_recurse<'tree, View: GridTreeView>(view: &View, node: NodeRef<View::NodeHandle>, min: UVec3, max: UVec3) -> bool {
	for child in view.occupied_children(node) {
		let child_end = child.origin + UVec3::splat(child.size()); // exclusive
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		match child.kind {
			CellKind::Data => return true,
			CellKind::Node => {
				if region_any_recurse(view, child.node_ref().expect("node cell has child"), min, max) {
					return true;
				}
			}
			CellKind::Empty => unreachable!(),
		}
	}
	false
}

#[inline]
fn occupied_tile_cover_recurse<'tree, View: GridTreeView, F>(
	view: &View,
	node: NodeRef<View::NodeHandle>,
	min: UVec3,
	max: UVec3,
	tile_size: u32,
	seen: &mut HashSet<UVec3>,
	f: &mut F,
) where
	F: FnMut(UVec3),
{
	let node_end = node.origin + UVec3::splat(View::size(node.depth));
	if node.origin.cmpgt(max).any() || node_end.cmple(min).any() {
		return;
	}
	for child in view.occupied_children(node) {
		let child_end = child.origin + UVec3::splat(child.size());
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		let overlap_min = child.origin.max(min);
		let overlap_max = (child_end - UVec3::ONE).min(max);
		let first = overlap_min / UVec3::splat(tile_size);
		let last = overlap_max / UVec3::splat(tile_size);
		match child.kind {
			CellKind::Data => {
				for x in first.x..=last.x {
					for y in first.y..=last.y {
						for z in first.z..=last.z {
							let tile_min = UVec3::new(x, y, z) * tile_size;
							if seen.insert(tile_min) {
								f(tile_min);
							}
						}
					}
				}
			}
			CellKind::Node => occupied_tile_cover_recurse(
				view,
				child.node_ref().expect("node cell has child"),
				min,
				max,
				tile_size,
				seen,
				f,
			),
			CellKind::Empty => unreachable!(),
		}
	}
}

fn region_filled_recurse<'tree, View: GridTreeView>(view: &View, node: NodeRef<View::NodeHandle>, min: UVec3, end: UVec3) -> bool {
	for child in view.children(node) {
		let child_end = child.origin + UVec3::splat(child.size());
		if child.origin.cmpge(end).any() || child_end.cmple(min).any() {
			continue;
		}
		match child.kind {
			CellKind::Empty => return false,
			CellKind::Data => {}
			CellKind::Node => {
				if !region_filled_recurse(view, child.node_ref().expect("node cell has child"), min, end) {
					return false;
				}
			}
		}
	}
	true
}

pub struct ChildCells<'tree, View: GridTreeView> {
	view: View,
	node: NodeRef<View::NodeHandle>,
	next: u8,
	occupied_only: bool,
	remaining_mask: u64,
}

impl<'tree, View: GridTreeView> ChildCells<'tree, View> {
	#[inline]
	fn new(view: View, node: NodeRef<View::NodeHandle>, occupied_only: bool) -> Self {
		let remaining_mask = view.occupancy_mask(node.handle);
		Self { view, node, next: 0, occupied_only, remaining_mask }
	}
}

impl<'tree, View: GridTreeView> Iterator for ChildCells<'tree, View> {
	type Item = Cell<'tree, View>;

	#[inline]
	fn next(&mut self) -> Option<Self::Item> {
		let count = View::children_per_node();
		if self.occupied_only {
			let idx = self.remaining_mask.trailing_zeros() as u8;
			if idx >= count { return None; }
			self.remaining_mask &= self.remaining_mask - 1;
			return Some(self.view.child(self.node, idx));
		}
		if self.next >= count { return None; }
		let idx = self.next;
		self.next += 1;
		return Some(self.view.child(self.node, idx));
	}
}

pub struct ChildCellsInRegion<'tree, View: GridTreeView> {
	view: View,
	node: NodeRef<View::NodeHandle>,
	region: NonZeroVoxelRegion,
	remaining_mask: u64,
}

impl<'tree, View: GridTreeView> Copy for ChildCellsInRegion<'tree, View> {}
impl<'tree, View: GridTreeView> Clone for ChildCellsInRegion<'tree, View> {
	fn clone(&self) -> Self { *self }
}

impl<'tree, View: GridTreeView> ChildCellsInRegion<'tree, View> {
	fn new(view: View, node: NodeRef<View::NodeHandle>, region: NonZeroVoxelRegion) -> Self {
		assert!(region.min().is_negative_bitmask() == 0);
		let node_region = NonZeroVoxelRegion::from_min_size(node.origin.as_ivec3(), UVec3::splat(View::size(node.depth))).unwrap();
		let Some(overlap) = node_region.intersection(region) else {
			return Self { view, node, region, remaining_mask: 0 };
		};
		let cell_size = View::child_size(node.depth);
		let min = (overlap.min().as_uvec3() - node.origin) / UVec3::splat(cell_size);
		let max = (overlap.end().as_uvec3() - node.origin - UVec3::ONE) / UVec3::splat(cell_size);

		let mut region_mask = 0u64;
		for z in min.z..=max.z {
			for y in min.y..=max.y {
				for x in min.x..=max.x {
					region_mask |= 1u64 << View::child_index_of(UVec3::new(x, y, z).as_u8vec3());
				}
			}
		}

		Self {
			view,
			node,
			region,
			remaining_mask: view.occupancy_mask(node.handle) & region_mask
		}
	}

	fn resolve(&self, child_index: u8) -> Option<(Cell<'tree, View>, NonZeroVoxelRegion)> {
		let child = self.view.child(self.node, child_index);
		if child.kind == CellKind::Empty { return None; }
		let child_region = NonZeroVoxelRegion::from_min_size(child.origin.as_ivec3(), UVec3::splat(child.size())).unwrap();
		let clipped = child_region.intersection(self.region).expect("masked child intersects region");
		Some((child, clipped))
	}
}

impl<'tree, View: GridTreeView> Iterator for ChildCellsInRegion<'tree, View> {
	type Item = (Cell<'tree, View>, NonZeroVoxelRegion);

	#[inline]
	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let idx = self.remaining_mask.trailing_zeros() as u8;
			if idx >= View::children_per_node() { return None; }
			self.remaining_mask &= self.remaining_mask - 1;
			return self.resolve(idx);
		}
	}
}

struct LeafFrame<H> {
	node: NodeRef<H>,
	next_child: u8,
}
impl<H: Copy> Copy for LeafFrame<H> {}
impl<H: Copy> Clone for LeafFrame<H> {
	fn clone(&self) -> Self { *self }
}

/// Depth-first iterator over DATA leaves. Fixed stack sized by `View::MAX_DEPTH`
pub struct LeafCells<'tree, View: GridTreeView> {
	view: View,
	stack: Vec<LeafFrame<View::NodeHandle>>,
	stack_len: usize,
}

impl<'tree, View: GridTreeView> LeafCells<'tree, View> {
	#[inline]
	fn new(view: View) -> Self {
		let root = view.root();
		let mut stack = Vec::with_capacity(View::MAX_DEPTH as usize + 1);
		let stack_len = if view.is_empty() { 0 } else { stack.push(LeafFrame { node: root, next_child: 0 }); 1 };
		Self { view, stack, stack_len }
	}

	pub fn get_data(&self, cell: &Cell<'tree, View>) -> Option<View::Data<'tree>> {
		Some(self.view.cell_data(cell.child_handle?, cell.child_index))
	}
}

impl<'tree, View: GridTreeView> Iterator for LeafCells<'tree, View> {
	type Item = Cell<'tree, View>;

	#[inline]
	fn next(&mut self) -> Option<Self::Item> {
		while self.stack_len > 0 {
			let frame_index = self.stack_len - 1;
			let frame = &mut self.stack[frame_index];
			if frame.next_child >= View::children_per_node() {
				self.stack_len -= 1;
				continue;
			}

			let child_index = frame.next_child;
			frame.next_child += 1;
			let node = frame.node;
			let child = self.view.child(node, child_index);
			match child.kind {
				CellKind::Empty => {}
				CellKind::Data => return Some(child),
				CellKind::Node => {
					if let Some(next_node) = child.node_ref() {
						let new_frame = LeafFrame { node: next_node, next_child: 0 };
						if self.stack_len < self.stack.len() {
							self.stack[self.stack_len] = new_frame;
						} else {
							self.stack.push(new_frame);
						}
						self.stack_len += 1;
					}
				}
			}
		}
		None
	}
}

impl<'tree, View: GridTreeView + 'tree> GridView<'tree> for View {
	type Data<'d> = View::Data<'d> where Self: 'd;

	fn get(&self, pos: UVec3) -> Option<Self::Data<'tree>> {
		if pos.cmplt(self.root_pos()).any() { return None; }
		let root_relative_pos = pos - self.root_pos();
		let root_size = View::size(self.root_depth());
		if root_relative_pos.x >= root_size || root_relative_pos.y >= root_size || root_relative_pos.z >= root_size {
			return None;
		}

		let mut current = self.root();
		let mut current_relative_pos = root_relative_pos;
		loop {
			let contents_pos = (current_relative_pos / Self::child_size(current.depth)).as_u8vec3();
			let cell = self.child(current, View::child_index_of(contents_pos));
			match cell.kind {
				CellKind::Empty => return None,
				CellKind::Data => return Some(self.cell_data(current.handle, cell.child_index)),
				CellKind::Node => {
					current_relative_pos %= Self::child_size(current.depth);
					current = cell.node_ref()?;
				}
			}
		}
	}

	fn bounds(&self) -> Option<NonZeroVoxelRegion> {
		if self.is_empty() {
			return None;
		}
		let root = self.root();
		NonZeroVoxelRegion::from_min_size(root.origin.as_ivec3(), UVec3::splat(Self::size(root.depth)))
	}

	fn for_each_in_region<F, 'tree>(&'tree self, region: NonZeroVoxelRegion, mut f: F)
	where
		F: FnMut(UVec3, Self::Data<'tree>),
	{
		self.for_each_leaf_in_region(region, &mut |pos: UVec3, size: u32, data| {
			let leaf_region = NonZeroVoxelRegion::from_min_size(pos.as_ivec3(), UVec3::splat(size)).unwrap();
			let overlap = leaf_region.intersection(region).expect("visited leaf intersects region");
			for x in overlap.min().x..overlap.end().x {
				for y in overlap.min().y..overlap.end().y {
					for z in overlap.min().z..overlap.end().z {
						f(IVec3::new(x, y, z).as_uvec3(), data);
					}
				}
			}
		});
	}

	fn any_in_region(&self, region: NonZeroVoxelRegion) -> bool {
		region_any_recurse(self, self.root(), region.min().as_uvec3(), region.max().as_uvec3())
	}

	fn is_region_filled(&self, region: NonZeroVoxelRegion) -> bool {
		if region.size().cmple(UVec3::ZERO).any() {
			return true;
		}
		if self.is_empty() {
			return false;
		}
		let root = self.root();
		let root_end = root.origin + UVec3::splat(Self::size(root.depth));
		let min = region.min().as_uvec3();
		let end = region.end().as_uvec3();
		if min.cmplt(root.origin).any() || end.cmpgt(root_end).any() {
			return false;
		}
		region_filled_recurse(self, root, min, end)
	}
}
