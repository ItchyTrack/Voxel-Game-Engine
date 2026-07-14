use std::collections::HashSet;

use bevy::math::IVec3;

use super::{CellKind, GridCell, GridCoord, GridTreeView, NodeRef};

#[inline]
pub fn get<C: GridCell, Co: GridCoord>(view: GridTreeView<'_, C, Co>, pos: Co::Pos) -> Option<C::Data> {
	let root_relative_pos = Co::to_ivec3(pos) - view.root_origin();
	if root_relative_pos.is_negative_bitmask() != 0 {
		return None;
	}
	let root_relative_pos = root_relative_pos.as_uvec3();
	let root_size = super::size(view.root_depth());
	if root_relative_pos.x >= root_size || root_relative_pos.y >= root_size || root_relative_pos.z >= root_size {
		return None;
	}

	let mut current = view.root();
	let mut current_relative_pos = root_relative_pos;
	loop {
		let contents_pos = (current_relative_pos / super::child_size(current.depth)).as_u8vec3();
		let cell = view.child(current, super::get_child_contents_index(contents_pos));
		match cell.kind() {
			CellKind::Empty => return None,
			CellKind::Data => return Some(cell.data_value()),
			CellKind::Node => {
				current_relative_pos %= super::child_size(current.depth);
				current = view.child_node(cell)?;
			}
		}
	}
}

/// Visit every DATA leaf whose cell box intersects inclusive region `[min, max]`.
#[inline]
pub fn for_each_in_region<C, Co, F>(view: GridTreeView<'_, C, Co>, min: Co::Pos, max: Co::Pos, mut f: F)
where
	C: GridCell,
	Co: GridCoord,
	F: FnMut(Co::Pos, Co::Size, C::Data),
{
	if view.is_empty() {
		return;
	}
	let (min, max) = (Co::to_ivec3(min), Co::to_ivec3(max));
	region_recurse(view, view.root(), min, max, &mut |o, s, v| f(Co::from_ivec3(o), Co::size_from_u32(s), v));
}

/// Visit every occupied cell (internal node or data leaf) whose cell box
/// intersects inclusive region `[min, max]`.
#[inline]
pub fn for_each_node_in_region<C, Co, F>(view: GridTreeView<'_, C, Co>, min: Co::Pos, max: Co::Pos, mut f: F)
where
	C: GridCell,
	Co: GridCoord,
	F: FnMut(Co::Pos, Co::Size, bool),
{
	if view.is_empty() {
		return;
	}
	let (min, max) = (Co::to_ivec3(min), Co::to_ivec3(max));
	node_region_recurse(view, view.root(), min, max, &mut |o, s, is_leaf| f(Co::from_ivec3(o), Co::size_from_u32(s), is_leaf));
}

#[inline]
pub fn any_in_region<C: GridCell, Co: GridCoord>(view: GridTreeView<'_, C, Co>, min: Co::Pos, max: Co::Pos) -> bool {
	if view.is_empty() {
		return false;
	}
	let (min, max) = (Co::to_ivec3(min), Co::to_ivec3(max));
	region_any_recurse(view, view.root(), min, max)
}

#[inline]
pub fn is_area_filled<C: GridCell, Co: GridCoord>(view: GridTreeView<'_, C, Co>, pos: Co::Pos, size: IVec3) -> bool {
	if size.cmple(IVec3::ZERO).any() {
		return true;
	}
	if view.is_empty() {
		return false;
	}
	let min = Co::to_ivec3(pos);
	let end = min + size;
	let root = view.root();
	let root_end = root.origin + IVec3::splat(super::size(root.depth) as i32);
	if min.cmplt(root.origin).any() || end.cmpgt(root_end).any() {
		return false;
	}
	region_filled_recurse(view, root, min, end)
}

#[inline]
fn region_recurse<C, Co, F>(view: GridTreeView<'_, C, Co>, node: NodeRef, min: IVec3, max: IVec3, f: &mut F)
where
	C: GridCell,
	Co: GridCoord,
	F: FnMut(IVec3, u32, C::Data),
{
	for child in view.occupied_children(node) {
		let child_end = child.origin + IVec3::splat(child.size as i32); // exclusive
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		match child.kind() {
			CellKind::Data => f(child.origin, child.size, child.data_value()),
			CellKind::Node => region_recurse(view, view.child_node(child).expect("node cell has child"), min, max, f),
			CellKind::Empty => unreachable!(),
		}
	}
}

#[inline]
fn node_region_recurse<C, Co, F>(view: GridTreeView<'_, C, Co>, node: NodeRef, min: IVec3, max: IVec3, f: &mut F)
where
	C: GridCell,
	Co: GridCoord,
	F: FnMut(IVec3, u32, bool),
{
	for child in view.occupied_children(node) {
		let child_end = child.origin + IVec3::splat(child.size as i32); // exclusive
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		let is_leaf = child.kind() == CellKind::Data;
		f(child.origin, child.size, is_leaf);
		if child.kind() == CellKind::Node {
			node_region_recurse(view, view.child_node(child).expect("node cell has child"), min, max, f);
		}
	}
}

#[inline]
fn region_any_recurse<C: GridCell, Co: GridCoord>(view: GridTreeView<'_, C, Co>, node: NodeRef, min: IVec3, max: IVec3) -> bool {
	for child in view.occupied_children(node) {
		let child_end = child.origin + IVec3::splat(child.size as i32); // exclusive
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		match child.kind() {
			CellKind::Data => return true,
			CellKind::Node => {
				if region_any_recurse(view, view.child_node(child).expect("node cell has child"), min, max) {
					return true;
				}
			}
			CellKind::Empty => unreachable!(),
		}
	}
	false
}

#[inline]
pub fn for_each_occupied_tile_cover<C, Co, F>(
	view: GridTreeView<'_, C, Co>,
	min: Co::Pos,
	max: Co::Pos,
	tile_size: i32,
	mut f: F,
) where
	C: GridCell,
	Co: GridCoord,
	F: FnMut(IVec3),
{
	if view.is_empty() || tile_size <= 0 {
		return;
	}
	let (min, max) = (Co::to_ivec3(min), Co::to_ivec3(max));
	let first = min.div_euclid(IVec3::splat(tile_size));
	let last = max.div_euclid(IVec3::splat(tile_size));
	if first == last {
		if region_any_recurse(view, view.root(), min, max) {
			f(first * tile_size);
		}
		return;
	}
	let mut seen = HashSet::new();
	occupied_tile_cover_recurse(view, view.root(), min, max, tile_size, &mut seen, &mut f);
}

#[inline]
fn occupied_tile_cover_recurse<C, Co, F>(
	view: GridTreeView<'_, C, Co>,
	node: NodeRef,
	min: IVec3,
	max: IVec3,
	tile_size: i32,
	seen: &mut HashSet<IVec3>,
	f: &mut F,
) where
	C: GridCell,
	Co: GridCoord,
	F: FnMut(IVec3),
{
	let node_end = node.origin + IVec3::splat(super::size(node.depth) as i32);
	if node.origin.cmpgt(max).any() || node_end.cmple(min).any() {
		return;
	}
	for child in view.occupied_children(node) {
		let child_end = child.origin + IVec3::splat(child.size as i32);
		if child.origin.cmpgt(max).any() || child_end.cmple(min).any() {
			continue;
		}
		let overlap_min = child.origin.max(min);
		let overlap_max = (child_end - IVec3::ONE).min(max);
		let first = overlap_min.div_euclid(IVec3::splat(tile_size));
		let last = overlap_max.div_euclid(IVec3::splat(tile_size));
		match child.kind() {
			CellKind::Data => {
				for x in first.x..=last.x {
					for y in first.y..=last.y {
						for z in first.z..=last.z {
							let tile_min = IVec3::new(x, y, z) * tile_size;
							if seen.insert(tile_min) {
								f(tile_min);
							}
						}
					}
				}
			}
			CellKind::Node => occupied_tile_cover_recurse(
				view,
				view.child_node(child).expect("node cell has child"),
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

fn region_filled_recurse<C: GridCell, Co: GridCoord>(view: GridTreeView<'_, C, Co>, node: NodeRef, min: IVec3, end: IVec3) -> bool {
	for child in view.children(node) {
		let child_end = child.origin + IVec3::splat(child.size as i32);
		if child.origin.cmpge(end).any() || child_end.cmple(min).any() {
			continue;
		}
		match child.kind() {
			CellKind::Empty => return false,
			CellKind::Data => {}
			CellKind::Node => {
				if !region_filled_recurse(view, view.child_node(child).expect("node cell has child"), min, end) {
					return false;
				}
			}
		}
	}
	true
}
