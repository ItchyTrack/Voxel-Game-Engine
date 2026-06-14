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

#[inline]
pub fn any_in_region<C: GridCell, Co: GridCoord>(view: GridTreeView<'_, C, Co>, min: Co::Pos, max: Co::Pos) -> bool {
    if view.is_empty() {
        return false;
    }
    let (min, max) = (Co::to_ivec3(min), Co::to_ivec3(max));
    region_any_recurse(view, view.root(), min, max)
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
