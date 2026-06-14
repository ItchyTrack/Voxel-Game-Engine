use std::marker::PhantomData;

use bevy::math::IVec3;

use super::{CellKind, GridCell, GridCoord, GridTreeNode, MAX_TREE_DEPTH_USIZE, SIZE_CUBED, child_size, get_child_contents_pos};

/// Borrowed, read-only view over a grid tree's raw node arena.
///
/// This is the low-level API algorithms should use when they need to traverse
/// tree structure directly. It intentionally stores only references and small
/// copyable metadata so callers pay the same cost as indexing the arena by hand.
#[derive(Clone, Copy, Debug)]
pub struct GridTreeView<'a, C: GridCell, Co: GridCoord> {
    nodes: &'a [GridTreeNode<C>],
    root_pos: IVec3,
    root_depth: u8,
    item_count: u64,
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
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct CellRef<C: GridCell> {
    pub parent: NodeRef,
    pub child_index: u8,
    pub origin: IVec3,
    pub size: u32,
    pub cell: C,
}

impl<C: GridCell> CellRef<C> {
    #[inline]
    pub fn kind(self) -> CellKind {
        self.cell.kind()
    }

    #[inline]
    pub fn data_value(self) -> C::Data {
        self.cell.data_value()
    }

    #[inline]
    pub fn node_offset(self) -> u32 {
        self.cell.node_offset()
    }
}

impl<'a, C: GridCell, Co: GridCoord> GridTreeView<'a, C, Co> {
    #[inline]
    pub(crate) fn new(nodes: &'a [GridTreeNode<C>], root_pos: IVec3, root_depth: u8, item_count: u64) -> Self {
        Self { nodes, root_pos, root_depth, item_count, _coord: PhantomData }
    }

    #[inline]
    pub fn nodes(self) -> &'a [GridTreeNode<C>] {
        self.nodes
    }

    #[inline]
    pub fn root(self) -> NodeRef {
        NodeRef { index: 0, depth: self.root_depth, origin: self.root_pos }
    }

    #[inline]
    pub fn root_pos(self) -> Co::Pos {
        Co::from_ivec3(self.root_pos)
    }

    #[inline]
    pub fn root_origin(self) -> IVec3 {
        self.root_pos
    }

    #[inline]
    pub fn root_depth(self) -> u8 {
        self.root_depth
    }

    #[inline]
    pub fn len(self) -> u64 {
        self.item_count
    }

    #[inline]
    pub fn is_empty(self) -> bool {
        self.item_count == 0
    }

    #[inline]
    pub fn node(self, node: NodeRef) -> &'a GridTreeNode<C> {
        &self.nodes[node.index as usize]
    }

    #[inline]
    pub fn child(self, node: NodeRef, child_index: u8) -> CellRef<C> {
        let size = child_size(node.depth);
        let origin = node.origin + (get_child_contents_pos(child_index).as_uvec3() * size).as_ivec3();
        CellRef { parent: node, child_index, origin, size, cell: self.node(node).contents[child_index as usize] }
    }

    #[inline]
    pub fn child_node(self, cell: CellRef<C>) -> Option<NodeRef> {
        if cell.kind() != CellKind::Node {
            return None;
        }
        Some(NodeRef { index: cell.parent.index + cell.node_offset(), depth: cell.parent.depth.saturating_sub(1), origin: cell.origin })
    }

    #[inline]
    pub fn children(self, node: NodeRef) -> ChildCells<'a, C, Co> {
        ChildCells { view: self, node, next: 0, occupied_only: false }
    }

    #[inline]
    pub fn occupied_children(self, node: NodeRef) -> ChildCells<'a, C, Co> {
        ChildCells { view: self, node, next: 0, occupied_only: true }
    }

    #[inline]
    pub fn leaves(self) -> LeafCells<'a, C, Co> {
        LeafCells::new(self)
    }
}

/// Concrete iterator over a node's child cells.
pub struct ChildCells<'a, C: GridCell, Co: GridCoord> {
    view: GridTreeView<'a, C, Co>,
    node: NodeRef,
    next: u8,
    occupied_only: bool,
}

impl<'a, C: GridCell, Co: GridCoord> Iterator for ChildCells<'a, C, Co> {
    type Item = CellRef<C>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while self.next < SIZE_CUBED {
            let i = self.next;
            self.next += 1;
            let cell = self.view.child(self.node, i);
            if !self.occupied_only || cell.kind() != CellKind::Empty {
                return Some(cell);
            }
        }
        None
    }
}

#[derive(Clone, Copy, Debug)]
struct LeafFrame {
    node: NodeRef,
    next_child: u8,
}

/// Depth-first iterator over DATA leaves. Uses a fixed stack sized by the tree
/// depth cap, avoiding per-iterator heap allocation.
pub struct LeafCells<'a, C: GridCell, Co: GridCoord> {
    view: GridTreeView<'a, C, Co>,
    stack: [LeafFrame; MAX_TREE_DEPTH_USIZE + 1],
    stack_len: usize,
}

impl<'a, C: GridCell, Co: GridCoord> LeafCells<'a, C, Co> {
    #[inline]
    fn new(view: GridTreeView<'a, C, Co>) -> Self {
        let root = view.root();
        Self { view, stack: [LeafFrame { node: root, next_child: 0 }; MAX_TREE_DEPTH_USIZE + 1], stack_len: (!view.nodes().is_empty()) as usize }
    }
}

impl<'a, C: GridCell, Co: GridCoord> Iterator for LeafCells<'a, C, Co> {
    type Item = CellRef<C>;

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
