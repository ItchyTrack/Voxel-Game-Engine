use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_data::grid_tree::{CellKind, GridCell, GridTree, I32Coord};

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

pub type LodRequestGridTree = GridTree<LodRequestCell, I32Coord>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LodKey {
    pub grid: GridId,
    pub min: IVec3,
    pub size: IVec3,
    pub level: u32,
}

impl LodKey {
    pub fn new(grid: GridId, min: IVec3, size: IVec3, lod: f32) -> Self {
        Self::from_level(grid, min, size, lod.max(0.0).floor() as u32)
    }

    pub fn from_level(grid: GridId, min: IVec3, size: IVec3, level: u32) -> Self {
        Self { grid, min, size, level }
    }

    pub fn lod(self) -> f32 {
        self.level as f32
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LodAreaDelta {
    pub grid: GridId,
    pub min: IVec3,
    pub size: IVec3,
    pub old_lod: Option<u32>,
    pub new_lod: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LodVisibleKind {
    SubGrid,
    Lod,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LodVisibleDelta {
    pub grid: GridId,
    pub min: IVec3,
    pub size: IVec3,
    pub requested_lod: u32,
    pub actual_lod: u32,
    pub entity: Entity,
    pub kind: LodVisibleKind,
}
