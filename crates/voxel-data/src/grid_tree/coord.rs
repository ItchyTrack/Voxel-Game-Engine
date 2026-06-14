use std::fmt::Debug;

use bevy::math::IVec3;

/// The coordinate system a tree is keyed in. The tree stores and traverses
/// everything in `IVec3`/`u32` internally; this trait only bridges the public
/// position/size types at the API boundary, plus the per-system depth cap.
pub trait GridCoord: Copy + Debug + 'static {
    /// Public signed world-space position vector (e.g. `I16Vec3` or `IVec3`).
    type Pos: Copy + Eq + Debug;
    /// Public size scalar yielded for cells (e.g. `u16` or `u32`).
    type Size: Copy + Eq + Ord + Debug;
    /// Highest root depth allowed before an out-of-range insert is skipped.
    const MAX_ROOT_DEPTH: u8;
    fn to_ivec3(pos: Self::Pos) -> IVec3;
    fn from_ivec3(v: IVec3) -> Self::Pos;
    fn size_from_u32(s: u32) -> Self::Size;
}

/// `i16` coordinate system (world voxels).
#[derive(Clone, Copy, Debug)]
pub struct I16Coord;
impl GridCoord for I16Coord {
    type Pos = bevy::math::I16Vec3;
    type Size = u16;
    // size(7) = 1<<16 is the first depth no i16 span can reach, so 6 is the cap.
    const MAX_ROOT_DEPTH: u8 = 6;
    fn to_ivec3(pos: Self::Pos) -> IVec3 {
        pos.as_ivec3()
    }
    fn from_ivec3(v: IVec3) -> Self::Pos {
        v.as_i16vec3()
    }
    fn size_from_u32(s: u32) -> Self::Size {
        s as u16
    }
}

/// `i32` coordinate system (chunk-space and other coarse grids).
#[derive(Clone, Copy, Debug)]
pub struct I32Coord;
impl GridCoord for I32Coord {
    type Pos = IVec3;
    type Size = u32;
    // size(15) = 1<<32 overflows u32; the node-arena cap bounds us long before this.
    const MAX_ROOT_DEPTH: u8 = 13;
    fn to_ivec3(pos: Self::Pos) -> IVec3 {
        pos
    }
    fn from_ivec3(v: IVec3) -> Self::Pos {
        v
    }
    fn size_from_u32(s: u32) -> Self::Size {
        s
    }
}
