use tile_data::CHUNK_SIZE;

pub(super) const PLANET_TILE_COUNT: usize = 1024;
pub(crate) const PLANET_RADIUS: f32 = 512.0;

// Each tangent grid is clipped by the spherical Voronoi cell around its
// Fibonacci point. These are only radial limits; x/y bounds are inferred from
// the cached Voronoi halfspaces per tile.
pub(super) const TILE_INWARD_DEPTH: i32 = 64;
pub(super) const TILE_OUTWARD_HEIGHT: i32 = 128;
pub(super) const TILE_BOUND_PADDING: i32 = CHUNK_SIZE * 2;
pub(super) const VORONOI_NEIGHBORS: usize = 32;
pub(super) const TERRAIN_HEIGHT: f32 = 80.0;
pub(super) const TILE_SHAPE_EPSILON: f32 = 0.001;
