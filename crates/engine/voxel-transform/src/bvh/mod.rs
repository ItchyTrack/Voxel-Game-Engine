use std::fmt::Debug;

use voxel_math::FixedVec3;

mod building;
mod query;

pub use query::BVHRaycastIterator;

#[derive(Debug)]
pub enum BVHInternal {
	SubNodes {
		sub1: u16,
		sub2: u16,
	},
	Leaf {
		start: u16,
		count: u16,
	},
}

#[derive(Debug)]
pub struct BVHNode {
	pub min_corner: FixedVec3,
	pub max_corner: FixedVec3,
	pub sub_nodes: BVHInternal,
}

#[derive(Debug)]
pub struct BVH<Index: Copy + Debug + PartialEq> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (FixedVec3, FixedVec3))>,
}
