//! Float BVH data in render-origin-relative coordinates.

use std::fmt::Debug;

use bevy::math::Vec3;

mod building;

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
	pub min_corner: Vec3,
	pub max_corner: Vec3,
	pub sub_nodes: BVHInternal,
}

#[derive(Debug)]
pub struct BVH<Index: Copy + Debug + PartialEq> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (Vec3, Vec3))>,
}

impl<Index: Copy + Debug + PartialEq> BVH<Index> {
	pub fn internals(&self) -> (&Vec<BVHNode>, &Vec<(Index, (Vec3, Vec3))>) {
		(&self.nodes, &self.items)
	}
}
