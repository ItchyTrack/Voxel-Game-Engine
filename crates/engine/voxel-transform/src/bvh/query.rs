use std::{cmp::Ordering, collections::BinaryHeap, fmt::Debug};

use voxel_math::{Fixed, FixedVec3, Transform, ray_aabb_intersection};
use tracy_client::span;

use super::{BVH, BVHInternal, BVHNode};

impl<Index: Copy + Debug + PartialEq> BVH<Index> {
	fn intersects(aabb_a: &(FixedVec3, FixedVec3), aabb_b: &(FixedVec3, FixedVec3)) -> bool {
		aabb_a.0.cmple(aabb_b.1).all() && aabb_a.1.cmpge(aabb_b.0).all()
	}

	pub fn collisions(&self, bounds: &(FixedVec3, FixedVec3)) -> Vec<Index> {
		let _zone = span!("BVH get collisions");
		let mut out: Vec<Index> = vec![];
		let mut stack = vec![0u16];

		while let Some(node_index) = stack.pop() {
			let node = &self.nodes[node_index as usize];

			if !Self::intersects(bounds, &(node.min_corner, node.max_corner)) { continue; }

			match node.sub_nodes {
				BVHInternal::SubNodes { sub1, sub2 } => {
					stack.push(sub1);
					stack.push(sub2);
				}
				BVHInternal::Leaf { start, count } => {
					out.extend(
						self.items[start as usize..start as usize + count as usize]
							.iter()
							.filter_map(|item|
								if Self::intersects(bounds, &item.1) { Some(item.0) } else { None }
							)
					);
				}
			}
		}

		out
	}

	/// Hits are ordered by their parameter along the ray's direction.
	pub fn raycast(&self, transform: &(impl Transform + ?Sized), max_length: Option<Fixed>) -> BVHRaycastIterator<'_, Index> {
		let start = transform.translation();
		let direction = transform.direction();
		let mut heap = BinaryHeap::new();
		let root = &self.nodes[0];
		if let Some(length) = ray_aabb_intersection(&start, &direction, &(root.min_corner, root.max_corner)) {
			if max_length.is_none_or(|max| length <= max) {
				heap.push(Candidate { length, entry: BVHEntry::Node(0) });
			}
		}
		BVHRaycastIterator { bvh: self, start, direction, max_length, heap }
	}

	pub fn internals(&self) -> (&Vec<BVHNode>, &Vec<(Index, (FixedVec3, FixedVec3))>) {
		(&self.nodes, &self.items)
	}
}

enum BVHEntry<Index> {
	Node(u16),
	Hit(Index),
}

struct Candidate<Index> {
	length: Fixed,
	entry: BVHEntry<Index>,
}

impl<Index> PartialEq for Candidate<Index> {
	fn eq(&self, other: &Self) -> bool { self.length == other.length }
}
impl<Index> Eq for Candidate<Index> {}
impl<Index> PartialOrd for Candidate<Index> {
	fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl<Index> Ord for Candidate<Index> {
	fn cmp(&self, other: &Self) -> Ordering { other.length.cmp(&self.length) }
}

pub struct BVHRaycastIterator<'a, Index: Copy + Debug + PartialEq> {
	bvh: &'a BVH<Index>,
	start: FixedVec3,
	direction: FixedVec3,
	max_length: Option<Fixed>,
	heap: BinaryHeap<Candidate<Index>>,
}

impl<'a, Index: Copy + Debug + PartialEq> Iterator for BVHRaycastIterator<'a, Index> {
	type Item = (Index, Fixed);

	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let Candidate { length, entry } = self.heap.pop()?;
			match entry {
				BVHEntry::Hit(index) => return Some((index, length)),
				BVHEntry::Node(idx) => {
					let node = &self.bvh.nodes[idx as usize];
					match node.sub_nodes {
						BVHInternal::SubNodes { sub1, sub2 } => {
							for child in [sub1, sub2] {
								let cn = &self.bvh.nodes[child as usize];
								if let Some(length) = ray_aabb_intersection(&self.start, &self.direction, &(cn.min_corner, cn.max_corner)) {
									if self.max_length.is_none_or(|max| length <= max) {
										self.heap.push(Candidate { length, entry: BVHEntry::Node(child) });
									}
								}
							}
						}
						BVHInternal::Leaf { start, count } => {
							for item in &self.bvh.items[start as usize..start as usize + count as usize] {
								if let Some(length) = ray_aabb_intersection(&self.start, &self.direction, &item.1) {
									if self.max_length.is_none_or(|max| length <= max) {
										self.heap.push(Candidate { length, entry: BVHEntry::Hit(item.0) });
									}
								}
							}
						}
					}
				}
			}
		}
	}
}
