use std::{cmp::Ordering, collections::BinaryHeap, fmt::Debug};

use bevy::math::Vec3;
use bevy::transform::components::Transform;
use tracy_client::span;

use super::{BVH, BVHInternal, BVHNode};

impl<Index: Copy + Debug + PartialEq> BVH<Index> {
	fn intersects(aabb_a: &(Vec3, Vec3), aabb_b: &(Vec3, Vec3)) -> bool {
		aabb_a.0.cmple(aabb_b.1).all() && aabb_a.1.cmpge(aabb_b.0).all()
	}

	pub fn collisions(&self, bounds: &(Vec3, Vec3)) -> Vec<Index> {
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
						self.items[start as usize..(start + count) as usize]
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

	fn ray_aabb_intersection(start: &Vec3, direction: &Vec3, aabb: &(Vec3, Vec3)) -> Option<f32> {
		let (min, max) = aabb;

		if start.cmpge(*min).all() && start.cmple(*max).all() {
			return Some(0.0);
		}

		let inv = Vec3::ONE / *direction;
		let t1  = (*min - *start) * inv;
		let t2  = (*max - *start) * inv;

		let tmin = t1.min(t2).max_element();
		let tmax = t1.max(t2).min_element();

		if tmax < 0.0 || tmin > tmax { return None; }

		Some(tmin)
	}

	pub fn raycast(&'_ self, transform: &Transform, max_length: Option<f32>) -> BVHRaycastIterator<'_, Index> {
		let start	 = transform.translation;
		let direction = transform.rotation * Vec3::Z;
		let mut heap  = BinaryHeap::new();
		let root	  = &self.nodes[0];
		if let Some(length) = Self::ray_aabb_intersection(&start, &direction, &(root.min_corner, root.max_corner)) {
			if max_length.is_none() || length <= max_length.unwrap() {
				heap.push(Candidate { length, entry: BVHEntry::Node(0) });
			}
		}
		BVHRaycastIterator { bvh: self, start, direction, max_length, heap }
	}

	pub fn internals(&self) -> (&Vec<BVHNode>, &Vec<(Index, (Vec3, Vec3))>) {
		(&self.nodes, &self.items)
	}
}

#[derive(PartialEq)]
enum BVHEntry<Index> {
	Node(u16),
	Hit(Index),
}

#[derive(PartialEq)]
struct Candidate<Index> {
	length: f32,
	entry:  BVHEntry<Index>,
}

impl<Index: PartialEq> Eq for Candidate<Index> {}
impl<Index: PartialEq> PartialOrd for Candidate<Index> {
	fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
}
impl<Index: PartialEq> Ord for Candidate<Index> {
	fn cmp(&self, other: &Self) -> Ordering {
		other.length.partial_cmp(&self.length).unwrap_or(Ordering::Equal)
	}
}

pub struct BVHRaycastIterator<'a, Index: Copy + Debug + PartialEq> {
	bvh:		&'a BVH<Index>,
	start:	  Vec3,
	direction:  Vec3,
	max_length: Option<f32>,
	heap:	   BinaryHeap<Candidate<Index>>,
}

impl<'a, Index: Copy + Debug + PartialEq> Iterator for BVHRaycastIterator<'a, Index> {
	type Item = (Index, f32);

	fn next(&mut self) -> Option<Self::Item> {
		loop {
			let Candidate { length, entry } = self.heap.pop()?;
			assert!(self.max_length.is_none() || length <= self.max_length.unwrap());
			match entry {
				BVHEntry::Hit(index) => return Some((index, length)),
				BVHEntry::Node(idx) => {
					let node = &self.bvh.nodes[idx as usize];
					match node.sub_nodes {
						BVHInternal::SubNodes { sub1, sub2 } => {
							for child in [sub1, sub2] {
								let cn = &self.bvh.nodes[child as usize];
								if let Some(length) = BVH::<Index>::ray_aabb_intersection(&self.start, &self.direction, &(cn.min_corner, cn.max_corner)) {
									if self.max_length.is_none() || length <= self.max_length.unwrap() {
										self.heap.push(Candidate { length, entry: BVHEntry::Node(child) });
									}
								}
							}
						}
						BVHInternal::Leaf { start, count } => {
							for item in &self.bvh.items[start as usize..(start + count) as usize] {
								if let Some(length) = BVH::<Index>::ray_aabb_intersection(&self.start, &self.direction, &item.1) {
									if self.max_length.is_none() || length <= self.max_length.unwrap() {
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
