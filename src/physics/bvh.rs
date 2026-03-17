use std::{cmp::Ordering, collections::BinaryHeap, fmt::Debug};

use itertools::partition;
use glam::{Vec3, Vec4};

use tracy_client::span;

use crate::{debug_draw, pose::Pose};

#[derive(Debug)]
enum BVHInternal {
	SubNodes {
		sub1: u32,
		sub2: u32,
	},
	Leaf {
		start: u32,
		count: u32,
	},
}

#[derive(Debug)]
struct BVHNode {
	min_corner: Vec3,
	max_corner: Vec3,
	sub_nodes: BVHInternal,
}

impl BVHNode {
	fn build_range<Index>(items: &mut [(Index, (Vec3, Vec3))], nodes: &mut Vec<BVHNode>, start: u32, end: u32) -> Self {
		assert!(end > start);
		let slice = &mut items[start as usize..end as usize];

		let mut center_min = slice[0].1.0 + slice[0].1.1;
		let mut center_max = slice[0].1.0 + slice[0].1.1;
		let mut min = slice[0].1.0;
		let mut max = slice[0].1.1;
		let mut avg = Vec3::ZERO;

		for v in slice.iter() {
			center_min = center_min.min(v.1.0 + v.1.1);
			center_max = center_max.max(v.1.0 + v.1.1);
			min = min.min(v.1.0);
			max = max.max(v.1.1);
			avg += v.1.0 + v.1.1;
		}

		let count = slice.len() as u32;

		if count <= 8 || min == max || center_min == center_max {
			return Self {
				min_corner: min,
				max_corner: max,
				sub_nodes: BVHInternal::Leaf { start, count },
			};
		}

		avg /= count as f32;

		let axis = (center_max - center_min).max_position();
		let split_value = avg[axis];

		let split_index = partition(slice.iter_mut(), |a| (a.1.0 + a.1.1)[axis] < split_value) as u32;
		assert!(split_index != 0 || split_index != end - start);
		// slice.select_nth_unstable_by(mid as usize, |a, b| (a.1.0 + a.1.1)[axis].partial_cmp(&(b.1.0 + b.1.1)[axis]).unwrap());
		nodes.push(BVHNode {
			min_corner: Vec3::ZERO,
			max_corner: Vec3::ZERO,
			sub_nodes: BVHInternal::Leaf { start: 0, count: 0 },
		});
		let sub1 = nodes.len() - 1;
		nodes[sub1] = Self::build_range(items, nodes, start, start + split_index);
		nodes.push(BVHNode {
			min_corner: Vec3::ZERO,
			max_corner: Vec3::ZERO,
			sub_nodes: BVHInternal::Leaf { start: 0, count: 0 },
		});
		let sub2 = nodes.len() - 1;
		nodes[sub2] = Self::build_range(items, nodes, start + split_index, end);
		Self {
			min_corner: min,
			max_corner: max,
			sub_nodes: BVHInternal::SubNodes {
				sub1: sub1 as u32,
				sub2: sub2 as u32,
			},
		}
	}
}

#[derive(Debug)]
pub struct BVH<Index: Copy + Debug + PartialEq> {
	nodes: Vec<BVHNode>,
	items: Vec<(Index, (Vec3, Vec3))>,
}

impl<Index: Copy + Debug + PartialEq> BVH<Index> {
	pub fn new(mut items: Vec<(Index, (Vec3, Vec3))>) -> Self {
		let _zone = span!("BVH creation");
		let items_len = items.len() as u32;
		let mut nodes: Vec<BVHNode> = vec![];
		nodes.push(BVHNode {
			min_corner: Vec3::ZERO,
			max_corner: Vec3::ZERO,
			sub_nodes: BVHInternal::Leaf { start: 0, count: 0 },
		});
		nodes[0] = BVHNode::build_range(&mut items, &mut nodes, 0, items_len);
		BVH {
			nodes,
			items,
		}
	}
	fn intersects(a: &(Vec3, Vec3), b: &(Vec3, Vec3)) -> bool {
		a.0.cmple(b.1).all() && a.1.cmpge(b.0).all()
	}
	pub fn get_collisions(&self, bounds: &(Vec3, Vec3)) -> Vec<Index> {
		let mut out: Vec<Index> = vec![];
		let mut stack = vec![0];

		while let Some(idx) = stack.pop() {
			let node = &self.nodes[idx as usize];

			if !Self::intersects(bounds, &(node.min_corner, node.max_corner)) { continue; }

			match node.sub_nodes {
				BVHInternal::SubNodes { sub1, sub2 } => {
					stack.push(sub1);
					stack.push(sub2);
				}
				BVHInternal::Leaf { start, count } => {
					out.extend(self.items[start as usize..(start + count) as usize].iter().filter_map(|item|
						if Self::intersects(bounds, &item.1) { Some(item.0) } else { None }
					));
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
		let t1 = (*min - *start) * inv;
		let t2 = (*max - *start) * inv;

		let tmin = t1.min(t2).max_element();
		let tmax = t1.max(t2).min_element();

		if tmax < 0.0 || tmin > tmax { return None; }

		Some(tmin)
	}

	pub fn raycast(&'_ self, pose: &Pose, max_length: Option<f32>) -> BVHRaycastIterator<'_, Index> {
		let start = pose.translation;
		let direction = pose.rotation * Vec3::Z;
		let mut heap = BinaryHeap::new();
		let root = &self.nodes[0];
		if let Some(length) = Self::ray_aabb_intersection(&start, &direction, &(root.min_corner, root.max_corner)) {
			if max_length.is_none() || length <= max_length.unwrap() {
				heap.push(Candidate { length, entry: BVHEntry::Node(0) });
			}
		}
		BVHRaycastIterator { bvh: self, start, direction, max_length: max_length, heap }
	}

	pub fn render_debug(&self) {
		let mut stack = vec![0];

		while let Some(idx) = stack.pop() {
			let node = &self.nodes[idx as usize];
			match node.sub_nodes {
				BVHInternal::SubNodes { sub1, sub2 } => {
					debug_draw::aabb(node.min_corner, node.max_corner, &Vec4::ONE);
					stack.push(sub1);
					stack.push(sub2);
				}
				BVHInternal::Leaf { start, count } => {
					for item in self.items[start as usize..(start + count) as usize].iter() {
						debug_draw::aabb(item.1.0, item.1.1, &Vec4::ONE);
					}
				}
			}
		}
	}
}

#[derive(PartialEq)]
enum BVHEntry<Index> {
	Node(u32),
	Hit(Index),
}

#[derive(PartialEq)]
struct Candidate<Index> {
	length: f32,
	entry: BVHEntry<Index>,
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
	bvh: &'a BVH<Index>,
	start: Vec3,
	direction: Vec3,
	max_length: Option<f32>,
	heap: BinaryHeap<Candidate<Index>>,
}

impl<'a, Index: Copy + Debug + PartialEq + PartialEq> Iterator for BVHRaycastIterator<'a, Index> {
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
