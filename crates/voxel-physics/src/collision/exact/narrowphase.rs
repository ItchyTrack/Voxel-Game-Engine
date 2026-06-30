use bevy::math::{Quat, U16Vec3, U8Vec3, Vec3};

use bevy::transform::components::Transform;
use voxel_data::grid_tree::{get_child_contents_pos, CellKind, GridTreeView, NodeRef, SIZE, SIZE_CUBED, SIZE_USIZE_CUBED, U16Coord};
use voxel_data::voxel_grid_tree::PackedCell;
use voxel_data::transform_ext::TransformExt;
use voxel_data::voxels;

use crate::collision::CubeFeature;

/// ((min_a, max_a), (min_b, max_b), axis, index)
type SeparatingAxes = Vec<((f32, f32), (f32, f32), Vec3, u8)>;

type SubgridContact = (Vec3, CubeFeature, Vec3, CubeFeature, U16Vec3, U16Vec3);

fn get_bit(num: u8, bit: u8) -> u8 {
	((num & (1 << bit)) != 0) as u8
}

#[derive(Clone, Copy)]
struct DescendBox {
	origin: U16Vec3,
	size: u16,
	src: BoxSrc,
}

#[derive(Clone, Copy)]
enum BoxSrc {
	Node(NodeRef),
	Solid,
}

pub(super) fn get_collisions_between_subgrids(
	voxels_1: &voxels::Voxels,
	voxels_2: &voxels::Voxels,
	transform_of_1_in_2: &Transform,
) -> Vec<SubgridContact> {
	let mut collisions = vec![];
	if voxels_1.grid_tree().is_empty() || voxels_2.grid_tree().is_empty() { return collisions; }
	let separating_axes = compute_1x1x1_cube_separating_axes(transform_of_1_in_2.rotation);
	let view_1 = voxels_1.grid_tree().view();
	let view_2 = voxels_2.grid_tree().view();
	let root_1 = view_1.root();
	let root_2 = view_2.root();
	let box_1 = DescendBox { origin: root_1.origin.as_u16vec3(), size: voxel_data::grid_tree::size(root_1.depth) as u16, src: BoxSrc::Node(root_1) };
	let box_2 = DescendBox { origin: root_2.origin.as_u16vec3(), size: voxel_data::grid_tree::size(root_2.depth) as u16, src: BoxSrc::Node(root_2) };
	descend(&mut collisions, &separating_axes, transform_of_1_in_2, view_1, box_1, view_2, box_2);
	collisions
}

fn descend(
	collisions: &mut Vec<SubgridContact>,
	separating_axes: &SeparatingAxes,
	transform_of_1_in_2: &Transform,
	view_1: GridTreeView<'_, PackedCell, U16Coord>,
	box_1: DescendBox,
	view_2: GridTreeView<'_, PackedCell, U16Coord>,
	box_2: DescendBox,
) {
	let center_1 = *transform_of_1_in_2 * (box_1.origin.as_vec3() + Vec3::splat(box_1.size as f32 * 0.5));
	let center_2 = box_2.origin.as_vec3() + Vec3::splat(box_2.size as f32 * 0.5);
	if !boxes_overlap(separating_axes, center_1 - center_2, box_1.size as f32, box_2.size as f32) { return; }

	if box_1.size == 1 && box_2.size == 1 {
		collisions.extend(get_collision_1x1x1_voxel_pair(separating_axes, transform_of_1_in_2, box_1.origin, box_2.origin));
		return;
	}

	// Subdivide whichever box is larger so the two stay roughly the same scale as we descend.
	if box_1.size >= box_2.size {
		let mut children = [box_1; SIZE_USIZE_CUBED];
		let count = collect_children(&box_1, view_1, &mut children);
		for child in &children[..count] {
			descend(collisions, separating_axes, transform_of_1_in_2, view_1, *child, view_2, box_2);
		}
	} else {
		let mut children = [box_2; SIZE_USIZE_CUBED];
		let count = collect_children(&box_2, view_2, &mut children);
		for child in &children[..count] {
			descend(collisions, separating_axes, transform_of_1_in_2, view_1, box_1, view_2, *child);
		}
	}
}

fn collect_children(parent: &DescendBox, view: GridTreeView<'_, PackedCell, U16Coord>, out: &mut [DescendBox; SIZE_USIZE_CUBED]) -> usize {
	let mut count = 0;
	match parent.src {
		BoxSrc::Node(node) => {
			for child in view.occupied_children(node) {
				let origin = child.origin.as_u16vec3();
				let size = child.size as u16;
				match child.kind() {
					CellKind::Empty => {}
					CellKind::Data => { out[count] = DescendBox { origin, size, src: BoxSrc::Solid }; count += 1; }
					CellKind::Node => { out[count] = DescendBox { origin, size, src: BoxSrc::Node(view.child_node(child).expect("node cell has child")) }; count += 1; }
				}
			}
		}
		BoxSrc::Solid => {
			let child_size = parent.size / SIZE as u16;
			for i in 0..SIZE_CUBED {
				let origin = parent.origin + get_child_contents_pos(i).as_u16vec3() * child_size;
				out[count] = DescendBox { origin, size: child_size, src: BoxSrc::Solid };
				count += 1;
			}
		}
	}
	count
}

/// Separating-axis overlap test. rel_center is in box 2's frame
fn boxes_overlap(separating_axes: &SeparatingAxes, rel_center: Vec3, size_1: f32, size_2: f32) -> bool {
	for ((min_a, max_a), (min_b, max_b), axis, _) in separating_axes {
		let shift = rel_center.dot(*axis);
		let min_1 = size_1 * min_a + shift;
		let max_1 = size_1 * max_a + shift;
		let min_2 = size_2 * min_b;
		let max_2 = size_2 * max_b;
		if max_1 <= min_2 || min_1 >= max_2 { return false; }
	}
	true
}

fn get_collision_1x1x1_voxel_pair(
	separating_axes: &SeparatingAxes,
	transform_of_1_in_2: &Transform,
	pos_1: U16Vec3,
	pos_2: U16Vec3,
) -> Vec<SubgridContact> {
	let shift = Transform { translation: pos_2.as_vec3() + Vec3::splat(0.5), rotation: Quat::IDENTITY, scale: Vec3::ONE };
	let local = shift.inverse() * *transform_of_1_in_2 * Transform::from_translation(pos_1.as_vec3() + Vec3::splat(0.5));
	get_collision_1x1x1_voxel(&local, separating_axes)
		.into_iter()
		.map(|c| (shift * c.0, c.1, shift * c.2, c.3, pos_1, pos_2))
		.collect()
}

fn get_collision_1x1x1_voxel(
	transform: &Transform,
	separating_axes: &SeparatingAxes,
) -> Vec<(Vec3, CubeFeature, Vec3, CubeFeature)> {
	if transform.translation.length_squared() >= 3.0 { return vec![]; }
	let mut bests = vec![];
	let mut best_dis = 10.0;
	for ((unshifted_min_1, unshifted_max_1), (min_2, max_2), axis, index) in separating_axes {
		let shift = transform.translation.dot(*axis);
		let min_1 = unshifted_min_1 + shift;
		let max_1 = unshifted_max_1 + shift;
		if max_1 <= *min_2 || min_1 >= *max_2 { return vec![]; }
		if min_1 > *min_2 {
			let overlap = max_2 - min_1;
			if (overlap - best_dis).abs() < 0.001 {
				bests.push(((min_1, *max_2), *axis, *index));
			} else if overlap < best_dis {
				best_dis = overlap;
				bests.clear();
				bests.push(((min_1, *max_2), *axis, *index));
			}
		} else {
			let overlap = max_1 - *min_2;
			if (overlap - best_dis).abs() < 0.001 {
				bests.push(((max_1, *min_2), *axis, *index));
			} else if overlap < best_dis {
				best_dis = overlap;
				bests.clear();
				bests.push(((max_1, *min_2), *axis, *index));
			}
		}
	}

	let mut collisions: Vec<(Vec3, CubeFeature, Vec3, CubeFeature)> = vec![];

	for best in bests {
		let axis_neg = if best.0.1 < 0.0 { -1.0 } else { 1.0 };
		if best.2 < 3 {
			assert!((best.1 - Vec3::X).length() < 0.0001 || (best.1 - Vec3::Y).length() < 0.0001 || (best.1 - Vec3::Z).length() < 0.0001);
			let mut best_vertices = vec![];
			let mut best_dis = 10.0;
			(0..8).for_each(|i| {
				let v = *transform * (U8Vec3::new(get_bit(i, 0), get_bit(i, 1), get_bit(i, 2)).as_vec3() - 0.5);
				let dis = v.dot(best.1) * axis_neg;
				let surface_pos = v - v.project_onto(best.1);
				if
					surface_pos.x > 0.5 || surface_pos.x < -0.5 ||
					surface_pos.y > 0.5 || surface_pos.y < -0.5 ||
					surface_pos.z > 0.5 || surface_pos.z < -0.5
				{
					if (best_dis - dis).abs() >= 0.001 && best_dis > dis {
						best_vertices.clear();
						best_dis = dis;
					}
					return;
				}
				if (best_dis - dis).abs() < 0.001 {
					best_vertices.push((v, CubeFeature::Vertex { xyz: i }));
				} else if best_dis > dis {
					best_vertices.clear();
					best_dis = dis;
					best_vertices.push((v, CubeFeature::Vertex { xyz: i }));
				}
			});
			let face_vec = best.1.round().as_i8vec3() * axis_neg as i8;
			collisions.extend(best_vertices.into_iter().map(|v| (
				v.0,
				v.1,
				best.1 * best.0.1 + v.0 - v.0.project_onto(best.1),
				CubeFeature::Face { xyzs: face_vec.abs().as_u8vec3().dot(U8Vec3::new(1, 2, 4)) + 8 * (face_vec.element_sum().signum() == -1) as u8 },
			)));
		} else if best.2 < 6 {
			assert!((best.1 - transform.rotation * Vec3::X).length() < 0.0001 || (best.1 - transform.rotation * Vec3::Y).length() < 0.0001 || (best.1 - transform.rotation * Vec3::Z).length() < 0.0001);
			let mut best_vertices = vec![];
			let mut best_dis = 10.0;
			(0..8).for_each(|i| {
				let v = U8Vec3::new(get_bit(i, 0), get_bit(i, 1), get_bit(i, 2)).as_vec3() - 0.5;
				let dis = (v - transform.translation).dot(best.1) * -axis_neg;
				let surface_pos = transform.rotation.inverse() * ((v - transform.translation) - (v - transform.translation).project_onto(best.1));
				if
					surface_pos.x > 0.5 || surface_pos.x < -0.5 ||
					surface_pos.y > 0.5 || surface_pos.y < -0.5 ||
					surface_pos.z > 0.5 || surface_pos.z < -0.5
				{
					if (best_dis - dis).abs() >= 0.001 && best_dis > dis {
						best_vertices.clear();
						best_dis = dis;
					}
					return;
				}
				if (best_dis - dis).abs() < 0.001 {
					best_vertices.push((v, CubeFeature::Vertex { xyz: i }));
				} else if best_dis > dis {
					best_vertices.clear();
					best_dis = dis;
					best_vertices.push((v, CubeFeature::Vertex { xyz: i }));
				}
			});
			let face_vec = (transform.rotation.inverse() * best.1).round().as_i8vec3() * -axis_neg as i8;
			collisions.extend(best_vertices.into_iter().map(|v| (
				best.1 * best.0.0 + v.0 - v.0.project_onto(best.1),
				CubeFeature::Face { xyzs: face_vec.abs().as_u8vec3().dot(U8Vec3::new(1, 2, 4)) + 8 * (face_vec.element_sum().signum() == -1) as u8 },
				v.0,
				v.1,
			)));
		} else {
			let axes = [Vec3::X, Vec3::Y, Vec3::Z];
			let not_axes = [(Vec3::Y, Vec3::Z), (Vec3::X, Vec3::Z), (Vec3::X, Vec3::Y)];
			let not_axes_xyz_u8 = [(2, 4), (1, 4), (1, 2)];

			let index_1 = (best.2 - 6) % 3;
			let index_2 = (best.2 - 6) / 3;

			let axis1 = transform.rotation * axes[index_1 as usize]; // 1 axis
			let not_axes_1 = not_axes[index_1 as usize];
			let not_axes_xyz_u8_1 = not_axes_xyz_u8[index_1 as usize];

			let axis2 = axes[index_2 as usize]; // 2 axis
			let not_axes_2 = not_axes[index_2 as usize];
			let not_axes_xyz_u8_2 = not_axes_xyz_u8[index_2 as usize];

			(0..4).for_each(|i| {
				let edge_1 = *transform * ((if i & 1 == 0 { -not_axes_1.0 } else { not_axes_1.0 } + if i & 2 == 0 { -not_axes_1.1 } else { not_axes_1.1 }) * 0.5);
				(0..4).for_each(|j| {
					let edge_2 = (if j & 1 == 0 { -not_axes_2.0 } else { not_axes_2.0 } + if j & 2 == 0 { -not_axes_2.1 } else { not_axes_2.1 }) * 0.5;
					let result = points_with_direction(edge_1, axis1, edge_2, axis2, best.1 * axis_neg);
					if result.is_none() { return; }
					let (v1, v2) = result.unwrap();
					if (v2 - v1).normalize().dot(best.1) * axis_neg < 0.9 { return; }
					collisions.push((
						v1,
						CubeFeature::Edge { vertex_vertex: (1 << index_1 as u8) + (get_bit(i, 0) * not_axes_xyz_u8_1.0 + get_bit(i, 1) * not_axes_xyz_u8_1.1) * 9 },
						v2,
						CubeFeature::Edge { vertex_vertex: (1 << index_2 as u8) + (get_bit(j, 0) * not_axes_xyz_u8_2.0 + get_bit(j, 1) * not_axes_xyz_u8_2.1) * 9 },
					));
				});
			});
		}
	}
	collisions
}

fn points_with_direction(p1: Vec3, d1: Vec3, p2: Vec3, d2: Vec3, u: Vec3) -> Option<(Vec3, Vec3)> {
	let r = p2 - p1;
	let denom = d1.dot((-d2).cross(u));
	if denom.abs() < 1e-6 { return None; }

	let s = r.dot((-d2).cross(u)) / denom;
	let t = d1.dot(r.cross(u)) / denom;
	if s != s.clamp(-0.5, 0.5) || t != t.clamp(-0.5, 0.5) { return None; }

	Some((p1 + d1 * s, p2 + d2 * t))
}

// assumes other cube has no rotation and both are centered at (0,0,0)
pub(super) fn compute_1x1x1_cube_separating_axes(orientation: Quat) -> SeparatingAxes {
	let axes_6 = [
		Vec3::X,
		Vec3::Y,
		Vec3::Z,
		orientation * Vec3::X,
		orientation * Vec3::Y,
		orientation * Vec3::Z,
	];
	let axes_9 = (0..9).map(|i| axes_6[i / 3].cross(axes_6[3 + i % 3]));

	let corners = [
		Vec3::new(0.0, 0.0, 0.0) - Vec3::splat(0.5),
		Vec3::new(1.0, 0.0, 0.0) - Vec3::splat(0.5),
		Vec3::new(0.0, 1.0, 0.0) - Vec3::splat(0.5),
		Vec3::new(0.0, 0.0, 1.0) - Vec3::splat(0.5),
		Vec3::new(1.0, 1.0, 0.0) - Vec3::splat(0.5),
		Vec3::new(1.0, 0.0, 1.0) - Vec3::splat(0.5),
		Vec3::new(0.0, 1.0, 1.0) - Vec3::splat(0.5),
		Vec3::new(1.0, 1.0, 1.0) - Vec3::splat(0.5),
	];

	(axes_6.into_iter().chain(axes_9)).zip(0..15).filter_map(|(axis, index)| {
		if axis.length_squared() < 1e-6 { return None; }
		let norm_axis = axis.normalize();
		let mut min_a: f32 = 0.0; // cube 1
		let mut max_a: f32 = 0.0;
		for l in corners.map(|c| (orientation * c).dot(norm_axis)) {
			min_a = min_a.min(l);
			max_a = max_a.max(l);
		}
		let mut min_b: f32 = 0.0; // cube 2
		let mut max_b: f32 = 0.0;
		for l in corners.map(|c| c.dot(norm_axis)) {
			min_b = min_b.min(l);
			max_b = max_b.max(l);
		}
		assert!(min_b + max_b < 0.0001); // should be true
		assert!(min_a + max_a < 0.0001);

		Some(((min_a, max_a), (min_b, max_b), norm_axis, index))
	}).collect()
}
