use glam::{I16Vec3, Quat, U8Vec3, Vec3};

use bevy::transform::components::Transform;
use voxel_data::transform_ext::TransformExt;
use voxel_data::voxels;

use super::CubeFeature;

fn get_bit(num: u8, bit: u8) -> u8 {
	((num & (1 << bit)) != 0) as u8
}

pub(super) fn get_collision(
	transform: &Transform,
	voxels: &voxels::Voxels,
	separating_axes: &Vec<((f32, f32), (f32, f32), Vec3, u8)>,
	to_global: &Transform,
) -> Vec<(Vec3, CubeFeature, Vec3, CubeFeature, I16Vec3)> {
	let mut collisions = vec![];
	for x in -1..2 {
		for y in -1..2 {
			for z in -1..2 {
				let vec = I16Vec3::new(x, y, z);
				if voxels.get_voxel(&(transform.translation.floor().as_i16vec3() + vec)).is_some() {
					let shift = Transform { translation: transform.translation.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5), rotation: Quat::IDENTITY, scale: Vec3::ONE };
					get_collision_1x1x1_voxel(
						&(shift.inverse() * *transform),
						separating_axes,
						&(shift * *to_global),
					).into_iter().for_each(|c| {
						collisions.push((
							shift * c.0,
							c.1,
							shift * c.2,
							c.3,
							transform.translation.floor().as_i16vec3() + vec,
						))
					});
				}
			}
		}
	}
	collisions
}

fn get_collision_1x1x1_voxel(
	transform: &Transform,
	separating_axes: &Vec<((f32, f32), (f32, f32), Vec3, u8)>,
	_to_global: &Transform,
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
pub(super) fn compute_1x1x1_cube_separating_axes(orientation: Quat) -> Vec<((f32, f32), (f32, f32), Vec3, u8)> {
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
