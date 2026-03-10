use std::vec;

use glam::{IVec3, Quat, U8Vec3, Vec3, Vec4};

use crate::{debug_draw, entity, voxels, physics::bvh};

#[derive(Copy, Clone, Hash, PartialEq, Eq)]
pub enum CubeFeature {
	Vertex { xyz: u8 },
	Edge{ vertex_vertex: u8 },
	Face{ xyzs: u8 },
}

#[derive(Copy, Clone)]
pub struct Collision {
	pub id1: u32,
	pub id2: u32,
	pub voxel_pos1: IVec3,
	pub voxel_pos2: IVec3,
	pub feature1: CubeFeature,
	pub feature2: CubeFeature,
	pub collision1: Vec3,
	pub collision2: Vec3,
	pub local_collision1: Vec3,
	pub local_collision2: Vec3,
}

fn get_bit(num: u8, bit: u8) -> u8 {
	((num & (1 << bit)) != 0) as u8
}

pub fn get_collisions(entities: &Vec<entity::Entity>, pose_to_eval_at: &Vec<(Vec3, Quat)>) -> Vec<Collision> {
	let mut bounds = vec![];
	for index in 0..entities.len() {
		let local_bounding_box_a = entities[index].get_voxels().get_bounding_box();
		if local_bounding_box_a.is_none() { continue; }
		let (aabb_a_min, aabb_a_max) = calc_aabb(
			&(pose_to_eval_at[index].0 + pose_to_eval_at[index].1 * entities[index].get_voxels_local_pos()),
			&pose_to_eval_at[index].1,
			&(local_bounding_box_a.unwrap().0.as_vec3(), local_bounding_box_a.unwrap().1.as_vec3())
		);
		bounds.push((index as u32, (aabb_a_min, aabb_a_max)));
	}
	let bvh = bvh::BVH::new(bounds.clone());
	let mut collisions: Vec<Collision> = vec![];
	for id_a in 0..entities.len() {
		let entity_a = &entities[id_a];
		for id_b in bvh.get_collisions(&bounds[id_a].1) {
			if (id_a as u32) <= id_b { continue; }
			let entity_b = &entities[id_b as usize];

			let no_swap = entity_a.get_voxels().get_voxels().len() < entity_b.get_voxels().get_voxels().len();
			let (entity1, pose1, entity2, pose2) = {
				if no_swap { (entity_a, pose_to_eval_at[id_a], entity_b, pose_to_eval_at[id_b as usize]) }
				else { (entity_b, pose_to_eval_at[id_b as usize], entity_a, pose_to_eval_at[id_a]) }
			};
			let pos_of_1_in_2 = pose2.1.inverse() * (pose1.1 * entity1.get_voxels_local_pos() + pose1.0 - pose2.0) - entity2.get_voxels_local_pos();
			let orientation_of_1_in_2 = pose2.1.inverse() * pose1.1;
			let separating_axis = compute_1x1x1_cube_separating_axes(orientation_of_1_in_2);
			for voxel in entity1.get_voxels().get_voxels().iter() {
				collisions.extend(get_collision(
					&(&orientation_of_1_in_2 * (voxel.0.as_vec3() + Vec3::new(0.5, 0.5, 0.5)) + pos_of_1_in_2),
					&orientation_of_1_in_2,
					entity2.get_voxels(),
					&separating_axis,
					&(pose2.0 + pose2.1 * entity2.get_voxels_local_pos()),
					&pose2.1
				).iter().filter_map(|c| {
					// let mut first_edge_covered = false;
					// match c.1 {
					// 	CubeFeature::Vertex { xyz } => {
					// 		for i in 1..8 {
					// 			if entity1.get_voxel(voxel.0 + IVec3::new(
					// 					get_bit(i, 0) as i32 * (get_bit(xyz, 0) as i32 * 2 - 1),
					// 					get_bit(i, 1) as i32 * (get_bit(xyz, 1) as i32 * 2 - 1),
					// 					get_bit(i, 2) as i32 * (get_bit(xyz, 2) as i32 * 2 - 1)
					// 				)).is_some() { return None; }
					// 		}
					// 	},
					// 	CubeFeature::Edge { vertex_vertex } => {
					// 		if get_bit(vertex_vertex, 0) ^ get_bit(vertex_vertex, 0 + 3) == 1 {
					// 			if
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					0,
					// 					get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 					0
					// 				)).is_some() ||
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					0,
					// 					0,
					// 					get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 				)).is_some() ||
					// 				entity2.get_voxel(voxel.0 + IVec3::new(
					// 					0,
					// 					get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 					get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 				)).is_some()
					// 			{ first_edge_covered = true; }
					// 		} else if get_bit(vertex_vertex, 1) ^ get_bit(vertex_vertex, 1 + 3) == 1 {
					// 			if
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 					0,
					// 					0
					// 				)).is_some() ||
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					0,
					// 					0,
					// 					get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 				)).is_some() ||
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 					0,
					// 					get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 				)).is_some()
					// 			{ first_edge_covered = true; }
					// 		} else if get_bit(vertex_vertex, 2) ^ get_bit(vertex_vertex, 2 + 3) == 1 {
					// 			if
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 					0,
					// 					0
					// 				)).is_some() ||
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					0,
					// 					get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 					0
					// 				)).is_some() ||
					// 				entity1.get_voxel(voxel.0 + IVec3::new(
					// 					get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 					get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 					0
					// 				)).is_some()
					// 			{ first_edge_covered = true; }
					// 		}
					// 	},
					// 	CubeFeature::Face { xyzs } => {
					// 		if entity1.get_voxel(U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_ivec3() * (1 - 2 * get_bit(xyzs, 3) as i32)).is_some() {
					// 			return None;
					// 		}
					// 	},
					// };
					// match c.3 {
					// 	CubeFeature::Vertex { xyz } => {
					// 		for i in 1..8 {
					// 			if entity2.get_voxel(c.4 + IVec3::new(
					// 					get_bit(i, 0) as i32 * (get_bit(xyz, 0) as i32 * 2 - 1),
					// 					get_bit(i, 1) as i32 * (get_bit(xyz, 1) as i32 * 2 - 1),
					// 					get_bit(i, 2) as i32 * (get_bit(xyz, 2) as i32 * 2 - 1)
					// 				)).is_some() { return None; }
					// 		}
					// 	},
					// 	CubeFeature::Edge { vertex_vertex } => {
					// 		if first_edge_covered { // only if both edges are covered
					// 			if get_bit(vertex_vertex, 0) ^ get_bit(vertex_vertex, 0 + 3) == 1 {
					// 				if
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						0,
					// 						get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 						0
					// 					)).is_some() ||
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						0,
					// 						0,
					// 						get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 					)).is_some() ||
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						0,
					// 						get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 						get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 					)).is_some()
					// 				{ return None; }
					// 			} else if get_bit(vertex_vertex, 1) ^ get_bit(vertex_vertex, 1 + 3) == 1 {
					// 				if
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 						0,
					// 						0
					// 					)).is_some() ||
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						0,
					// 						0,
					// 						get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 					)).is_some() ||
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 						0,
					// 						get_bit(vertex_vertex, 2) as i32 * 2 - 1
					// 					)).is_some()
					// 				{ return None; }
					// 			} else if get_bit(vertex_vertex, 2) ^ get_bit(vertex_vertex, 2 + 3) == 1 {
					// 				if
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 						0,
					// 						0
					// 					)).is_some() ||
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						0,
					// 						get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 						0
					// 					)).is_some() ||
					// 					entity2.get_voxel(c.4 + IVec3::new(
					// 						get_bit(vertex_vertex, 0) as i32 * 2 - 1,
					// 						get_bit(vertex_vertex, 1) as i32 * 2 - 1,
					// 						0
					// 					)).is_some()
					// 				{ return None; }
					// 			}
					// 		}
					// 	},
					// 	CubeFeature::Face { xyzs } => {
					// 		if entity2.get_voxel(U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_ivec3() * (1 - 2 * get_bit(xyzs, 3) as i32)).is_some() {
					// 			return None;
					// 		}
					// 	},
					// };


					Some(Collision {
						id1: if no_swap { id_a as u32 } else { id_b as u32 },
						id2: if no_swap { id_b as u32 } else { id_a as u32 },
						voxel_pos1: voxel.0,
						voxel_pos2:	c.4,
						feature1: c.1,
						feature2: c.3,
						collision1: pose2.1 * (c.0 + entity2.get_voxels_local_pos()) + pose2.0,
						collision2: pose2.1 * (c.2 + entity2.get_voxels_local_pos()) + pose2.0,
						local_collision1: orientation_of_1_in_2.inverse() * (c.0 - pos_of_1_in_2) + entity1.get_voxels_local_pos(),
						local_collision2: c.2 + entity2.get_voxels_local_pos(),
					})
				}));
			}
		}
	}
	for collision in collisions.iter() {
		let e1 = &entities[collision.id1 as usize];
		let e2 = &entities[collision.id2 as usize];

		debug_draw::line(collision.collision1, collision.collision2, Vec4::new(1.0, 0.0, 0.0, 1.0));
		debug_draw::point(collision.collision1, Vec4::new(1.0, 1.0, 1.0, 1.0), 0.1);
		debug_draw::point(collision.collision2, Vec4::new(1.0, 1.0, 1.0, 1.0), 0.1);

		match collision.feature1 {
			CubeFeature::Vertex { xyz } => {
				let v = U8Vec3::new(get_bit(xyz, 0), get_bit(xyz, 1), get_bit(xyz, 2)).as_vec3() - 0.5;
				debug_draw::aabb(
					e1.position + e1.orientation * (v + collision.voxel_pos1.as_vec3() + e1.get_voxels_local_pos() + Vec3::splat(0.5)) + Vec3::splat(0.01),
					e1.position + e1.orientation * (v + collision.voxel_pos1.as_vec3() + e1.get_voxels_local_pos() + Vec3::splat(0.5)) + Vec3::splat(0.01),
					Vec4::ONE
				);
			},
			CubeFeature::Edge { vertex_vertex } => {
				let v1 = U8Vec3::new(get_bit(vertex_vertex, 0), get_bit(vertex_vertex, 1), get_bit(vertex_vertex, 2)).as_vec3() - 0.5;
				let v2 = U8Vec3::new(get_bit(vertex_vertex, 3), get_bit(vertex_vertex, 4), get_bit(vertex_vertex, 5)).as_vec3() - 0.5;
				debug_draw::line(
					e1.position + e1.orientation * (v1 + collision.voxel_pos1.as_vec3() + e1.get_voxels_local_pos() + Vec3::splat(0.5)),
					e1.position + e1.orientation * (v2 + collision.voxel_pos1.as_vec3() + e1.get_voxels_local_pos() + Vec3::splat(0.5)),
					Vec4::ONE
				);
			},
			CubeFeature::Face { xyzs } => {
				let v = U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_vec3() * (0.5 - get_bit(xyzs, 3) as f32);
				debug_draw::aabb(
					e1.position + e1.orientation * (v + collision.voxel_pos1.as_vec3() + e1.get_voxels_local_pos() + Vec3::splat(0.5)) - Vec3::splat(0.01),
					e1.position + e1.orientation * (v + collision.voxel_pos1.as_vec3() + e1.get_voxels_local_pos() + Vec3::splat(0.5)) + Vec3::splat(0.01),
					Vec4::W
				);
			},
		}
		match collision.feature2 {
			CubeFeature::Vertex { xyz } => {
				let v = U8Vec3::new(get_bit(xyz, 0), get_bit(xyz, 1), get_bit(xyz, 2)).as_vec3() - 0.5;
				debug_draw::aabb(
					e2.position + e2.orientation * (v + collision.voxel_pos2.as_vec3() + e2.get_voxels_local_pos() + Vec3::splat(0.5)) - Vec3::splat(0.01),
					e2.position + e2.orientation * (v + collision.voxel_pos2.as_vec3() + e2.get_voxels_local_pos() + Vec3::splat(0.5)) + Vec3::splat(0.01),
					Vec4::ONE
				);
			},
			CubeFeature::Edge { vertex_vertex } => {
				let v1 = U8Vec3::new(get_bit(vertex_vertex, 0), get_bit(vertex_vertex, 1), get_bit(vertex_vertex, 2)).as_vec3() - 0.5;
				let v2 = U8Vec3::new(get_bit(vertex_vertex, 3), get_bit(vertex_vertex, 4), get_bit(vertex_vertex, 5)).as_vec3() - 0.5;
				debug_draw::line(
					e2.position + e2.orientation * (v1 + collision.voxel_pos2.as_vec3() + e2.get_voxels_local_pos() + Vec3::splat(0.5)),
					e2.position + e2.orientation * (v2 + collision.voxel_pos2.as_vec3() + e2.get_voxels_local_pos() + Vec3::splat(0.5)),
					Vec4::ONE
				);
			},
			CubeFeature::Face { xyzs } => {
				let v = U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_vec3() * (0.5 - get_bit(xyzs, 3) as f32);
				debug_draw::aabb(
					e2.position + e2.orientation * (v + collision.voxel_pos2.as_vec3() + e2.get_voxels_local_pos() + Vec3::splat(0.5)) - Vec3::splat(0.01),
					e2.position + e2.orientation * (v + collision.voxel_pos2.as_vec3() + e2.get_voxels_local_pos() + Vec3::splat(0.5)) + Vec3::splat(0.01),
					Vec4::W
				);
			},
		}
	}
	collisions
}

fn get_collision(pos: &Vec3, orientation: &Quat, voxels: &voxels::Voxels, separating_axes: &Vec<((f32, f32), (f32, f32), Vec3, u8)>, p: &Vec3, q:&Quat) -> Vec<(Vec3, CubeFeature, Vec3, CubeFeature, IVec3)> {
	let mut collisions = vec![];
	for x in -1..2 {
		for y in -1..2 {
			for z in -1..2 {
				let vec = IVec3::new(x, y, z);
				if voxels.get_voxel(pos.floor().as_ivec3() + vec).is_some() {
					get_collision_1x1x1_voxel(&(pos - pos.floor() - vec.as_vec3() - Vec3::new(0.5, 0.5, 0.5)), orientation, separating_axes, p + q * (pos.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5)), *q).into_iter().for_each(|c| {
						collisions.push((
							c.0 + pos.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5),
							c.1,
							c.2 + pos.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5),
							c.3,
							pos.floor().as_ivec3() + vec
						))
					});
				}
			}
		}
	}
	return collisions;
}

fn get_collision_1x1x1_voxel(pos: &Vec3, orientation: &Quat, separating_axes: &Vec<((f32, f32), (f32, f32), Vec3, u8)>, _p: Vec3, _q:Quat) -> Vec<(Vec3, CubeFeature, Vec3, CubeFeature)> {
	if pos.length_squared() >= 3.0 { return vec![]; }
	let mut bests = vec![];
	let mut best_dis = 10.0;
	for ((unshifted_min_1, unshifted_max_1), (min_2, max_2), axis, index) in separating_axes {
		let shift = pos.dot(*axis);
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
			let mut best_verties = vec![];
			let mut best_dis = 10.0;
			(0..8).for_each(|i| {
				let v = pos + orientation * (U8Vec3::new(get_bit(i, 0), get_bit(i, 1), get_bit(i, 2)).as_vec3() - 0.5);
				let dis = v.dot(best.1) * axis_neg;
				let surface_pos = v - v.project_onto(best.1);
				if
					surface_pos.x > 0.5 || surface_pos.x < -0.5 ||
					surface_pos.y > 0.5 || surface_pos.y < -0.5 ||
					surface_pos.z > 0.5 || surface_pos.z < -0.5
				{
					if (best_dis - dis).abs() >= 0.001 && best_dis > dis {
						best_verties.clear();
						best_dis = dis;
					}
					return;
				}
				if (best_dis - dis).abs() < 0.001 {
					best_verties.push((v, CubeFeature::Vertex { xyz: i }));
				} else if best_dis > dis {
					best_verties.clear();
					best_dis = dis;
					best_verties.push((v, CubeFeature::Vertex { xyz: i }));
				}
			});
			// best_verties.iter().for_each(|v| debug_draw::point(p + q * v.0, Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			// best_verties.iter().for_each(|v| debug_draw::point(p + q * (best.1 * best.0.0 + v.0 - v.0.project_onto(best.1)), Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			let face_vec = best.1.round().as_i8vec3() * axis_neg as i8;
			collisions.extend(best_verties.into_iter().map(|v| {(
				v.0,
				v.1,
				best.1 * best.0.1 + v.0 - v.0.project_onto(best.1),
				CubeFeature::Face { xyzs: face_vec.abs().as_u8vec3().dot(U8Vec3::new(1, 2, 4)) + 8 * (face_vec.element_sum().signum() == -1) as u8 },
			)}));
		} else if best.2 < 6 {
			assert!((best.1 - orientation * Vec3::X).length() < 0.0001 || (best.1 - orientation * Vec3::Y).length() < 0.0001 || (best.1 - orientation * Vec3::Z).length() < 0.0001);
			let mut best_verties = vec![];
			let mut best_dis = 10.0;
			(0..8).for_each(|i| {
				let v = U8Vec3::new(get_bit(i, 0), get_bit(i, 1), get_bit(i, 2)).as_vec3() - 0.5;
				let dis = (v - pos).dot(best.1) * -axis_neg;
				let surface_pos = orientation.inverse() * ((v - pos) - (v - pos).project_onto(best.1));
				if
					surface_pos.x > 0.5 || surface_pos.x < -0.5 ||
					surface_pos.y > 0.5 || surface_pos.y < -0.5 ||
					surface_pos.z > 0.5 || surface_pos.z < -0.5
				{
					if (best_dis - dis).abs() >= 0.001 && best_dis > dis {
						best_verties.clear();
						best_dis = dis;
					}
					return;
				}
				if (best_dis - dis).abs() < 0.001 {
					best_verties.push((v, CubeFeature::Vertex { xyz: i }));
				} else if best_dis > dis {
					best_verties.clear();
					best_dis = dis;
					best_verties.push((v, CubeFeature::Vertex { xyz: i }));
				}
			});
			// best_verties.iter().for_each(|v| debug_draw::point(p + q * v.0, Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			// best_verties.iter().for_each(|v| debug_draw::point(p + q * (best.1 * best.0.0 + v.0 - v.0.project_onto(best.1)), Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			let face_vec = (orientation.inverse() * best.1).round().as_i8vec3() * -axis_neg as i8;
			collisions.extend(best_verties.into_iter().map(|v| {(
				best.1 * best.0.0 + v.0 - v.0.project_onto(best.1),
				CubeFeature::Face { xyzs: face_vec.abs().as_u8vec3().dot(U8Vec3::new(1, 2, 4)) + 8 * (face_vec.element_sum().signum() == -1) as u8 },
				v.0,
				v.1,
			)}));
		} else {
			let axes = [Vec3::X, Vec3::Y, Vec3::Z];
			let not_axes = [(Vec3::Y, Vec3::Z), (Vec3::X, Vec3::Z), (Vec3::X, Vec3::Y)];
			let not_axes_xyz_u8 = [(2, 4), (1, 4), (1, 2)];

			let index_1 = (best.2 - 6) % 3;
			let index_2 = (best.2 - 6) / 3;

			let axis1 = orientation * axes[index_1 as usize]; // 1 axis
			let not_axes_1 = not_axes[index_1 as usize];
			let not_axes_xyz_u8_1 = not_axes_xyz_u8[index_1 as usize];

			let axis2 = axes[index_2 as usize]; // 2 axis
			let not_axes_2 = not_axes[index_2 as usize];
			let not_axes_xyz_u8_2 = not_axes_xyz_u8[index_2 as usize];

			(0..4).for_each(|i| {
				let edge_1 = pos + orientation * (if i & 1 == 0 { -not_axes_1.0 } else { not_axes_1.0 } + if i & 2 == 0 { -not_axes_1.1 } else { not_axes_1.1 }) * 0.5;
				(0..4).for_each(|j| {
					let edge_2 = (if j & 1 == 0 { -not_axes_2.0 } else { not_axes_2.0 } + if j & 2 == 0 { -not_axes_2.1 } else { not_axes_2.1 }) * 0.5;
					let result = points_with_direction(edge_1, axis1, edge_2, axis2, best.1 * axis_neg);
					if result.is_none() { return; }
					let (v1, v2) = result.unwrap();
					if (v2 - v1).normalize().dot(best.1) * axis_neg < 0.9 { return; }
					// debug_draw::point(p + q * v1, Vec4::W, 0.2);
					// debug_draw::point(p + q * v2, Vec4::W, 0.2);
					collisions.push((
						v1,
						CubeFeature::Edge { vertex_vertex: (1 << index_1 as u8) + (get_bit(i, 0) * not_axes_xyz_u8_1.0 + get_bit(i, 1) * not_axes_xyz_u8_1.1) * 9 },
						v2,
						CubeFeature::Edge { vertex_vertex: (1 << index_2 as u8) + (get_bit(j, 0) * not_axes_xyz_u8_2.0 + get_bit(j, 1) * not_axes_xyz_u8_2.1) * 9 }
					));
				});
			});
		}
	}
	return collisions;
}

pub fn points_with_direction(
    p1: Vec3,
    d1: Vec3,
    p2: Vec3,
    d2: Vec3,
    u: Vec3,
) -> Option<(Vec3, Vec3)> {
    let r = p2 - p1;

    let denom = d1.dot((-d2).cross(u));

    if denom.abs() < 1e-6 { return None; }

    let s = r.dot((-d2).cross(u)) / denom;
    let t = d1.dot(r.cross(u)) / denom;

	if s != s.clamp(-0.5, 0.5) || t != t.clamp(-0.5, 0.5) { return None; }

    let x1 = p1 + d1 * s;
    let x2 = p2 + d2 * t;

    Some((x1, x2))
}

// assumes other cube has no rotation and both are centered at (0,0,0)
fn compute_1x1x1_cube_separating_axes(orientation: Quat) -> Vec<((f32, f32), (f32, f32), Vec3, u8)> {
	let axes_6 = [
		Vec3::X,
		Vec3::Y,
		Vec3::Z,
		orientation * Vec3::X,
		orientation * Vec3::Y,
		orientation * Vec3::Z,
	];
	let axes_9 = (0..9).map(|i| axes_6[i/3].cross(axes_6[3+i%3]));

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

fn calc_aabb(pos: &Vec3, orientation: &Quat, local_bounding_box: &(Vec3, Vec3)) -> (Vec3, Vec3) {
	// eventually, this will aggregate the bounding boxes of multiple voxel grids
	let (min, max) = local_bounding_box;
	let min = min;
	let max = max + Vec3::new(1.0, 1.0, 1.0);
	// rotate the 8 corners of the bounding box and find the new bounding box that contains them
	let corners = [
		*min,
		Vec3::new(max.x, min.y, min.z),
		Vec3::new(min.x, max.y, min.z),
		Vec3::new(min.x, min.y, max.z),
		Vec3::new(max.x, max.y, min.z),
		Vec3::new(max.x, min.y, max.z),
		Vec3::new(min.x, max.y, max.z),
		max,
	];
	let rotated_corners = corners.map(|c| orientation * c);
	let min = rotated_corners.iter().fold(Vec3::new(f32::MAX, f32::MAX, f32::MAX), |acc, c| acc.min(*c));
	let max = rotated_corners.iter().fold(Vec3::new(f32::MIN, f32::MIN, f32::MIN), |acc, c| acc.max(*c));
	(min + pos, max + pos)
}
