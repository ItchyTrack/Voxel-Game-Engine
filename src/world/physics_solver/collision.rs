use std::vec;

use glam::{I16Vec3, IVec3, Quat, U8Vec3, Vec3};
use super::super::sparse_set::SparseSet;
use super::super::{pose::Pose, voxels};
use super::{bvh::BVH};
use super::super::physics_body::{PhysicsBody, PhysicsBodyId};
use super::super::grid::{Grid, GridId, SubGridId};

use tracy_client::span;

#[derive(Copy, Clone, Hash, PartialEq, Eq)]
pub enum CubeFeature {
	Vertex { xyz: u8 },
	Edge{ vertex_vertex: u8 },
	Face{ xyzs: u8 },
}

#[derive(Copy, Clone)]
pub struct HalfCollision {
	pub body_id: PhysicsBodyId,
	pub grid_id: GridId,
	pub voxel_pos: IVec3,
	pub feature: CubeFeature,
	pub collision: Vec3,
	pub local_collision: Vec3,
}

#[derive(Copy, Clone)]
pub struct Collision {
	pub part1: HalfCollision,
	pub part2: HalfCollision,
}

impl Collision {
	pub fn get_swapped(&self) -> Collision {
		Collision {
			part1: self.part2,
			part2: self.part1,
		}
	}
}

fn get_bit(num: u8, bit: u8) -> u8 {
	((num & (1 << bit)) != 0) as u8
}

static mut CHECK_COUNTER: u32 = 0;

pub fn get_collisions(
	physics_bodies: &SparseSet<PhysicsBodyId, PhysicsBody>,
	grids: &SparseSet<GridId, Grid>,
	bvh: &BVH<(PhysicsBodyId, GridId, SubGridId)>
) -> Vec<Collision> {
	unsafe { CHECK_COUNTER = 0; }
	let _zone = span!("Do Collisions");
	// bvh.render_debug();
	let mut collisions: Vec<Collision> = vec![];
	for (physics_body_id_a, physics_body_a) in physics_bodies {
		if physics_body_a.is_static { continue; }
		for grid_id_a in physics_body_a.grids() {
			let grid_a = grids.get(grid_id_a).unwrap();
			let grid_pose_a = physics_body_a.pose * grid_a.pose();
			for (_sub_grid_id_a, sub_grid_a) in grid_a.sub_grids() {
				let sub_grid_pose_a = grid_pose_a * Pose::from_translation(sub_grid_a.sub_grid_pos().as_vec3());
				let bound = {
					let local_aabb = sub_grid_a.local_aabb(&sub_grid_pose_a.rotation);
					let local_aabb = if let Some(local_aabb) = local_aabb { local_aabb } else { continue; };
					(
						sub_grid_pose_a.translation + local_aabb.0,
						sub_grid_pose_a.translation + local_aabb.1
					)
				};
				for (physics_body_id_b, grid_id_b, sub_grid_id_b) in bvh.get_collisions(&bound) {
					if physics_body_id_b == *physics_body_id_a { continue; }
					let physics_body_b = physics_bodies.get(&physics_body_id_b).unwrap();
					if !physics_body_b.is_static && physics_body_id_a.0 < physics_body_id_b.0 { continue; }
					let grid_b = grids.get(&grid_id_b).unwrap();
					let sub_grid_b = grid_b.sub_grids().get(&sub_grid_id_b).unwrap();
					let no_swap = sub_grid_a.get_voxels().get_voxels().len() < sub_grid_b.get_voxels().get_voxels().len();
					let (physics_body1, grid_1, sub_grid_1, physics_body2, grid_2, sub_grid_2) = {
						if no_swap { (physics_body_a, grid_a, sub_grid_a, physics_body_b, grid_b, sub_grid_b) }
						else { (physics_body_b, grid_b, sub_grid_b, physics_body_a, grid_a, sub_grid_a) }
					};
					let pose_of_1_in_2 = {
						Pose::from_translation(-sub_grid_2.sub_grid_pos().as_vec3()) * grid_2.pose().inverse() * physics_body2.pose.inverse() *
						physics_body1.pose * grid_1.pose() * Pose::from_translation(sub_grid_1.sub_grid_pos().as_vec3())
					};
					let separating_axis = compute_1x1x1_cube_separating_axes(pose_of_1_in_2.rotation);
					for voxel in sub_grid_1.get_voxels().get_voxels().iter() {
						for x in 0..voxel.1 {
							for y in 0..voxel.1 {
								for z in 0..voxel.1 {
									collisions.extend(get_collision(
										&(&pose_of_1_in_2 * Pose::new((voxel.0 + I16Vec3::new(x as i16,  y as i16,  z as i16)).as_vec3() + Vec3::new(0.5, 0.5, 0.5), Quat::IDENTITY)),
										sub_grid_2.get_voxels(),
										&separating_axis,
										&(physics_body2.pose * grid_2.pose()),
									).iter().filter_map(|c| {
										let mut first_edge_covered = false;
										let collision_grid_pos_1 = voxel.0.as_ivec3() + sub_grid_1.sub_grid_pos();
										let collision_grid_pos_2 = c.4.as_ivec3() + sub_grid_2.sub_grid_pos();
										match c.1 {
											CubeFeature::Vertex { xyz } => {
												for i in 1..8 {
													if grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															get_bit(i, 0) as i32 * (get_bit(xyz, 0) as i32 * 2 - 1),
															get_bit(i, 1) as i32 * (get_bit(xyz, 1) as i32 * 2 - 1),
															get_bit(i, 2) as i32 * (get_bit(xyz, 2) as i32 * 2 - 1)
														))).is_some() { return None; }
												}
											},
											CubeFeature::Edge { vertex_vertex } => {
												if get_bit(vertex_vertex, 0) ^ get_bit(vertex_vertex, 0 + 3) == 1 {
													if
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															0,
															get_bit(vertex_vertex, 1) as i32 * 2 - 1,
															0
														))).is_some() ||
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															0,
															0,
															get_bit(vertex_vertex, 2) as i32 * 2 - 1
														))).is_some() ||
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															0,
															get_bit(vertex_vertex, 1) as i32 * 2 - 1,
															get_bit(vertex_vertex, 2) as i32 * 2 - 1
														))).is_some()
													{ first_edge_covered = true; }
												} else if get_bit(vertex_vertex, 1) ^ get_bit(vertex_vertex, 1 + 3) == 1 {
													if
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															get_bit(vertex_vertex, 0) as i32 * 2 - 1,
															0,
															0
														))).is_some() ||
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															0,
															0,
															get_bit(vertex_vertex, 2) as i32 * 2 - 1
														))).is_some() ||
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															get_bit(vertex_vertex, 0) as i32 * 2 - 1,
															0,
															get_bit(vertex_vertex, 2) as i32 * 2 - 1
														))).is_some()
													{ first_edge_covered = true; }
												} else if get_bit(vertex_vertex, 2) ^ get_bit(vertex_vertex, 2 + 3) == 1 {
													if
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															get_bit(vertex_vertex, 0) as i32 * 2 - 1,
															0,
															0
														))).is_some() ||
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															0,
															get_bit(vertex_vertex, 1) as i32 * 2 - 1,
															0
														))).is_some() ||
														grid_1.get_voxel(&(collision_grid_pos_1 + IVec3::new(
															get_bit(vertex_vertex, 0) as i32 * 2 - 1,
															get_bit(vertex_vertex, 1) as i32 * 2 - 1,
															0
														))).is_some()
													{ first_edge_covered = true; }
												}
											},
											CubeFeature::Face { xyzs } => {
												if grid_1.get_voxel(&(
													U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_ivec3() * (1 - 2 * get_bit(xyzs, 3) as i32
												))).is_some() {
													return None;
												}
											},
										};
										match c.3 {
											CubeFeature::Vertex { xyz } => {
												for i in 1..8 {
													if grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
															get_bit(i, 0) as i32 * (get_bit(xyz, 0) as i32 * 2 - 1),
															get_bit(i, 1) as i32 * (get_bit(xyz, 1) as i32 * 2 - 1),
															get_bit(i, 2) as i32 * (get_bit(xyz, 2) as i32 * 2 - 1)
														))).is_some() { return None; }
												}
											},
											CubeFeature::Edge { vertex_vertex } => {
												if first_edge_covered { // only if both edges are covered
													if get_bit(vertex_vertex, 0) ^ get_bit(vertex_vertex, 0 + 3) == 1 {
														if
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																0,
																get_bit(vertex_vertex, 1) as i32 * 2 - 1,
																0
															))).is_some() ||
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																0,
																0,
																get_bit(vertex_vertex, 2) as i32 * 2 - 1
															))).is_some() ||
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																0,
																get_bit(vertex_vertex, 1) as i32 * 2 - 1,
																get_bit(vertex_vertex, 2) as i32 * 2 - 1
															))).is_some()
														{ return None; }
													} else if get_bit(vertex_vertex, 1) ^ get_bit(vertex_vertex, 1 + 3) == 1 {
														if
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																get_bit(vertex_vertex, 0) as i32 * 2 - 1,
																0,
																0
															))).is_some() ||
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																0,
																0,
																get_bit(vertex_vertex, 2) as i32 * 2 - 1
															))).is_some() ||
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																get_bit(vertex_vertex, 0) as i32 * 2 - 1,
																0,
																get_bit(vertex_vertex, 2) as i32 * 2 - 1
															))).is_some()
														{ return None; }
													} else if get_bit(vertex_vertex, 2) ^ get_bit(vertex_vertex, 2 + 3) == 1 {
														if
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																get_bit(vertex_vertex, 0) as i32 * 2 - 1,
																0,
																0
															))).is_some() ||
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																0,
																get_bit(vertex_vertex, 1) as i32 * 2 - 1,
																0
															))).is_some() ||
															grid_2.get_voxel(&(collision_grid_pos_2 + IVec3::new(
																get_bit(vertex_vertex, 0) as i32 * 2 - 1,
																get_bit(vertex_vertex, 1) as i32 * 2 - 1,
																0
															))).is_some()
														{ return None; }
													}
												}
											},
											CubeFeature::Face { xyzs } => {
												if grid_2.get_voxel(&(
													U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_ivec3() * (1 - 2 * get_bit(xyzs, 3) as i32
												))).is_some() {
													return None;
												}
											},
										};

										Some(Collision {
											part1: HalfCollision{
												body_id: if no_swap { physics_body_a.id() } else { physics_body_b.id() },
												grid_id: if no_swap { *grid_id_a } else { grid_id_b },
												voxel_pos: collision_grid_pos_1 + IVec3::new(x as i32, y as i32, z as i32),
												feature: c.1,
												collision: physics_body2.pose * grid_2.pose() * Pose::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.0,
												local_collision: physics_body1.pose.inverse() * physics_body2.pose * grid_2.pose() * Pose::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.0,
											},
											part2: HalfCollision{
												body_id: if no_swap { physics_body_b.id() } else { physics_body_a.id() },
												grid_id: if no_swap { grid_id_b } else { *grid_id_a },
												voxel_pos: collision_grid_pos_2,
												feature: c.3,
												collision: physics_body2.pose * grid_2.pose() * Pose::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.2,
												local_collision: grid_2.pose() * Pose::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.2,
											},
										})
									}));
								}
							}
						}
					}
				}
			}
		}
	}
	// for collision in collisions.iter() {
	// 	let e1 = &physics_bodies[collision.part1.body_index as usize];
	// 	let e2 = &physics_bodies[collision.part2.body_index as usize];

	// 	debug_draw::line(collision.part1.collision, collision.part2.collision, &Vec4::new(1.0, 0.0, 0.0, 1.0));
	// 	debug_draw::point(collision.part1.collision, &Vec4::new(1.0, 1.0, 1.0, 1.0), 0.1);
	// 	debug_draw::point(collision.part2.collision, &Vec4::new(1.0, 1.0, 1.0, 1.0), 0.1);

	// 	match collision.part1.feature {
	// 		CubeFeature::Vertex { xyz } => {
	// 			let v = U8Vec3::new(get_bit(xyz, 0), get_bit(xyz, 1), get_bit(xyz, 2)).as_vec3();
	// 			debug_draw::aabb(
	// 				e1.pose * e1.grids()[collision.part1.grid_index as usize].pose * (v + collision.part1.voxel_pos.as_vec3()) + Vec3::splat(0.01),
	// 				e1.pose * e1.grids()[collision.part1.grid_index as usize].pose * (v + collision.part1.voxel_pos.as_vec3()) + Vec3::splat(0.01),
	// 				&Vec4::ONE
	// 			);
	// 		},
	// 		CubeFeature::Edge { vertex_vertex } => {
	// 			let v1 = U8Vec3::new(get_bit(vertex_vertex, 0), get_bit(vertex_vertex, 1), get_bit(vertex_vertex, 2)).as_vec3();
	// 			let v2 = U8Vec3::new(get_bit(vertex_vertex, 3), get_bit(vertex_vertex, 4), get_bit(vertex_vertex, 5)).as_vec3();
	// 			debug_draw::line(
	// 				e1.pose * e1.grids()[collision.part1.grid_index as usize].pose * (v1 + collision.part1.voxel_pos.as_vec3()),
	// 				e1.pose * e1.grids()[collision.part1.grid_index as usize].pose * (v2 + collision.part1.voxel_pos.as_vec3()),
	// 				&Vec4::ONE
	// 			);
	// 		},
	// 		CubeFeature::Face { xyzs } => {
	// 			let v = U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_vec3() * (0.5 - get_bit(xyzs, 3) as f32);
	// 			debug_draw::aabb(
	// 				e1.pose * e1.grids()[collision.part1.grid_index as usize].pose * (v + collision.part1.voxel_pos.as_vec3() + Vec3::splat(0.5)) - Vec3::splat(0.01),
	// 				e1.pose * e1.grids()[collision.part1.grid_index as usize].pose * (v + collision.part1.voxel_pos.as_vec3() + Vec3::splat(0.5)) + Vec3::splat(0.01),
	// 				&Vec4::W
	// 			);
	// 		},
	// 	}
	// 	match collision.part2.feature {
	// 		CubeFeature::Vertex { xyz } => {
	// 			let v = U8Vec3::new(get_bit(xyz, 0), get_bit(xyz, 1), get_bit(xyz, 2)).as_vec3();
	// 			debug_draw::aabb(
	// 				e2.pose * e2.grids()[collision.part2.grid_index as usize].pose * (v + collision.part2.voxel_pos.as_vec3()) - Vec3::splat(0.01),
	// 				e2.pose * e2.grids()[collision.part2.grid_index as usize].pose * (v + collision.part2.voxel_pos.as_vec3()) + Vec3::splat(0.01),
	// 				&Vec4::ONE
	// 			);
	// 		},
	// 		CubeFeature::Edge { vertex_vertex } => {
	// 			let v1 = U8Vec3::new(get_bit(vertex_vertex, 0), get_bit(vertex_vertex, 1), get_bit(vertex_vertex, 2)).as_vec3();
	// 			let v2 = U8Vec3::new(get_bit(vertex_vertex, 3), get_bit(vertex_vertex, 4), get_bit(vertex_vertex, 5)).as_vec3();
	// 			debug_draw::line(
	// 				e2.pose * e2.grids()[collision.part2.grid_index as usize].pose * (v1 + collision.part2.voxel_pos.as_vec3()),
	// 				e2.pose * e2.grids()[collision.part2.grid_index as usize].pose * (v2 + collision.part2.voxel_pos.as_vec3()),
	// 				&Vec4::ONE
	// 			);
	// 		},
	// 		CubeFeature::Face { xyzs } => {
	// 			let v = U8Vec3::new(get_bit(xyzs, 0), get_bit(xyzs, 1), get_bit(xyzs, 2)).as_vec3() * (0.5 - get_bit(xyzs, 3) as f32);
	// 			debug_draw::aabb(
	// 				e2.pose * e2.grids()[collision.part2.grid_index as usize].pose * (v + collision.part2.voxel_pos.as_vec3() + Vec3::splat(0.5)) - Vec3::splat(0.01),
	// 				e2.pose * e2.grids()[collision.part2.grid_index as usize].pose * (v + collision.part2.voxel_pos.as_vec3() + Vec3::splat(0.5)) + Vec3::splat(0.01),
	// 				&Vec4::W
	// 			);
	// 		},
	// 	}
	// }
	// tracy_client::plot!(
	// 	"collisions checked",
	// 	unsafe { CHECK_COUNTER }.into()
	// );
	// tracy_client::plot!(
	// 	"collisions",
	// 	collisions.len() as f64
	// );
	collisions
}

fn get_collision(pose: &Pose, voxels: &voxels::Voxels, separating_axes: &Vec<((f32, f32), (f32, f32), Vec3, u8)>, to_global: &Pose) -> Vec<(Vec3, CubeFeature, Vec3, CubeFeature, I16Vec3)> {
	let mut collisions = vec![];
	for x in -1..2 {
		for y in -1..2 {
			for z in -1..2 {
				let vec = I16Vec3::new(x, y, z);
				if voxels.get_voxel(&(pose.translation.floor().as_i16vec3() + vec)).is_some() {
					let shift = Pose::new(pose.translation.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5), Quat::IDENTITY);
					get_collision_1x1x1_voxel(
						&(shift.inverse() * pose),
						separating_axes,
						&(shift * to_global)
					).into_iter().for_each(|c| {
						collisions.push((
							shift * c.0,
							c.1,
							shift * c.2,
							c.3,
							pose.translation.floor().as_i16vec3() + vec
						))
					});
				}
			}
		}
	}
	return collisions;
}

fn get_collision_1x1x1_voxel(pose: &Pose, separating_axes: &Vec<((f32, f32), (f32, f32), Vec3, u8)>, _to_global: &Pose) -> Vec<(Vec3, CubeFeature, Vec3, CubeFeature)> {
	unsafe { CHECK_COUNTER += 1; }
	if pose.translation.length_squared() >= 3.0 { return vec![]; }
	let mut bests = vec![];
	let mut best_dis = 10.0;
	for ((unshifted_min_1, unshifted_max_1), (min_2, max_2), axis, index) in separating_axes {
		let shift = pose.translation.dot(*axis);
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
				let v = pose * (U8Vec3::new(get_bit(i, 0), get_bit(i, 1), get_bit(i, 2)).as_vec3() - 0.5);
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
			// best_vertices.iter().for_each(|v| debug_draw::point(p + q * v.0, Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			// best_vertices.iter().for_each(|v| debug_draw::point(p + q * (best.1 * best.0.0 + v.0 - v.0.project_onto(best.1)), Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			let face_vec = best.1.round().as_i8vec3() * axis_neg as i8;
			collisions.extend(best_vertices.into_iter().map(|v| {(
				v.0,
				v.1,
				best.1 * best.0.1 + v.0 - v.0.project_onto(best.1),
				CubeFeature::Face { xyzs: face_vec.abs().as_u8vec3().dot(U8Vec3::new(1, 2, 4)) + 8 * (face_vec.element_sum().signum() == -1) as u8 },
			)}));
		} else if best.2 < 6 {
			assert!((best.1 - pose.rotation * Vec3::X).length() < 0.0001 || (best.1 - pose.rotation * Vec3::Y).length() < 0.0001 || (best.1 - pose.rotation * Vec3::Z).length() < 0.0001);
			let mut best_vertices = vec![];
			let mut best_dis = 10.0;
			(0..8).for_each(|i| {
				let v = U8Vec3::new(get_bit(i, 0), get_bit(i, 1), get_bit(i, 2)).as_vec3() - 0.5;
				let dis = (v - pose.translation).dot(best.1) * -axis_neg;
				let surface_pos = pose.rotation.inverse() * ((v - pose.translation) - (v - pose.translation).project_onto(best.1));
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
			// best_vertices.iter().for_each(|v| debug_draw::point(p + q * v.0, Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			// best_vertices.iter().for_each(|v| debug_draw::point(p + q * (best.1 * best.0.0 + v.0 - v.0.project_onto(best.1)), Vec4::new(0.0, 1.0, 0.0, 1.0), 0.05));
			let face_vec = (pose.rotation.inverse() * best.1).round().as_i8vec3() * -axis_neg as i8;
			collisions.extend(best_vertices.into_iter().map(|v| {(
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

			let axis1 = pose.rotation * axes[index_1 as usize]; // 1 axis
			let not_axes_1 = not_axes[index_1 as usize];
			let not_axes_xyz_u8_1 = not_axes_xyz_u8[index_1 as usize];

			let axis2 = axes[index_2 as usize]; // 2 axis
			let not_axes_2 = not_axes[index_2 as usize];
			let not_axes_xyz_u8_2 = not_axes_xyz_u8[index_2 as usize];

			(0..4).for_each(|i| {
				let edge_1 = pose * ((if i & 1 == 0 { -not_axes_1.0 } else { not_axes_1.0 } + if i & 2 == 0 { -not_axes_1.1 } else { not_axes_1.1 }) * 0.5);
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
