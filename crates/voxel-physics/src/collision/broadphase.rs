use glam::{I16Vec3, IVec3, Quat, Vec3};
use tracy_client::span;

use bevy::transform::components::Transform;
use voxel_bvh::bvh::BVH;
use voxel_data::grid::SubGridId;
use voxel_data::transform_ext::TransformExt;

use crate::sparse_set::SparseSet;
use crate::{GridId, PhysicsBodyId};

use super::narrowphase::{compute_1x1x1_cube_separating_axes, get_collision};
use super::{BodyView, Collision, GridCollider, HalfCollision};

pub fn get_collisions(
	bodies: &SparseSet<PhysicsBodyId, BodyView>,
	grids: &SparseSet<GridId, GridCollider>,
	bvh: &BVH<(PhysicsBodyId, GridId, SubGridId)>,
) -> Vec<Collision> {
	let _zone = span!("Do Collisions");
	let mut collisions: Vec<Collision> = vec![];
	for (grid_id_a, grid_col_a) in grids {
		let body_a_id = grid_col_a.body;
		let physics_body_a = bodies.get(&body_a_id).unwrap();
		if physics_body_a.is_static { continue; }
		let grid_local_transform_a = grid_col_a.local_transform;
		let grid_transform_a = physics_body_a.transform * *grid_local_transform_a;
		for &(_sub_grid_id_a, sub_grid_a) in grid_col_a.sub_grids.iter() {
			let sub_grid_transform_a = grid_transform_a * Transform::from_translation(sub_grid_a.sub_grid_pos().as_vec3());
			let bound = if let Some(aabb) = sub_grid_a.aabb(&sub_grid_transform_a) { aabb } else { continue; };
			for (body_b_id, grid_id_b, sub_grid_id_b) in bvh.get_collisions(&bound) {
				if body_b_id == body_a_id { continue; }
				let physics_body_b = bodies.get(&body_b_id).unwrap();
				if !physics_body_b.is_static && body_a_id < body_b_id { continue; }
				let grid_col_b = grids.get(&grid_id_b).unwrap();
				let grid_local_transform_b = grid_col_b.local_transform;
				let sub_grid_b = grid_col_b.sub_grids.iter().find(|(id, _)| *id == sub_grid_id_b).map(|(_, s)| *s).unwrap();
				let no_swap = sub_grid_a.get_voxels().get_voxels().len() < sub_grid_b.get_voxels().get_voxels().len();
				let (physics_body1, grid_local_transform_1, sub_grid_1, physics_body2, grid_local_transform_2, sub_grid_2) = {
					if no_swap { (physics_body_a, grid_local_transform_a, sub_grid_a, physics_body_b, grid_local_transform_b, sub_grid_b) }
					else { (physics_body_b, grid_local_transform_b, sub_grid_b, physics_body_a, grid_local_transform_a, sub_grid_a) }
				};
				let transform_of_1_in_2 = {
					Transform::from_translation(-sub_grid_2.sub_grid_pos().as_vec3()) * grid_local_transform_2.inverse() * physics_body2.transform.inverse() *
					physics_body1.transform * *grid_local_transform_1 * Transform::from_translation(sub_grid_1.sub_grid_pos().as_vec3())
				};
				let separating_axis = compute_1x1x1_cube_separating_axes(transform_of_1_in_2.rotation);
				for voxel in sub_grid_1.get_voxels().get_voxels().iter() {
					for x in 0..voxel.1 {
						for y in 0..voxel.1 {
							for z in 0..voxel.1 {
								collisions.extend(get_collision(
									&(transform_of_1_in_2 * Transform { translation: (voxel.0 + I16Vec3::new(x as i16, y as i16, z as i16)).as_vec3() + Vec3::new(0.5, 0.5, 0.5), rotation: Quat::IDENTITY, scale: Vec3::ONE }),
									sub_grid_2.get_voxels(),
									&separating_axis,
									&(physics_body2.transform * *grid_local_transform_2),
								).iter().map(|c| {
									let collision_grid_pos_1 = voxel.0.as_ivec3() + sub_grid_1.sub_grid_pos();
									let collision_grid_pos_2 = c.4.as_ivec3() + sub_grid_2.sub_grid_pos();
									Collision {
										part1: HalfCollision {
											body_id: if no_swap { body_a_id } else { body_b_id },
											grid_id: if no_swap { *grid_id_a } else { grid_id_b },
											voxel_pos: collision_grid_pos_1 + IVec3::new(x as i32, y as i32, z as i32),
											feature: c.1,
											collision: physics_body2.transform * *grid_local_transform_2 * Transform::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.0,
											local_collision: physics_body1.transform.inverse() * physics_body2.transform * *grid_local_transform_2 * Transform::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.0,
										},
										part2: HalfCollision {
											body_id: if no_swap { body_b_id } else { body_a_id },
											grid_id: if no_swap { grid_id_b } else { *grid_id_a },
											voxel_pos: collision_grid_pos_2,
											feature: c.3,
											collision: physics_body2.transform * *grid_local_transform_2 * Transform::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.2,
											local_collision: *grid_local_transform_2 * Transform::from_translation(sub_grid_2.sub_grid_pos().as_vec3()) * c.2,
										},
									}
								}));
							}
						}
					}
				}
			}
		}
	}
	collisions
}
