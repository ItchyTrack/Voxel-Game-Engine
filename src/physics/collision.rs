use glam::{IVec3, Quat, Vec3};

use crate::{entity, voxels};

#[derive(Copy, Clone)]
pub struct Collision {
	pub id1: u32,
	pub id2: u32,
	pub collision1: Vec3,
	pub collision2: Vec3,
	pub normal: Vec3, // from 1 to 2
	pub overlap: f32,
}

pub fn get_collisions(entities: &Vec<entity::Entity>, pose_to_eval_at: &Vec<(Vec3, Quat)>) -> Vec<Collision> {
	let mut collisions: Vec<Collision> = vec![];
	for id_a in 0..(entities.len() - 1) {
		let entity_a = &entities[id_a];
		let entity_a_aabb = entity_a.get_aabb();
		for id_b in (id_a + 1)..entities.len() {
			let entity_b = &entities[id_b];

			let entity_b_aabb = entity_b.get_aabb();
			if entity_a_aabb.is_none() || entity_b_aabb.is_none() { continue; }
			let (a_min, a_max) = entity_a_aabb.unwrap();
			let (b_min, b_max) = entity_b_aabb.unwrap();
			if	a_max.x < b_min.x ||
				a_min.x > b_max.x ||
				a_max.y < b_min.y ||
				a_min.y > b_max.y ||
				a_max.z < b_min.z ||
				a_min.z > b_max.z { continue; }

			let swap = entity_a.get_voxels().get_voxels().len() < entity_b.get_voxels().get_voxels().len();
			let (entity1, pose1, entity2, pose2) = {
				if swap
					{ (entity_a, pose_to_eval_at[id_a], entity_b, pose_to_eval_at[id_b]) }
				else
					{ (entity_b, pose_to_eval_at[id_b], entity_a, pose_to_eval_at[id_a]) }
			};
			let pos_of_1_in_2 = pose2.1.inverse() * (pose1.0 - pose2.0);
			let orientation_of_1_in_2 = pose2.1.inverse() * pose1.1;
			for voxel in entity1.get_voxels().get_voxels().iter() {
				collisions.extend(get_collision(
					&(&orientation_of_1_in_2 * (voxel.0.as_vec3() + Vec3::new(0.5, 0.5, 0.5)) + pos_of_1_in_2),
					&orientation_of_1_in_2,
					entity2.get_voxels()
				).iter().map(|c| Collision {
					id1: if swap { id_b as u32 } else { id_a as u32 },
					id2: if swap { id_a as u32 } else { id_b as u32 },
					collision1: pose2.1 * c.collision1 + entity2.position,
					collision2: pose2.1 * c.collision2 + entity2.position,
					normal: pose2.1 * c.normal,
					..*c
				}));
			}
		}
	}
	collisions
}

fn get_collision(pos: &Vec3, orientation: &Quat, voxels: &voxels::Voxels) -> Vec<Collision> {
	let mut collisions: Vec<Collision> = vec![];
	for x in -1..2 {
		for y in -1..2 {
			for z in -1..2 {
				let vec = IVec3::new(x, y, z);
				if voxels.get_voxel(pos.floor().as_ivec3() + vec).is_some() {
					get_collision_1x1x1_voxel(&((pos - pos.floor()) - vec.as_vec3() - Vec3::new(0.5, 0.5, 0.5)), orientation).inspect(|c| {
						collisions.push(
							Collision {
								collision1: c.collision1 + pos.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5),
								collision2: c.collision2 + pos.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5),
								..*c
							}
						)
					});
				}
			}
		}
	}
	return collisions;
}

fn get_collision_1x1x1_voxel(pos: &Vec3, _orientation: &Quat) -> Option<Collision> {
	if pos.length_squared() >= 1.0 { return None; }
	Some(Collision {
		id1: 0,
		id2: 0,
		collision1: pos.normalize() * 0.5,
		collision2: pos - pos.normalize() * 0.5,
		normal: pos.normalize(),
		overlap: 1.0 - pos.length_squared(),
	})
}