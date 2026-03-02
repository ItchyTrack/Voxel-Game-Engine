use glam::{IVec3, Quat, Vec3};

use crate::{entity, voxels};

#[derive(Copy, Clone)]
pub struct Collision {
	pub id1: u32,
	pub id2: u32,
	pub pos: Vec3,
	pub normal: Vec3,
}

pub fn get_collisions(entities: &Vec<entity::Entity>) -> Vec<Collision> {
	let mut collisions: Vec<Collision> = vec![];
	for id_a in 0..(entities.len() - 1) {
		let entity_a = &entities[id_a];
		for id_b in (id_a + 1)..entities.len() {
			let entity_b = &entities[id_b];
			let (entity1, entity2) = {
				if entity_a.get_voxels().get_voxels().len() < entity_b.get_voxels().get_voxels().len()
					{ (entity_a, entity_b) }
				else
					{ (entity_b, entity_a) }
			};
			let pos_of_1_in_2 = entity2.orientation.inverse() * (entity1.position - entity2.position);
			let orientation_of_1_in_2 = entity2.orientation.inverse() * entity1.orientation;
			for voxel in entity1.get_voxels().get_voxels().iter() {
				collisions.extend(get_collision(
					&(&orientation_of_1_in_2 * (voxel.0.as_vec3() + Vec3::new(0.5, 0.5, 0.5)) + pos_of_1_in_2),
					&orientation_of_1_in_2,
					entity2.get_voxels()
				).iter().map(|c| Collision {
					id1: id_a as u32, // order of ids dont matter
					id2: id_b as u32,
					pos: entity2.orientation * c.pos + entity2.position,
					normal: c.normal,
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
								id1: c.id1,
								id2: c.id2,
								pos: c.pos + pos.floor() + vec.as_vec3() + Vec3::new(0.5, 0.5, 0.5),
								normal: c.normal,
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
		pos: pos / 2.0,
		normal: pos.normalize(),
	})
}