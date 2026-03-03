use glam::{IVec3, Quat, Vec3};

use crate::{entity, voxels};

#[derive(Copy, Clone)]
pub struct Collision {
	pub id1: u32,
	pub id2: u32,
	pub collision1: Vec3,
	pub collision2: Vec3,
	pub local_collision1: Vec3,
	pub local_collision2: Vec3,
}

pub fn get_collisions(entities: &Vec<entity::Entity>, pose_to_eval_at: &Vec<(Vec3, Quat)>) -> Vec<Collision> {
	let mut collisions: Vec<Collision> = vec![];
	for id_a in 0..(entities.len() - 1) {
		let entity_a = &entities[id_a];
		let local_bounding_box_a = entity_a.get_voxels().get_bounding_box();
		if local_bounding_box_a.is_none() { continue; }
		let (aabb_a_min, aabb_a_max) = calc_aabb(
			&pose_to_eval_at[id_a].0,
			&pose_to_eval_at[id_a].1,
			&(local_bounding_box_a.unwrap().0.as_vec3(), local_bounding_box_a.unwrap().1.as_vec3())
		);

		for id_b in (id_a + 1)..entities.len() {
			let entity_b = &entities[id_b];

			let local_bounding_box_b = entity_b.get_voxels().get_bounding_box();
			if local_bounding_box_b.is_none() { continue; }
			let (aabb_b_min, aabb_b_max) = calc_aabb(
				&pose_to_eval_at[id_b].0,
				&pose_to_eval_at[id_b].1,
				&(local_bounding_box_b.unwrap().0.as_vec3(), local_bounding_box_b.unwrap().1.as_vec3())
			);
			if	aabb_a_max.x < aabb_b_min.x ||
				aabb_a_min.x > aabb_b_max.x ||
				aabb_a_max.y < aabb_b_min.y ||
				aabb_a_min.y > aabb_b_max.y ||
				aabb_a_max.z < aabb_b_min.z ||
				aabb_a_min.z > aabb_b_max.z { continue; }

			let no_swap = entity_a.get_voxels().get_voxels().len() < entity_b.get_voxels().get_voxels().len();
			let (entity1, pose1, entity2, pose2) = {
				if no_swap { (entity_a, pose_to_eval_at[id_a], entity_b, pose_to_eval_at[id_b]) }
				else { (entity_b, pose_to_eval_at[id_b], entity_a, pose_to_eval_at[id_a]) }
			};
			let pos_of_1_in_2 = pose2.1.inverse() * (pose1.0 - pose2.0);
			let orientation_of_1_in_2 = pose2.1.inverse() * pose1.1;
			let separating_axis = compute_1x1x1_cube_separating_axis(orientation_of_1_in_2);
			for voxel in entity1.get_voxels().get_voxels().iter() {
				collisions.extend(get_collision(
					&(&orientation_of_1_in_2 * (voxel.0.as_vec3() + Vec3::new(0.5, 0.5, 0.5)) + pos_of_1_in_2),
					&orientation_of_1_in_2,
					entity2.get_voxels(),
					&separating_axis
				).iter().map(|c| Collision {
					id1: if no_swap { id_a as u32 } else { id_b as u32 },
					id2: if no_swap { id_b as u32 } else { id_a as u32 },
					collision1: pose2.1 * c.collision1 + entity2.position,
					collision2: pose2.1 * c.collision2 + entity2.position,
					local_collision1: orientation_of_1_in_2.inverse() * (c.collision1 - pos_of_1_in_2),
					local_collision2: c.collision2,
					..*c
				}));
			}
		}
	}
	collisions
}

fn get_collision(pos: &Vec3, orientation: &Quat, voxels: &voxels::Voxels, separating_axis: &Vec<((f32, f32), (f32, f32), Vec3)>) -> Vec<Collision> {
	let mut collisions: Vec<Collision> = vec![];
	for x in -1..2 {
		for y in -1..2 {
			for z in -1..2 {
				let vec = IVec3::new(x, y, z);
				if voxels.get_voxel(pos.floor().as_ivec3() + vec).is_some() {
					get_collision_1x1x1_voxel(&((pos - pos.floor()) - vec.as_vec3() - Vec3::new(0.5, 0.5, 0.5)), orientation, separating_axis).inspect(|c| {
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

fn get_collision_1x1x1_voxel(pos: &Vec3, _orientation: &Quat, separating_axis: &Vec<((f32, f32), (f32, f32), Vec3)>) -> Option<Collision> {
	if pos.length_squared() >= 3.0 { return None; }
	let mut best = Collision{
		id1: 0,
		id2: 0,
		collision1: Vec3::ZERO,
		collision2: Vec3::ZERO,
		local_collision1: Vec3::ZERO,
		local_collision2: Vec3::ZERO,
	};
	let mut best_dis = 10.0;
	for ((unshifted_min_1, unshifted_max_1), (min_2, max_2), axes) in separating_axis {
		let shift = pos.dot(*axes);
		let min_1 = unshifted_min_1 + shift;
		let max_1 = unshifted_max_1 + shift;
		if max_1 <= *min_2 || min_1 >= *max_2 {
			return None;
		}
		if min_1 > *min_2 {
			let overlap = max_2 - min_1;
			if overlap < best_dis {
				best_dis = overlap;
				best.collision1 = axes * min_1 + (pos - axes * shift) * 0.5;
				best.collision2 = axes * max_2 + (pos - axes * shift) * 0.5;
			}
		} else {
			let overlap = max_1 - *min_2;
			if overlap < best_dis {
				best_dis = overlap;
				best.collision1 = axes * max_1 + (pos - axes * shift) * 0.5;
				best.collision2 = axes * min_2 + (pos - axes * shift) * 0.5;
			}
		}
	}
	Some(best)
}


// assumes other cube has no rotation and both are centered at (0,0,0)
fn compute_1x1x1_cube_separating_axis(orientation: Quat) -> Vec<((f32, f32), (f32, f32), Vec3)> {
	let axis_6 = [
		Vec3::X,
		Vec3::Y,
		Vec3::Z,
		orientation * Vec3::X,
		orientation * Vec3::Y,
		orientation * Vec3::Z,
	];
	let axis_9 = (0..9).map(|i| axis_6[i/3].cross(axis_6[3+i%3]));

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

	axis_6.into_iter().chain(axis_9).filter_map(|axes| {
		if axes.length_squared() < 1e-6 { return None; }
		let mut min_a: f32 = 0.0; // cube 1
		let mut max_a: f32 = 0.0;
		for l in corners.map(|c| (orientation * c).dot(axes.normalize())) {
			min_a = min_a.min(l);
			max_a = max_a.max(l);
		}
		let mut min_b: f32 = 0.0; // cube 2
		let mut max_b: f32 = 0.0;
		for l in corners.map(|c| c.dot(axes.normalize())) {
			min_b = min_b.min(l);
			max_b = max_b.max(l);
		}
		assert!(min_b + max_b < 0.0001); // should be true
		assert!(min_a + max_a < 0.0001);

		Some(((min_a, max_a), (min_b, max_b), axes.normalize()))
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
