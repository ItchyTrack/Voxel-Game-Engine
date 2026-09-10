use crate::math::Mat3;
use voxel_math::Fixed;
use voxel_math::FixedVec3;
use std::collections::HashMap;

use bevy::ecs::change_detection::Mut;
use bevy::math::{IVec3, Quat};
use bevy::prelude::{Entity, Query};
use bevy::tasks::{ComputeTaskPool, ParallelSliceMut};
use tracy_client::span;

use voxel_transform::{Scale, Transform};
use crate::math::{Mat6, Vec6, Wide, WideVec3, solve_symmetric};
use super::body::SolverBody;
use crate::sparse_set::SparseSet;
use crate::{GridId, PhysicsBodyId};
use crate::{constraints::BallJoint, collision};

use super::ball_joint_constraint::AvbdBallJointConstraint;
use super::collision_constraint::CollisionConstraint;
use super::physics_constraint::PhysicsConstraint;

type CollisionKlMapKey = (PhysicsBodyId, GridId, IVec3, collision::CubeFeature, PhysicsBodyId, GridId, IVec3, collision::CubeFeature);

pub struct Solver {
	collisions_kl_map: HashMap<CollisionKlMapKey, (WideVec3, WideVec3)>,
}

// fn mat6_outer(a: Vec6, b: Vec6) -> Mat6 {
// 	Mat6::from_cols(a * b.get(0), a * b.get(1), a * b.get(2), a * b.get(3), a * b.get(4), a * b.get(5))
// }

impl Solver {
	pub fn new() -> Self {
		Self {
			collisions_kl_map: HashMap::new(),
		}
	}

	pub fn sub_quat(q1: &Quat, q2: &Quat) -> FixedVec3 { crate::math::rotation_delta(*q1, *q2) }

	pub fn sub_state(state_a: &Transform, state_b: &Transform) -> Vec6 {
		Vec6::from_vec3((state_a.translation - state_b.translation).into(), Self::sub_quat(&state_a.rotation, &state_b.rotation).into())
	}

	pub fn solve(
		&mut self,
		physics_bodies: &mut SparseSet<PhysicsBodyId, SolverBody>,
		collisions: &[collision::Collision],
		constraints: &mut Query<(Entity, &BallJoint, &mut AvbdBallJointConstraint)>,
		dt: Fixed,
	) {
		let _zone = span!("Solve Collisions");
		if !crate::math::usable_timestep(dt) { return; }
		let mut constraint_map: HashMap<Entity, ((PhysicsBodyId, PhysicsBodyId), Mut<'_, AvbdBallJointConstraint>)> = HashMap::new();
		for (entity, joint, avbd_constraint) in constraints.iter_mut() {
			if let Some(physics_body_1) = physics_bodies.get(&joint.body_1) {
				if let Some(physics_body_2) = physics_bodies.get(&joint.body_2) {
					let mut avbd_constraint = avbd_constraint;
					avbd_constraint.update_attachment_com(joint, &physics_body_1.local_center_of_mass(), &physics_body_2.local_center_of_mass());
					constraint_map.insert(entity, ((joint.body_1, joint.body_2), avbd_constraint));
				}
			}
		}
		let initial_all: SparseSet<PhysicsBodyId, Transform> = SparseSet::from_iter(
			physics_bodies.iter().map(|(physics_body_id, physics_body)| (
				*physics_body_id,
				Transform { translation: physics_body.global_rotated_center_of_mass(), rotation: Quat::IDENTITY, scale: Scale::ONE } * physics_body.transform
			))
		);
		for ((body_1, body_2), constraint) in constraint_map.values_mut() {
			constraint.init(&initial_all[body_1], &initial_all[body_2]);
		}
		let mut collision_constraints: Vec<CollisionConstraint> = collisions.iter().map(
			|c| {
				let body1 = &physics_bodies.get(&c.part1.body_id).unwrap();
				let body2 = &physics_bodies.get(&c.part2.body_id).unwrap();
				let collision = collision::Collision {
					part1: collision::HalfCollision {
						local_collision: c.part1.local_collision - body1.local_center_of_mass(),
						..c.part1
					},
					part2: collision::HalfCollision {
						local_collision: c.part2.local_collision - body2.local_center_of_mass(),
						..c.part2
					},
				};
				let (old_penalty, old_lambda) = self.collisions_kl_map.get(&if c.part1.body_id < c.part2.body_id {
					(
						collision.part1.body_id, collision.part1.grid_id, collision.part1.voxel_pos, collision.part1.feature,
						collision.part2.body_id, collision.part2.grid_id, collision.part2.voxel_pos, collision.part2.feature
					)
				} else {
					(
						collision.part2.body_id, collision.part2.grid_id, collision.part2.voxel_pos, collision.part2.feature,
						collision.part1.body_id, collision.part1.grid_id, collision.part1.voxel_pos, collision.part1.feature
					)
				}).unwrap_or(&(WideVec3::ZERO, WideVec3::ZERO));
				let mut collision_constraint = CollisionConstraint::new(collision, old_penalty, old_lambda);
				collision_constraint.init(initial_all.get(&c.part1.body_id).unwrap(), &initial_all.get(&c.part2.body_id).unwrap());
				collision_constraint
			}
		).collect();
		self.collisions_kl_map.clear();
		let y_all: SparseSet<PhysicsBodyId, Transform> = SparseSet::from_iter(physics_bodies.iter().map(|(physics_body_id, physics_body)| {
			(*physics_body_id, physics_body.integrated_center_of_mass_transform)
		}));
		let mut x_guess = y_all.clone();

		let _color_zone = span!("Graph Coloring");
		let mut body_collisions: HashMap<PhysicsBodyId, Vec<(usize, bool)>> = HashMap::new();
		for (index, collision_constraint) in collision_constraints.iter().enumerate() {
			body_collisions.entry(collision_constraint.collision.part1.body_id).or_default().push((index, true));
			body_collisions.entry(collision_constraint.collision.part2.body_id).or_default().push((index, false));
		}
		let mut body_joints: HashMap<PhysicsBodyId, Vec<(Entity, bool)>> = HashMap::new();
		for (joint_entity, ((body_1, body_2), _avbd_constraint)) in constraint_map.iter() {
			body_joints.entry(*body_1).or_default().push((*joint_entity, true));
			body_joints.entry(*body_2).or_default().push((*joint_entity, false));
		}

		// Color the dynamic bodies (VBD colors vertices, not constraints) so that two
		// bodies sharing a constraint never share a color. Bodies of one color touch
		// disjoint constraints, so their per-body solves can run in parallel.
		let mut adjacency: HashMap<PhysicsBodyId, Vec<PhysicsBodyId>> = HashMap::new();
		let mut edges: Vec<(PhysicsBodyId, PhysicsBodyId)> = Vec::with_capacity(collision_constraints.len() + constraint_map.len());
		for collision_constraint in &collision_constraints {
			edges.push((collision_constraint.collision.part1.body_id, collision_constraint.collision.part2.body_id));
		}
		edges.extend(constraint_map.values().map(|((body_1, body_2), _avbd_constraint)| (*body_1, *body_2)));
		for (a, b) in edges {
			let both_dynamic = a != b
				&& physics_bodies.get(&a).is_some_and(|body| !body.is_static)
				&& physics_bodies.get(&b).is_some_and(|body| !body.is_static);
			if both_dynamic {
				adjacency.entry(a).or_default().push(b);
				adjacency.entry(b).or_default().push(a);
			}
		}

		// Sorted visitation makes the greedy coloring deterministic across frames.
		let mut dynamic_ids: Vec<PhysicsBodyId> = physics_bodies.iter()
			.filter(|(_, body)| !body.is_static)
			.map(|(id, _)| *id)
			.collect();
		dynamic_ids.sort_unstable_by_key(|id| id.to_bits());

		let mut color_of: HashMap<PhysicsBodyId, usize> = HashMap::new();
		let mut colors: Vec<Vec<PhysicsBodyId>> = Vec::new();
		let mut neighbor_colors: Vec<usize> = Vec::new();
		for id in dynamic_ids {
			neighbor_colors.clear();
			if let Some(neighbors) = adjacency.get(&id) {
				neighbor_colors.extend(neighbors.iter().filter_map(|n| color_of.get(n).copied()));
			}
			let mut color = 0;
			while neighbor_colors.contains(&color) { color += 1; }
			color_of.insert(id, color);
			while colors.len() <= color { colors.push(Vec::new()); }
			colors[color].push(id);
		}
		drop(_color_zone);

		let pool = ComputeTaskPool::get();
		{
			use std::sync::Once;
			static ONCE: Once = Once::new();
			ONCE.call_once(|| {
				let sizes: Vec<usize> = colors.iter().map(|c| c.len()).collect();
				bevy::log::info!("AVBD solver: {} compute threads, {} colors, bodies per color = {:?}", pool.thread_num(), colors.len(), sizes);
			});
		}

		// Split each color into work buckets balanced by estimated cost (number of
		// touching constraints) rather than body count. Heaviest bodies are dealt into
		// the currently-lightest bucket (greedy LPT), so no single thread is handed the
		// heaviest work and a thread that grabs two buckets stays near one bucket's load.
		let body_cost = |id: &PhysicsBodyId| -> usize {
			body_collisions.get(id).map_or(0, |v| v.len()) + body_joints.get(id).map_or(0, |v| v.len()) + 1
		};
		let color_buckets: Vec<Vec<Vec<PhysicsBodyId>>> = colors.iter().map(|color| {
			let bucket_count = pool.thread_num().clamp(1, color.len().max(1));
			let mut sorted = color.clone();
			sorted.sort_unstable_by_key(|id| std::cmp::Reverse(body_cost(id)));
			let mut buckets: Vec<Vec<PhysicsBodyId>> = vec![Vec::new(); bucket_count];
			let mut loads = vec![0usize; bucket_count];
			for id in sorted {
				let lightest = loads.iter().enumerate().min_by_key(|(_, load)| **load).map(|(i, _)| i).unwrap();
				loads[lightest] += body_cost(&id);
				buckets[lightest].push(id);
			}
			buckets
		}).collect();

		let iterations = 30;
		let total_iterations = iterations + 1; // because post stabilize
		for iteration in 0..total_iterations {
			let _zone = span!("Solve Iteration");
			let alpha = if iteration < iterations { Wide::from_num(999) / Wide::from_num(1000) } else { Wide::ZERO };

			// Primal step. Colors run sequentially; bodies within a color run in parallel
			// against the current `x_guess`, then their results are committed before the
			// next color (multi-color Gauss-Seidel).
			{
				let _zone = span!("Primal");
				let bodies = &*physics_bodies;
				let collision_constraints = &collision_constraints;
				let constraints = &constraint_map;
				let body_collisions = &body_collisions;
				let body_joints = &body_joints;
				let y_all = &y_all;
				let initial_all = &initial_all;
				for buckets in &color_buckets {
					let x_guess_ref = &x_guess;
					let results: Vec<Vec<(PhysicsBodyId, Transform)>> = pool.scope(|scope| {
						for bucket in buckets {
							scope.spawn(async move {
							let _zone = span!("Primal Chunk");
							bucket.iter().map(|physics_body_id| {
								let physics_body = bodies.get(physics_body_id).unwrap();
							let m = Mat6::from_mat3(Wide::from_num(physics_body.mass()) * Mat3::IDENTITY, Mat3::ZERO, Mat3::ZERO, Mat3::from_inertia(physics_body.rotational_inertia().mat));
							let mut h: Mat6 = m / (Wide::from(dt) * Wide::from(dt));
							let mut f: Vec6 = h * Self::sub_state(&x_guess_ref[physics_body_id], &y_all[physics_body_id]);

							if let Some(touching) = body_collisions.get(physics_body_id) {
								for (index, is_part1) in touching {
									let collision_constraint = &collision_constraints[*index];
									let result = collision_constraint.get_updated(
										&x_guess_ref[&collision_constraint.collision.part1.body_id], &initial_all[&collision_constraint.collision.part1.body_id],
										&x_guess_ref[&collision_constraint.collision.part2.body_id], &initial_all[&collision_constraint.collision.part2.body_id],
										alpha,
										*is_part1
									);
									if let Some((c_f, c_h)) = result {
										f += c_f;
										h += c_h;
									}
								}
							}
							if let Some(touching) = body_joints.get(physics_body_id) {
								for (joint_entity, is_first) in touching {
									let ((body_1, body_2), avbd_constraint) = &constraints[joint_entity];
									let result = avbd_constraint.get_updated(
										&x_guess_ref[body_1], &initial_all[body_1],
										&x_guess_ref[body_2], &initial_all[body_2],
										alpha,
										*is_first
									);
									if let Some((c_f, c_h)) = result {
										f += c_f;
										h += c_h;
									}
								}
							}

							let x_change = solve_symmetric(h, -f);
							let mut state = x_guess_ref[physics_body_id];
							state.translation += x_change.upper_vec3().to_fixed();
							state.rotation = crate::math::rotation_correction(state.rotation, x_change.lower_vec3().to_fixed());
							(*physics_body_id, state)
						}).collect::<Vec<_>>()
							});
						}
					});
					for (id, state) in results.into_iter().flatten() {
						x_guess.insert(id, state);
					}
				}
			}

			if iteration < iterations {
				let _zone = span!("Dual");
				{
					let x_guess = &x_guess;
					let initial_all = &initial_all;
					collision_constraints.par_splat_map_mut(pool, None, |_, chunk| {
						let _zone = span!("Dual Chunk");
						for collision_constraint in chunk {
							collision_constraint.update_dual(
								&x_guess[&collision_constraint.collision.part1.body_id], &initial_all[&collision_constraint.collision.part1.body_id],
								&x_guess[&collision_constraint.collision.part2.body_id], &initial_all[&collision_constraint.collision.part2.body_id],
								alpha
							);
						}
					});
				}
				for ((physics_body_id_1, physics_body_id_2), avbd_constraint) in constraint_map.values_mut() {
					avbd_constraint.update_dual(
						&x_guess[physics_body_id_1], &initial_all[physics_body_id_1],
						&x_guess[physics_body_id_2], &initial_all[physics_body_id_2],
						alpha
					);
				}
			}
			if iteration == iterations - 1 { // before post stabilize
				for (physics_body_id, physics_body) in physics_bodies.iter_mut() {
					physics_body.velocity = (x_guess[physics_body_id].translation - initial_all[physics_body_id].translation) / dt;
					physics_body.angular_velocity = crate::math::rotation_difference(x_guess[physics_body_id].rotation, initial_all[physics_body_id].rotation) / dt;
				}
			}
		}
		// after post stabilize
		for (physics_body_id, physics_body) in physics_bodies.iter_mut() {
			physics_body.transform.rotation = x_guess.get(physics_body_id).unwrap().rotation;
			physics_body.transform.translation = x_guess.get(physics_body_id).unwrap().translation - physics_body.global_rotated_center_of_mass();
		}
		// save K and L
		for collision_constraint in collision_constraints {
			let collision = collision_constraint.collision;
			self.collisions_kl_map.insert(if collision.part1.body_id < collision.part2.body_id {
				(
					collision.part1.body_id, collision.part1.grid_id, collision.part1.voxel_pos, collision.part1.feature,
					collision.part2.body_id, collision.part2.grid_id, collision.part2.voxel_pos, collision.part2.feature
				)
			} else {
				(
					collision.part2.body_id, collision.part2.grid_id, collision.part2.voxel_pos, collision.part2.feature,
					collision.part1.body_id, collision.part1.grid_id, collision.part1.voxel_pos, collision.part1.feature
				)
			}, (collision_constraint.penalty, collision_constraint.lambda));
		}
	}
}
