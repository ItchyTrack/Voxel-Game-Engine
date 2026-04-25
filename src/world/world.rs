use crate::world::{grid::{Grid, GridId}, physics_body::{PhysicsBody, PhysicsBodyId}, resource_manager::ResourceManager, sparse_set::SparseSet, sub_grid::{SubGrid, SubGridId}};

pub struct World {
	pub physics_bodies: SparseSet<PhysicsBodyId, PhysicsBody>,
	pub grids: SparseSet<GridId, Grid>,
	pub sub_grids: SparseSet<SubGridId, SubGrid>,
	pub next_body_id: PhysicsBodyId,
	pub next_grid_id: GridId,
	pub next_sub_grid_id: SubGridId,
	pub voxel_tracker: VoxelTracker,

	pub constraints: HashMap<(PhysicsBodyId, PhysicsBodyId), BallJointConstraint>,
	pub impulses: HashMap<PhysicsBodyId, Vec<Impulse>>,
	pub solver: Solver,
	pub leaky_bucket: f32,

	pub bvh: RefCell<Option<BVH<(PhysicsBodyId, GridId, SubGridId)>>>,

	pub ecs: entity_component_system::EntityComponentSystem,

	pub resource_manager: ResourceManager,

	pub task_queue: TaskQueue,
	pub async_task_priority_queue: AsyncTaskPriorityQueue,
	pub async_task_priority_queue_threads: Vec<tokio::task::JoinHandle<()>>,
}

impl World {
	// called at any rate (faster is better and should be at least once per frame)
	pub fn update(&mut self) {
		let _zone = span!("World Update");
		if !self.debug_enables.freeze_gpu_grids {
			let _zone = span!("Do tasks");
			while let Some(task) = { self.task_queue.lock().unwrap().pop_front() } {
				task.run(self);
			}
		}

		{
			// let _zone = span!("Clean up");
			// let mut index = 0u32;
			// while index < self.physics_bodies.len() as u32 {
			// 	if self.physics_bodies[index as usize].clean_up() {
			// 		self.remove_physics_body_by_index(index);
			// 	} else {
			// 		index += 1;
			// 	}
			// }
			*self.bvh.borrow_mut() = None; // have to clear afer cleanup
		}

		self.leaky_bucket += dt;
		let time_step = 1.0 / 100.0;
		let current_time = Instant::now();
		while self.leaky_bucket >= time_step {
			self.physics_update(time_step);
			self.leaky_bucket -= time_step;
			let elapsed = current_time.elapsed().as_secs_f32(); // timeout should maybe be removed at some point
			if elapsed > 1.0 / 60.0 {
				self.leaky_bucket = 0.0;
			}
		}
	}

	fn physics_update(&mut self, dt: f32) {
		self.ecs.run_on_components_pair_mut::<Camera, ObjectPickup, _>(&mut |_entity_id, camera, object_pickup| {
			if object_pickup.is_holding() {
				object_pickup.hold_at_pos(&(camera.position + camera.forward() * 40.0), time_step, &mut self.physics_engine);
			}
		});
		self.ecs.run_on_components_mut::<Orientator, _>(&mut |_entity_id, orientator| {
			orientator.hold_at_orientation(&Quat::IDENTITY, time_step, &mut self.physics_engine);
		});
		let player_position = self.ecs.get_component::<Camera>(self.player_id).unwrap().position;
		self.ecs.run_on_components_mut::<PlayerTracker, _>(&mut |_entity_id, player_tracker| {
			player_tracker.track_pos(&player_position, time_step, &mut self.physics_engine);
		});
		let _zone = span!("PhysicsEngine update physics");
		{
			let bvh = &get_bvh_macro!(self);
			self.solver.solve(&mut self.physics_bodies, &self.grids, &self.sub_grids, &mut self.constraints, &self.impulses, dt, bvh);
			self.impulses.clear();
		}
		*self.bvh.borrow_mut() = None;
	}
}

// pub fn update_gpu_grid_tree(
// 	&self,
// 	packed_64_tree_dynamic_buffer: &mut PackedDynamicBuffer,
// 	packed_voxel_data_dynamic_buffer: &mut PackedDynamicBuffer,
// 	resource_manager: &mut ResourceManager,
// 	physics_engine: &PhysicsEngine,
// 	async_task_priority_queue: &AsyncTaskPriorityQueue,
// 	task_queue: &TaskQueue,
// 	id_to_hit_count: &HashMap<(PhysicsBodyId, GridId, SubGridId), u32>,
// 	physics_body_id: PhysicsBodyId,
// 	view_frustum: &camera::ViewFrustum,
// 	camera_pose: Pose,
// 	pose: &Pose
// ) -> Vec<(SubGridId, (u32, u32, Pose))> {
// 	self.sub_grids.iter().filter_map(|sub_grid| {
// 		let hit_count = id_to_hit_count.get(&(physics_body_id, self.id, sub_grid.id)).unwrap_or(&1);
// 		let grid_pose = pose * self.pose * Pose::from_translation(self.sub_grid_pos_to_grid_pos(&sub_grid.sub_grid_pos.as_ivec3()).as_vec3());
// 		Some((
// 			sub_grid.id(),
// 			(sub_grid.update_gpu_grid_tree(
// 				packed_64_tree_dynamic_buffer,
// 				packed_voxel_data_dynamic_buffer,
// 				resource_manager,
// 				physics_engine,
// 				async_task_priority_queue,
// 				task_queue,
// 				*hit_count,
// 				view_frustum,
// 				f32::max(camera_pose.translation.distance(grid_pose.translation) - 1000.0, 0.0) / 1000.0,
// 				-camera_pose.translation.distance(grid_pose.translation) / 1000.0,
// 				grid_pose
// 			)?)
// 		))}
// 	).collect()
// }

// pub fn update_gpu_grid_tree(
// 	&self,
// 	packed_64_tree_dynamic_buffer: &mut PackedDynamicBuffer,
// 	packed_voxel_data_dynamic_buffer: &mut PackedDynamicBuffer,
// 	resource_manager: &mut ResourceManager,
// 	physics_engine: &PhysicsEngine,
// 	async_task_priority_queue: &AsyncTaskPriorityQueue,
// 	task_queue: &TaskQueue,
// 	id_to_hit_count: &HashMap<(PhysicsBodyId, GridId, SubGridId), u32>,
// 	view_frustum: &camera::ViewFrustum,
// 	camera_pose: Pose,
// ) -> Vec<((GridId, SubGridId), (u32, u32, Pose))> {
// 	let mut gpu_grid_tree_id_to_id_poses = vec![];
// 	for grid in self.grids.iter() {
// 		for (sub_grid_id, data) in grid.update_gpu_grid_tree(
// 			packed_64_tree_dynamic_buffer,
// 			packed_voxel_data_dynamic_buffer,
// 			resource_manager,
// 			physics_engine,
// 			async_task_priority_queue,
// 			task_queue,
// 			id_to_hit_count,
// 			self.id(),
// 			view_frustum,
// 			camera_pose,
// 			&self.pose
// 		) {
// 			gpu_grid_tree_id_to_id_poses.push(((grid.id(), sub_grid_id), data));
// 		}
// 	}
// 	gpu_grid_tree_id_to_id_poses
// }


