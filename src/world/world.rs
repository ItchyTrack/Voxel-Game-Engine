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


