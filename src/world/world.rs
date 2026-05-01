use std::{collections::{HashMap, VecDeque}, ops::{AddAssign, SubAssign}, pin::Pin, sync::Arc, time::Instant};
use glam::{I8Vec3, IVec3, Vec3};
use num::Zero;
use parking_lot::{MappedRwLockReadGuard, MappedRwLockWriteGuard, Mutex, RwLock, RwLockReadGuard, RwLockWriteGuard};

use async_priority_queue::PriorityQueue;
use tracy_client::span;
use wgpu::Device;

use crate::player::camera::ViewFrustum;

use super::{physics_body::{PhysicsBody, PhysicsBodyId}, grid::{Grid, GridId, GridManager, SubGridId}, physics_body_resource::GridResource};
use super::{physics_solver::{ball_joint_constraint::BallJointConstraint, solver::{Solver, Impulse}, bvh::BVH}};
use super::{sparse_set::SparseSet, entity_component_system::entity_component_system::EntityComponentSystem};
use super::{resource_manager::{ResourceManager, ResourceUUID, ResourceInfoType}, physics_body_resource::PhysicsBodyResource};
use super::{pose::Pose, gpu::world_gpu_data::WorldGpuData, voxel_tracker::{TrackedVoxelId, VoxelTracker}};

pub struct Task {
	task_func: Box<dyn FnOnce(&World) + Send + 'static>,
}

impl Task {
	pub fn new<F: FnOnce(&World) + Send + 'static>(function: F) -> Self {
		Self {
			task_func: Box::new(function),
		}
	}
	pub fn run(self, world: &World) {
		(self.task_func)(world);
	}
}

pub struct PriorityTask {
	priority: f32,
	task_func: Pin<Box<dyn Future<Output = ()> + Send + 'static>>,
}

impl PriorityTask {
	pub fn new<F: Future<Output = ()> + Send + 'static>(priority: f32, function: F) -> Self {
		Self {
			priority,
			task_func: Box::pin(function),
		}
	}
	pub fn run(self) -> impl Future<Output = ()> {
		self.task_func
	}
}

impl PartialEq for PriorityTask {
	fn eq(&self, other: &Self) -> bool {
		self.priority == other.priority
	}
}

impl Eq for PriorityTask {}

impl PartialOrd for PriorityTask {
	fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
		return self.priority.partial_cmp(&other.priority);
	}
}

impl Ord for PriorityTask {
	fn cmp(&self, other: &Self) -> std::cmp::Ordering {
		self.priority.total_cmp(&other.priority)
	}
}

pub type TaskQueue = Arc<Mutex<VecDeque<Task>>>;
pub type AsyncTaskPriorityQueue = Arc<PriorityQueue<PriorityTask>>;

macro_rules! get_bvh_macro {
	($self:expr) => {{
		if $self.bvh.read().is_none() {
			let mut bounds = vec![];
			{
				let _zone = span!("Collect aabb");
				for (physics_body_id, physics_body) in $self.physics_bodies.read().iter() {
					for grid_id in physics_body.grids() {
						let grid = $self.grid(*grid_id).unwrap();
						for (sub_grid_id, sub_grid) in grid.sub_grids() {
							if let Some(bound) = sub_grid.local_aabb(&(physics_body.pose.rotation * grid.pose().rotation)) {
								bounds.push(((*physics_body_id, *grid_id, *sub_grid_id), bound));
							}
						}
					}
				}
			}
			*$self.bvh.write() = Some(BVH::new(bounds));
		}
		RwLockReadGuard::map($self.bvh.read(), |bvh| bvh.as_ref().unwrap())
	}};
}

pub struct World {
	pub physics_bodies: RwLock<SparseSet<PhysicsBodyId, PhysicsBody>>,
	pub grid_manager: RwLock<GridManager>,
	pub next_physics_body_id: RwLock<PhysicsBodyId>,
	pub next_grid_id: RwLock<GridId>,
	pub next_sub_grid_id: RwLock<SubGridId>,
	pub voxel_tracker: RwLock<VoxelTracker>,

	pub constraints: RwLock<HashMap<(PhysicsBodyId, PhysicsBodyId), BallJointConstraint>>,
	pub impulses: RwLock<SparseSet<PhysicsBodyId, Vec<Impulse>>>,
	pub solver: RwLock<Solver>,
	pub leaky_bucket: RwLock<f32>,

	pub bvh: RwLock<Option<BVH<(PhysicsBodyId, GridId, SubGridId)>>>,

	pub ecs: RwLock<EntityComponentSystem>,

	pub resource_manager: RwLock<ResourceManager>,

	pub task_queue: TaskQueue,
	pub async_task_priority_queue: AsyncTaskPriorityQueue,
	pub async_task_priority_queue_threads: Vec<tokio::task::JoinHandle<()>>,

	pub world_gpu_data: RwLock<WorldGpuData>,
}

impl World {
	pub fn new(device: &Device) -> Self {
		let async_task_priority_queue = Arc::new(PriorityQueue::<PriorityTask>::new());
		let async_task_priority_queue_threads = (0..8).map(|_| {
			let async_task_priority_queue = async_task_priority_queue.clone();
			tokio::spawn(async move {
				loop {
					let task = async_task_priority_queue.pop().await;
					task.run().await;
				}
			})
		}).collect();

		Self {
			physics_bodies: RwLock::new(SparseSet::new()),
			grid_manager: RwLock::new(GridManager::new()),
			next_physics_body_id: RwLock::new(PhysicsBodyId(0)),
			next_grid_id: RwLock::new(GridId(0)),
			next_sub_grid_id: RwLock::new(SubGridId(0)),
			voxel_tracker: RwLock::new(VoxelTracker::new()),

			constraints: RwLock::new(HashMap::new()),
			impulses: RwLock::new(SparseSet::new()),
			solver: RwLock::new(Solver::new()),
			leaky_bucket: RwLock::new(0.0),

			bvh: RwLock::new(None),

			ecs: RwLock::new(EntityComponentSystem::new()),

			resource_manager: RwLock::new(ResourceManager::new()),

			task_queue: TaskQueue::new(Mutex::new(VecDeque::new())),
			async_task_priority_queue,
			async_task_priority_queue_threads,

			world_gpu_data: RwLock::new(WorldGpuData::new(device).unwrap())
		}
	}

	// called at any rate (faster is better and should be at least once per frame)
	pub fn update(&self, dt: f32) {
		let _zone = span!("World Update");
		// if !self.debug_enables.freeze_gpu_grids {
			let _zone = span!("Do tasks");
			while let Some(task) = { self.task_queue.lock().pop_front() } {
				task.run(self);
			}
		// }

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
			*self.bvh.write() = None; // have to clear afer cleanup
		}
		let leaky_bucket = &mut self.leaky_bucket.write();
		leaky_bucket.add_assign(dt);
		let time_step = 1.0 / 100.0;
		let current_time = Instant::now();
		while leaky_bucket.ge(&time_step) {
			self.physics_update(time_step);
			leaky_bucket.sub_assign(time_step);
			let elapsed = current_time.elapsed().as_secs_f32(); // timeout should maybe be removed at some point
			if elapsed > 1.0 / 60.0 {
				leaky_bucket.set_zero();
			}
		}
	}

	fn physics_update(&self, dt: f32) {
		// self.ecs.run_on_components_pair_mut::<Camera, ObjectPickup, _>(&mut |_entity_id, camera, object_pickup| {
		// 	if object_pickup.is_holding() {
		// 		object_pickup.hold_at_pos(&(camera.position + camera.forward() * 40.0), time_step, &mut self.physics_engine);
		// 	}
		// });
		// self.ecs.run_on_components_mut::<Orientator, _>(&mut |_entity_id, orientator| {
		// 	orientator.hold_at_orientation(&Quat::IDENTITY, time_step, &mut self.physics_engine);
		// });
		// let player_position = self.ecs.get_component::<Camera>(self.player_id).unwrap().position;
		// self.ecs.run_on_components_mut::<PlayerTracker, _>(&mut |_entity_id, player_tracker| {
		// 	player_tracker.track_pos(&player_position, time_step, &mut self.physics_engine);
		// });
		let _zone = span!("PhysicsEngine update physics");
		{
			let bvh = &get_bvh_macro!(self);
			self.solver.write().solve(
				&mut self.physics_bodies.write(),
				&self.grid_manager.read().grids(),
				&mut self.constraints.write(),
				&self.impulses.read(),
				dt,
				bvh
			);
			self.impulses.write().clear();
		}
		*self.bvh.write() = None;
	}

	pub fn add_physics_body(&self) -> PhysicsBodyId {
		let physics_body_uuid = ResourceUUID::generate();
		let resource_manager = &mut self.resource_manager.write();
		let physics_body_resource = resource_manager.create_resource_blank(
			physics_body_uuid.clone(),
			ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid.clone()) }
		).unwrap();
		let physics_body_id = self.next_physics_body_id.read().clone();
		self.next_physics_body_id.write().0 += 1;
		self.physics_bodies.write().insert(physics_body_id, PhysicsBody::new(physics_body_uuid, physics_body_id));
		match physics_body_resource.info_mut() {
			ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id)),
			_ => unreachable!()
		}
		physics_body_id
	}
	pub fn add_grid(&self, physics_body_id: PhysicsBodyId, pose: &Pose) -> Option<GridId> {
		let physics_body_uuid = self.physics_body(physics_body_id)?.uuid().clone();
		let grid_resource_uuid = ResourceUUID::generate();
		let resource_manager = &mut self.resource_manager.write();
		let grid_resource = resource_manager.create_resource_blank(
			grid_resource_uuid.clone(),
			ResourceInfoType::Grid { grid_resource: GridResource::new(grid_resource_uuid.clone(), physics_body_uuid) }
		).unwrap();
		let grid_id = self.grid_manager.write().add_grid(physics_body_id, pose, grid_resource_uuid);
		match grid_resource.info_mut() {
			ResourceInfoType::Grid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
			_ => unreachable!()
		}
		Some(grid_id)
	}
	pub fn remove_grid(&self, grid_id: GridId) -> bool {
		self.grid_manager.write().remove_grid(grid_id)
	}
	pub fn physics_body(&self, physics_body_id: PhysicsBodyId) -> Option<MappedRwLockReadGuard<'_, PhysicsBody>> {
		RwLockReadGuard::try_map(self.physics_bodies.read(), |physics_bodies| physics_bodies.get(&physics_body_id)).ok()
	}
	pub fn physics_body_mut(&self, physics_body_id: PhysicsBodyId) -> Option<MappedRwLockWriteGuard<'_, PhysicsBody>> {
		RwLockWriteGuard::try_map(self.physics_bodies.write(), |physics_bodies| physics_bodies.get_mut(&physics_body_id)).ok()
	}
	pub fn grid(&self, grid_id: GridId) -> Option<MappedRwLockReadGuard<'_, Grid>> {
		RwLockReadGuard::try_map(self.grid_manager.read(), |grid_manager| grid_manager.grid(grid_id)).ok()
	}
	pub fn grid_mut(&self, grid_id: GridId) -> Option<MappedRwLockWriteGuard<'_, Grid>> {
		RwLockWriteGuard::try_map(self.grid_manager.write(), |grid_manager| grid_manager.grid_mut(grid_id)).ok()
	}

	pub fn apply_central_impulse(&self, physics_body_id: PhysicsBodyId, impluse: &Vec3) {
		self.impulses.write().entry(physics_body_id).or_default().push(Impulse::CentralImpulse { central_impluse: *impluse });
	}
	pub fn apply_rotational_impulse(&self, physics_body_id: PhysicsBodyId, rotational_impluse: &Vec3) {
		self.impulses.write().entry(physics_body_id).or_default().push(Impulse::RotationalImpulse { rotational_impluse: *rotational_impluse });
	}
	pub fn apply_impulse(&mut self, physics_body_id: PhysicsBodyId, impluse_pos: &Vec3, impluse: &Vec3) {
		self.impulses.write().entry(physics_body_id).or_default().push(Impulse::Impulse { impluse: *impluse, impluse_pos: *impluse_pos });
	}

	pub fn create_ball_joint_constraint(&self, physics_body_id_1: PhysicsBodyId, body_1_attachment: &Pose, physics_body_id_2: PhysicsBodyId, body_2_attachment: &Pose) {
		self.constraints.write().insert(
			if physics_body_id_1.0 < physics_body_id_2.0 { (physics_body_id_1, physics_body_id_2) } else { (physics_body_id_2, physics_body_id_1) },
			BallJointConstraint::new(body_1_attachment, body_2_attachment, f32::INFINITY, 0.0)
		);
	}

	pub fn create_ball_joint_spring_constraint(&self, physics_body_id_1: PhysicsBodyId, body_1_attachment: &Pose, physics_body_id_2: PhysicsBodyId, body_2_attachment: &Pose, stiffness : f32) {
		self.constraints.write().insert(
			if physics_body_id_1.0 < physics_body_id_2.0 { (physics_body_id_1, physics_body_id_2) } else { (physics_body_id_2, physics_body_id_1) },
			BallJointConstraint::new(body_1_attachment, body_2_attachment, stiffness, 0.0)
		);
	}

	pub fn raycast(&self, pose: &Pose, max_length: Option<f32>) -> Option<(PhysicsBodyId, GridId, IVec3, I8Vec3, f32)> {
		let mut best_hit: Option<(PhysicsBodyId, GridId, IVec3, I8Vec3, f32)> = None;
		for ((body_id, grid_id, sub_grid_id), bvh_distance) in self.get_bvh().raycast(pose, max_length) {
			if best_hit.is_some() && bvh_distance > best_hit.unwrap().4 { break; }
			let physics_body = &self.physics_body(body_id).unwrap();
			let grid = self.grid(grid_id).unwrap();
			// let sub_grid = grid.sub_grid(sub_grid_id).unwrap();
			// if let Some((hit_pos, hit_normal, grid_distance)) = sub_grid.get_voxels().get_voxels().raycast(
			// 	&(&Pose::from_translation(-grid.sub_grid_pos_to_grid_pos(&sub_grid.sub_grid_ipos()).as_vec3()) * grid.pose.inverse() * physics_body.pose.inverse() * Pose::new(
			// 		pose.translation + pose.rotation * Vec3::Z * bvh_distance, pose.rotation)),
			// 	max_length.map(|max_length| max_length - bvh_distance),
			// 	// &(&physics_body.pose * grid.pose * Pose::from_translation(grid.sub_grid_pos_to_grid_pos(&sub_grid.sub_grid_pos().as_ivec3()).as_vec3()))
			// ) {
			// 	if best_hit.is_none() || grid_distance + bvh_distance < best_hit.unwrap().4 {
			// 		best_hit = Some((body_id, grid_id, hit_pos.as_ivec3() + grid.sub_grid_pos_to_grid_pos(&sub_grid.sub_grid_ipos()), hit_normal, grid_distance + bvh_distance));
			// 	}
			// }
		}
		best_hit
	}

	pub fn get_bvh(&'_ self) -> MappedRwLockReadGuard<'_, BVH<(PhysicsBodyId, GridId, SubGridId)>> {
		if self.bvh.read().is_none() {
			let mut bounds = vec![];
			{
				let _zone = span!("Collect aabb");
				for (physics_body_id, physics_body) in self.physics_bodies.read().iter() {
					for grid_id in physics_body.grids() {
						let grid = self.grid(*grid_id).unwrap();
						for (sub_grid_id, sub_grid) in grid.sub_grids() {
							if let Some(bound) = sub_grid.local_aabb(&(physics_body.pose.rotation * grid.pose().rotation)) {
								bounds.push(((*physics_body_id, *grid_id, *sub_grid_id), bound));
							}
						}
					}
				}
			}
			*self.bvh.write() = Some(BVH::new(bounds));
		}
		RwLockReadGuard::map(self.bvh.read(), |bvh| bvh.as_ref().unwrap())
	}
	pub fn start_tracking(&self, body_id: PhysicsBodyId, grid_id: GridId, voxel_pos: IVec3) -> TrackedVoxelId {
		self.voxel_tracker.write().start_tracking(body_id, grid_id, voxel_pos)
	}
	pub fn stop_tracking(&self, tracked_voxel_id: TrackedVoxelId) {
		self.voxel_tracker.write().stop_tracking(tracked_voxel_id);
	}
	pub fn get_tracked_voxel(&self, tracked_voxel_id: TrackedVoxelId) -> Option<(PhysicsBodyId, GridId, IVec3)> {
		let tracked_voxel = self.voxel_tracker.read().get_tracked_voxel(tracked_voxel_id)?;
		Some((tracked_voxel.body_id, tracked_voxel.grid_id, tracked_voxel.voxel_pos))
	}

	pub fn get_rendering_buffers(
		&self,
		id_to_hit_count: HashMap<(PhysicsBodyId, GridId, SubGridId), u32>,
		view_frustum: &ViewFrustum,
		player_camera: Pose
	) -> RwLockReadGuard<'_, WorldGpuData> {
		self.world_gpu_data.read()
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


