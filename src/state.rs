#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
use async_priority_queue::PriorityQueue;
use uuid::Uuid;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;
use std::{collections::{HashMap, VecDeque}, f32, pin::Pin, sync::{Arc, Mutex}};

use glam::{IVec3, Quat, Vec3, Vec4};
use tracy_client::span;
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::{CursorGrabMode, Window}};

use crate::{entity_component_system, pose::Pose, render::renderer::Renderer, resource_manager::{ResourceInfoType, ResourceManager, ResourceUUID}};
use crate::{physics::{bvh::BVH, physics_body::{PhysicsBodyGridId, PhysicsBodyId, SubGridId}}, physics_body_resource::{PhysicsBodyGridResource, PhysicsBodyResource}};
use crate::player::{camera::{Camera, CameraController}, player_input::PlayerInput, object_pickup::ObjectPickup, player_tracker::PlayerTracker, orientator::Orientator};
use crate::physics::{physics_body::PhysicsBody, physics_engine::PhysicsEngine};
use crate::audio::audio_engine::{AudioEngine, ListenerState, SoundEffect};
use crate::voxels::{Voxel, self};
use crate::debug_draw;

const BLOCK_PLACE_SOUND_INTERVAL_SECONDS: f32 = 1.0 / (18.0 * 2.0);
const BLOCK_BREAK_SOUND_INTERVAL_SECONDS: f32 = 1.0 / (14.0 * 2.0);

pub struct Task {
	task_func: Box<dyn FnOnce(&mut State) + Send + 'static>,
}

impl Task {
	pub fn new<F: FnOnce(&mut State) + Send + 'static>(function: F) -> Self {
		Self {
			task_func: Box::new(function),
		}
	}
	pub fn run(self, state: &mut State) {
		(self.task_func)(state);
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

pub struct State {
	pub renderer: Renderer,
	pub mouse_captured: bool,
	pub audio_engine: AudioEngine,
	pub physics_engine: PhysicsEngine,
	pub ecs: entity_component_system::EntityComponentSystem,
	pub resource_manager: ResourceManager,
	pub player_id: u32,
	pub leaky_bucket: f32,
	pub raycast_pose: Option<Pose>,
	pub place_sound_cooldown: f32,
	pub break_sound_cooldown: f32,
	pub task_queue: TaskQueue,
	pub async_task_priority_queue: AsyncTaskPriorityQueue,
	pub async_task_priority_queue_threads: Vec<tokio::task::JoinHandle<()>>,
	pub freeze_gpu_grids: bool,
}

impl State {
	pub fn handle_key(&mut self, _event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
		if code == KeyCode::Escape && is_pressed {
			self.set_mouse_captured(false);
		} else {
			self.ecs.run_on_single_component_mut::<PlayerInput, _>(self.player_id, |_entity_id, player_input|
				player_input.set_state(code, is_pressed)
			);
		}
	}

	pub fn set_mouse_captured(&mut self, captured: bool) {
		self.mouse_captured = captured;
		let window = &self.renderer.window;
		if captured {
			let success = window.set_cursor_grab(CursorGrabMode::Confined).or_else(|_| window.set_cursor_grab(CursorGrabMode::Locked));
			if success.is_err() {
				self.mouse_captured = false;
				eprintln!("Failed to grab cursor");
			}
		} else {
			let _ = window.set_cursor_grab(CursorGrabMode::None);
		}
		window.set_cursor_visible(!self.mouse_captured);
	}

	pub fn update(&mut self, dt: f32) {
		let _zone = span!("State Update");
		if !self.freeze_gpu_grids {
			let _zone = span!("Do tasks");
			while let Some(task) = { self.task_queue.lock().unwrap().pop_front() } {
				task.run(self);
			}
		}
		self.place_sound_cooldown = (self.place_sound_cooldown - dt).max(0.0);
		self.break_sound_cooldown = (self.break_sound_cooldown - dt).max(0.0);
		self.ecs.run_on_components_tripl_mut::<PlayerInput, CameraController, Camera, _>(&mut |_entity_id, player_input, camera_controller, camera|
			CameraController::update_camera(camera_controller, camera, player_input, dt)
		);

		if let Some(camera) = self.ecs.get_component::<Camera>(self.player_id) {
			let (forward, right, _) = camera.forward_right_up();
			self.audio_engine.set_listener(ListenerState {
				position: camera.position,
				forward,
				right,
			});
			if let Some(player_input) = self.ecs.get_component::<PlayerInput>(self.player_id) {
				if player_input.key(KeyCode::KeyM).just_pressed {
					self.audio_engine.play_sound(SoundEffect::DebugBeep, camera.position + forward * 20.0);
				}
			}
		}

		self.ecs.run_on_components_mut::<Orientator, _>(&mut |_entity_id, orientator| {
			orientator.hold_at_orientation(&Quat::IDENTITY, &mut self.physics_engine);
		});
		let player_position = self.ecs.get_component::<Camera>(self.player_id).unwrap().position;
		self.ecs.run_on_components_mut::<PlayerTracker, _>(&mut |_entity_id, player_tracker| {
			player_tracker.track_pos(&player_position, &mut self.physics_engine);
		});
		self.ecs.run_on_components_tripl_mut::<PlayerInput, Camera, ObjectPickup, _>(&mut |_entity_id, player_input, camera, object_pickup| {
			let ray_start = if self.raycast_pose.is_none() {
				Pose::new(camera.position, Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch))
			} else {
				self.raycast_pose.unwrap()
			};
			if player_input.key(KeyCode::KeyH).just_pressed {
				if self.raycast_pose.is_none() {
					self.raycast_pose = Some(Pose::new(camera.position, Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch)));
				} else {
					self.raycast_pose = None;
				}
			}
			let started_holding = object_pickup.is_holding();
			if let Some((body_id, grid_id, hit_pos, hit_normal, distance)) = self.physics_engine.raycast(&ray_start, None) {
				let physics_body = self.physics_engine.physics_body(body_id).unwrap();
				let grid = physics_body.grid(grid_id).unwrap();
				let globle_hit_normal = physics_body.pose.rotation * grid.pose.rotation * hit_normal.as_vec3();
				let globle_hit_pos = ray_start.translation + ray_start.rotation * Vec3::Z * distance;
				let globle_hit_pos_snap = physics_body.pose * grid.pose * hit_pos.as_vec3();
				let place_voxel_pos = hit_pos + hit_normal.as_ivec3();
				let break_voxel_pos = hit_pos;
				let place_sound_pos = physics_body.pose * grid.pose * (place_voxel_pos.as_vec3() + Vec3::splat(0.5));
				let break_sound_pos = physics_body.pose * grid.pose * (break_voxel_pos.as_vec3() + Vec3::splat(0.5));
				debug_draw::line(globle_hit_pos, globle_hit_pos + globle_hit_normal, &Vec4::new(1.0, 0.0, 0.0, 1.0));
				debug_draw::rectangular_prism(&Pose::new(globle_hit_pos_snap, physics_body.pose.rotation * grid.pose.rotation), Vec3::splat(1.0), &Vec4::new(1.0, 0.0, 1.0, 0.1), true);
				if player_input.key(KeyCode::Space).just_pressed || player_input.key(KeyCode::KeyC).is_pressed {
					self.physics_engine.physics_body_mut(body_id).unwrap().grid_mut(grid_id).unwrap().add_voxel(place_voxel_pos, Voxel{ color: [100, 100, 100, 1], mass: 100 }, &mut self.resource_manager);
					if self.place_sound_cooldown <= 0.0 {
						self.audio_engine.play_sound(SoundEffect::BlockPlace, place_sound_pos);
						self.place_sound_cooldown = BLOCK_PLACE_SOUND_INTERVAL_SECONDS;
					}
				}
				if player_input.key(KeyCode::KeyX).just_pressed || player_input.key(KeyCode::KeyZ).is_pressed {
					self.physics_engine.physics_body_mut(body_id).unwrap().grid_mut(grid_id).unwrap().remove_voxel(&break_voxel_pos);
					if self.break_sound_cooldown <= 0.0 {
						self.audio_engine.play_sound(SoundEffect::BlockBreak, break_sound_pos);
						self.break_sound_cooldown = BLOCK_BREAK_SOUND_INTERVAL_SECONDS;
					}
				}
				if player_input.key(KeyCode::KeyR).just_pressed {
					self.physics_engine.apply_impulse(
						self.physics_engine.physics_body(body_id).unwrap().id(),
						&globle_hit_pos,
						&(ray_start.rotation * Vec3::Z * 1600000.0)
					);
				}
				if player_input.key(KeyCode::KeyF).just_pressed {
					if !started_holding {
						let body = self.physics_engine.physics_body(body_id).unwrap();
						if !body.is_static {
							object_pickup.set(body.id());
						}
					}
				}
			}
			if player_input.key(KeyCode::KeyF).just_pressed {
				if started_holding {
					object_pickup.reset();
				}
			}
			if player_input.key(KeyCode::KeyT).just_pressed {
				self.freeze_gpu_grids ^= true;
				// if self.fixed_view_frustum.is_none() {
				// 	self.fixed_view_frustum = Some(camera.frustum());
				// } else {
				// 	self.fixed_view_frustum = None;
				// }
			}
		});
		self.leaky_bucket += dt;
		let time_step = 1.0 / 100.0;
		let current_time = Instant::now();
		while self.leaky_bucket >= time_step {
			self.ecs.run_on_components_pair_mut::<Camera, ObjectPickup, _>(&mut |_entity_id, camera, object_pickup| {
				if object_pickup.is_holding() {
					object_pickup.hold_at_pos(&(camera.position + camera.forward() * 40.0), &mut self.physics_engine);
				}
			});
			self.physics_engine.update(time_step);
			self.leaky_bucket -= time_step;
			let elapsed = current_time.elapsed().as_secs_f32();
			if elapsed > 1.0 / 60.0 {
				self.leaky_bucket = 0.0;
			}
		}
		self.ecs.run_on_single_component_mut::<PlayerInput, _>(self.player_id, |_entity_id, player_input|
			player_input.end_frame()
		);
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
		self.ecs.run_on_components_mut::<Camera, _>(&mut |_entity_id, camera| {
			camera.aspect = self.renderer.config.width as f32 / self.renderer.config.height as f32;
		})
	}

	fn make_ball(resource_manager: &mut ResourceManager, physics_body: &mut PhysicsBody, radius: i32) {
		{
			let grid_uuid = ResourceUUID(Uuid::new_v4());
			let grid_resource = resource_manager.create_resource_blank(
				grid_uuid.clone(),
				ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body.uuid().clone()) }
			).unwrap();
			let grid_id = physics_body.add_grid(grid_uuid, Pose::new(Vec3::new(-0.5, -0.5, -0.5), Quat::IDENTITY));
			match grid_resource.info_mut() {
				ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
				_ => unreachable!()
			}
			let grid = physics_body.grid_mut(grid_id).unwrap();

			for x in -radius..radius + 1 {
				for y in -0..radius + 1 {
					for z in -radius..radius + 1 {
						if IVec3::new(x, y, z).length_squared() as f32 <= (radius as f32 - 0.5).powf(2.0) {
							grid.add_voxel(
								IVec3::new(x, y, z),
								voxels::Voxel { color: [(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255], mass: 100 },
								resource_manager
							);
						}
					}
				}
			}
		}
		{
			let grid_uuid = ResourceUUID(Uuid::new_v4());
			let grid_resource = resource_manager.create_resource_blank(
				grid_uuid.clone(),
				ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body.uuid().clone()) }
			).unwrap();
			let grid_id = physics_body.add_grid(grid_uuid, Pose::new(Vec3::new(-0.707, -0.5, 0.0), Quat::from_rotation_y(f32::consts::PI / 4.0)));
			match grid_resource.info_mut() {
				ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
				_ => unreachable!()
			}
			let grid = physics_body.grid_mut(grid_id).unwrap();

			for x in -radius..radius + 1 {
				for y in -radius..0 {
					for z in -radius..radius + 1 {
						if IVec3::new(x, y, z).length_squared() as f32 <= (radius as f32 - 0.5).powf(2.0) {
							grid.add_voxel(
								IVec3::new(x, y, z),
								voxels::Voxel { color: [(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255], mass: 100 },
								resource_manager
							);
						}
					}
				}
			}
		}
	}

	fn make_smooth_ball(resource_manager: &mut ResourceManager, physics_body: &mut PhysicsBody, radius: i32){
		let face_resolution = (radius * 2).max(1);
		let sphere_radius = radius as f32 - 0.5;
		let voxel_center = Vec3::splat(0.5);

		let cube_to_sphere = |p: Vec3| -> Vec3 {
			Vec3::new(
				p.x * (1.0 - (p.y * p.y) * 0.5 - (p.z * p.z) * 0.5 + (p.y * p.y) * (p.z * p.z) / 3.0).sqrt(),
				p.y * (1.0 - (p.z * p.z) * 0.5 - (p.x * p.x) * 0.5 + (p.z * p.z) * (p.x * p.x) / 3.0).sqrt(),
				p.z * (1.0 - (p.x * p.x) * 0.5 - (p.y * p.y) * 0.5 + (p.x * p.x) * (p.y * p.y) / 3.0).sqrt(),
			)
		};

		let face_point = |face: usize, u: f32, v: f32| -> Vec3 {
			match face {
				0 => Vec3::new(1.0, v, u),
				1 => Vec3::new(-1.0, v, -u),
				2 => Vec3::new(u, 1.0, v),
				3 => Vec3::new(u, -1.0, -v),
				4 => Vec3::new(u, v, 1.0),
				5 => Vec3::new(-u, v, -1.0),
				_ => unreachable!(),
			}
		};

		let mut placed_centers = std::collections::HashSet::new();
		for face in 0..6 {
			for y in 0..face_resolution {
				for x in 0..face_resolution {
					let u = ((x as f32 + 0.5) / face_resolution as f32) * 2.0 - 1.0;
					let v = ((y as f32 + 0.5) / face_resolution as f32) * 2.0 - 1.0;
					let cube_pos = face_point(face, u, v);
					let normal = cube_to_sphere(cube_pos).normalize_or_zero();
					if normal == Vec3::ZERO {
						continue;
					}

					let center = normal * sphere_radius;
					let center_key = (
						(center.x * 1000.0).round() as i32,
						(center.y * 1000.0).round() as i32,
						(center.z * 1000.0).round() as i32,
					);
					if !placed_centers.insert(center_key) {
						continue;
					}

					let inward = -normal;
					let rotation = Quat::from_rotation_arc(Vec3::Z, inward);
					let translation = center - rotation * voxel_center;
					let grid_uuid = ResourceUUID(Uuid::new_v4());
					resource_manager.create_resource(grid_uuid.clone(), false, true, vec![], ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body.uuid().clone()) });
					let grid_id = physics_body.add_grid(grid_uuid, Pose::new(translation, rotation));
					let grid = physics_body.grid_mut(grid_id).unwrap();
					grid.add_voxel(
						IVec3::ZERO,
						voxels::Voxel {
							color: [
								((normal.x * 0.5 + 0.5) * 255.0) as u8,
								((normal.y * 0.5 + 0.5) * 255.0) as u8,
								((normal.z * 0.5 + 0.5) * 255.0) as u8,
								1,
							],
							mass: 100,
						},
						resource_manager
					);
				}
			}
		}
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
		let renderer = Renderer::new(window).await?;
		let mut physics_engine = PhysicsEngine::new();
		let mut ecs = entity_component_system::EntityComponentSystem::new();

		// create player entity
		let player_id = ecs.add_entity();
		ecs.add_component_to_entity(player_id, PlayerInput::new());
		ecs.add_component_to_entity(player_id, Camera {
			position: Vec3::new(-30.0, 205.0, 0.0),
			yaw: f32::consts::PI / 2.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			#[cfg(not(target_arch = "wasm32"))]
			zfar: 5000.0,
			#[cfg(target_arch = "wasm32")]
			zfar: 900.0,
		});
		ecs.add_component_to_entity(player_id, CameraController::new(30.0, 1.5, 0.0015));
		ecs.add_component_to_entity(player_id, ObjectPickup::new());

		let mut resource_manager = ResourceManager::new();

		match crate::resources::load_binary("Church_Of_St_Sophia.vox").await {
			Ok(bytes) => {
				match dot_vox::load_bytes(&bytes) {
					Ok(dot_vox_data) => {
						let physics_body_uuid = ResourceUUID(Uuid::new_v4());
						let physics_body_resource = resource_manager.create_resource_blank(
							physics_body_uuid.clone(),
							ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid.clone()) }
						).unwrap();
						let physics_body_id = physics_engine.add_physics_body(physics_body_uuid.clone());
						match physics_body_resource.info_mut() {
							ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id)),
							_ => unreachable!()
						};
						let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
						physics_body.pose.translation.y -= 350.0;
						physics_body.is_static = true;
						let grid_uuid = ResourceUUID(Uuid::new_v4());
						let grid_resource = resource_manager.create_resource_blank(
							grid_uuid.clone(),
							ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body_uuid) }
						).unwrap();
						let grid_id = physics_body.add_grid(grid_uuid, Pose::ZERO);
						match grid_resource.info_mut() {
							ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
							_ => unreachable!()
						};
						let grid = physics_body.grid_mut(grid_id).unwrap();
						let mut stack = vec![(0, Pose::ZERO, IVec3::new(1, 1, -1))];
						while let Some((scene_id, pose, flip)) = stack.pop() {
							match &dot_vox_data.scenes[scene_id as usize] {
								dot_vox::SceneNode::Transform { attributes: _, frames, child, layer_id: _ } => {
									if let Some(frame) = frames.first() {
										let pos = frame.position().unwrap_or(dot_vox::Position{x: 0, y: 0, z: 0});
										let (rot, flip_vec) = frame.orientation().and_then(|quat| {
											let (q, v) = quat.to_quat_scale();
											let q = Quat::from_array(q);
											Some((Quat::from_xyzw(q.x, q.z, -q.y, q.w), Vec3::from_array(v).as_ivec3()))
										}).unwrap_or((Quat::IDENTITY, IVec3::ONE));
										stack.push((
											*child,
											Pose::new(pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32), pose.rotation * rot),
											flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y)
										));
									}
								},
								dot_vox::SceneNode::Group { attributes: _, children } => {
									for child in children {
										stack.push((*child, pose, flip));
									}
								},
								dot_vox::SceneNode::Shape { attributes: _, models } => {
									for shape_model in models {
										if let Some(model) = dot_vox_data.models.get(shape_model.model_id as usize) {
											let size = Vec3::new(model.size.x as f32, model.size.z as f32, model.size.y as f32);
											let half = (size / 2.0).floor();
											for voxel in &model.voxels {
												grid.add_voxel((
													pose * Pose::from_translation(-half * flip.as_vec3()) * (
														IVec3::new(
															voxel.x as i32,
															voxel.z as i32,
															voxel.y as i32,
														) * flip + flip.min(IVec3::ZERO)
													).as_vec3()
												).as_ivec3(), voxels::Voxel {
													color: [
														dot_vox_data.palette[voxel.i as usize].r,
														dot_vox_data.palette[voxel.i as usize].g,
														dot_vox_data.palette[voxel.i as usize].b,
														dot_vox_data.palette[voxel.i as usize].a,
													],
													mass: 100,
												}, &mut resource_manager);
											}
										}
									}
								},
							}
						}
					},
					Err(err) => println!("dot_vox error: {err}"),
				};
			},
			Err(err) => println!("load_string error: {err}"),
		}

		// ------------------------------ Static Box ------------------------------
		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let grid = physics_body.grid_mut(grid_id).unwrap();
		// 	for x in -2..3 {
		// 		for y in -14..-13 {
		// 			for z in -2..3 {
		// 				if (x as i32).abs() == 2 || (y as i32).abs() == 14 || (z as i32).abs() == 2 {
		// 					grid.add_voxel(IVec3::new(x, y+7, z), voxels::Voxel{ color: [0, 160, 0, 255], mass: 100 });
		// 				}
		// 			}
		// 		}
		// 	}
		// 	physics_body.pose.translation.z -= 8.0;
		// 	physics_body.pose.translation.y = 2.0;
		// 	physics_body.is_static = true;
		// }
		// ------------------------------ Cube Stack ------------------------------
		// {
		// 	for _x in 0..1 {
		// 		for y in 0..10 {
		// 			for _z in 0..1 {
		// 				let physics_body_id = physics_engine.add_physics_body();
		// 				let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 				let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 				let grid = physics_body.grid_mut(grid_id).unwrap();
		// 				for x_v in -1..2 {
		// 					for y_v in -1..2 {
		// 						for z_v in -1..2 {
		// 							grid.add_voxel(IVec3::new(x_v, y_v, z_v), voxels::Voxel{ color: [(x_v + 2) as u8 * 20, y * 4, (y_v + 2) as u8 * 20, 255], mass: 500 });
		// 						}
		// 					}
		// 				}
		// 				// physics_body.position.x += y as f32 * 0.01;
		// 				physics_body.pose.translation.y = y as f32 * 4.0 + 2.0;
		// 				physics_body.pose.translation.y = 0.0;
		// 				physics_body.pose.translation.z -= 8.0;
		// 				// physics_body.orientation = Quat::from_rotation_y(y as f32);
		// 			}
		// 		}
		// 	}
		// }
		// ------------------------------
		// {
		// 	physics_bodies.push(physics_body::PhysicsBody::new());
		// 	let physics_body = physics_bodies.last_mut().unwrap();
		// 	for x in -20..21 {
		// 		for z in -20..21 {
		// 			physics_body.add_voxel(IVec3::new(x, 0, z), voxels::Voxel{ color: [0.0, (z + 20) as f32 / 40.0, (x + 20) as f32 / 40.0, 1.0], mass: 1.0 });
		// 		}
		// 	}
		// 	physics_body.position.y -= 4.0;
		// 	physics_body.orientation = Quat::from_rotation_z(0.2);
		// 	physics_body.is_static = true;
		// }

		// {
		// 	for _x in 0..1 {
		// 		for y in 0..8 {
		// 			for _z in 0..1 {
		// 				physics_bodies.push(physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, (y % 4) as f32 / 4.0, 0.5, 1.0], mass: 1.0 });
		// 				// physics_body.position.x += y as f32 * 0.01;
		// 				physics_body.position.z += y as f32 * 4.0;
		// 				physics_body.position.z -= 8.0;
		// 				// physics_body.orientation = Quat::from_rotation_y(y as f32);
		// 			}
		// 		}
		// 	}
		// }
		// ------------------------------ Ramp ------------------------------
		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let grid = physics_body.grid_mut(grid_id).unwrap();
		// 	for x in -15..16 {
		// 		for y in 0..20 {
		// 			grid.add_voxel(I16Vec3::new(x, y, -10), voxels::Voxel{ color: [(x * 32 % 255 * 0) as u8, 0, (y * 32 % 255) as u8, 1], mass: 100 });
		// 		}
		// 	}
		// 	for x in -15..16 {
		// 		for z in -10..200 {
		// 			grid.add_voxel(I16Vec3::new(x, -(z + 10) / 3, z), voxels::Voxel{ color: [(x * 32 % 255 * 0) as u8, 0, (z * 32 % 255) as u8, 1], mass: 100 });
		// 		}
		// 	}
		// 	for y in 0..15 {
		// 		for z in -10..200 {
		// 			grid.add_voxel(I16Vec3::new(-16, -(z + 10) / 3 + y, z), voxels::Voxel{ color: [0, 0, (z * 32 % 255) as u8, 1], mass: 100 });
		// 			grid.add_voxel(I16Vec3::new(16, -(z + 10) / 3 + y, z), voxels::Voxel{ color: [0, 0, (z * 32 % 255) as u8, 1], mass: 100 });
		// 		}
		// 	}
		// 	physics_body.is_static = true;
		// 	physics_body.pose.translation.y = 2.0;
		// }
		// ------------------------------ Ball ------------------------------

		// {
		// 	let r = 6;
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	physics_body.pose.translation.y = (0 as f32) * (r as f32) * 2.0 + 7.0 + 40.0;
		// 	physics_body.pose.translation.z += (0 as f32) * (r as f32) * 2.0 + 3.0 as f32;
		// 	physics_body.pose.translation.x += (0 as f32) * (r as f32) * 2.0;
		// 	State::make_ball(physics_body, r);
		// }
		// for x in -1..2 {
		// 	for y in -1..0 {
		// 		for z in -1..2 {
		// 			let r = 6;
		// 			let physics_body_id = physics_engine.add_physics_body();
		// 			let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 			physics_body.pose.translation.y = (y as f32) * (r as f32) * 2.0 + 7.0 + 75.0;
		// 			physics_body.pose.translation.z += (z as f32) * (r as f32) * 2.0 + y as f32;
		// 			physics_body.pose.translation.x += (x as f32) * (r as f32) * 2.0;
		// 			State::make_ball(physics_body, r);
		// 		}
		// 	}
		// }
		{
			let r = 5;
			let physics_body_uuid_main = ResourceUUID(Uuid::new_v4());
			let physics_body_resource_main = resource_manager.create_resource_blank(
				physics_body_uuid_main.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid_main.clone()) }
			).unwrap();
			let physics_body_id_main = physics_engine.add_physics_body(physics_body_uuid_main);
			{
				match physics_body_resource_main.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id_main)),
					_ => unreachable!()
				};
				let physics_body = physics_engine.physics_body_mut(physics_body_id_main).unwrap();
				physics_body.pose.translation.x += 0.0;
				physics_body.pose.translation.y = 80.0;
				physics_body.pose.translation.z += 40.0 - 60.0;
				State::make_ball(&mut resource_manager, physics_body, 2);
			}
			let physics_body_uuid_1 = ResourceUUID(Uuid::new_v4());
			let physics_body_resource_1 = resource_manager.create_resource_blank(
				physics_body_uuid_1.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid_1.clone()) }
			).unwrap();
			let physics_body_id_1 = physics_engine.add_physics_body(physics_body_uuid_1);
			{
				match physics_body_resource_1.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id_1)),
					_ => unreachable!()
				};
				let physics_body = physics_engine.physics_body_mut(physics_body_id_1).unwrap();
				physics_body.pose.translation.x += 0.0;
				physics_body.pose.translation.y = 80.0;
				physics_body.pose.translation.z += 50.0 - 60.0;
				State::make_ball(&mut resource_manager, physics_body, r);
			}
			let physics_body_uuid_2 = ResourceUUID(Uuid::new_v4());
			let physics_body_resource_2 = resource_manager.create_resource_blank(
				physics_body_uuid_2.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid_2.clone()) }
			).unwrap();
			let physics_body_id_2 = physics_engine.add_physics_body(physics_body_uuid_2);
			{
				match physics_body_resource_2.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id_2)),
					_ => unreachable!()
				};
				let physics_body = physics_engine.physics_body_mut(physics_body_id_2).unwrap();
				physics_body.pose.translation.x += 0.0;
				physics_body.pose.translation.y = 80.0;
				physics_body.pose.translation.z += 30.0 - 60.0;
				State::make_ball(&mut resource_manager, physics_body, r);
			}
			let physics_body_uuid_3 = ResourceUUID(Uuid::new_v4());
			let physics_body_resource_3 = resource_manager.create_resource_blank(
				physics_body_uuid_3.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid_3.clone()) }
			).unwrap();
			let physics_body_id_3 = physics_engine.add_physics_body(physics_body_uuid_3);
			{
				match physics_body_resource_3.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id_3)),
					_ => unreachable!()
				};
				let physics_body = physics_engine.physics_body_mut(physics_body_id_3).unwrap();
				physics_body.pose.translation.x += 10.0;
				physics_body.pose.translation.y = 80.0;
				physics_body.pose.translation.z += 40.0 - 60.0;
				State::make_ball(&mut resource_manager, physics_body, r);
			}
			let physics_body_uuid_4 = ResourceUUID(Uuid::new_v4());
			let physics_body_resource_4 = resource_manager.create_resource_blank(
				physics_body_uuid_4.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid_4.clone()) }
			).unwrap();
			let physics_body_id_4 = physics_engine.add_physics_body(physics_body_uuid_4);
			{
				match physics_body_resource_4.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id_4)),
					_ => unreachable!()
				};
				let physics_body = physics_engine.physics_body_mut(physics_body_id_4).unwrap();
				physics_body.pose.translation.x += -10.0;
				physics_body.pose.translation.y = 80.0;
				physics_body.pose.translation.z += 40.0 - 60.0;
				State::make_ball(&mut resource_manager, physics_body, r);
			}
			physics_engine.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(0.0, 0.0, 10.0)), physics_body_id_1, &Pose::ZERO);
			physics_engine.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(0.0, 0.0, -10.0)), physics_body_id_2, &Pose::ZERO);
			physics_engine.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(10.0, 0.0, 0.0)), physics_body_id_3, &Pose::ZERO);
			physics_engine.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(-10.0, 0.0, 0.0)), physics_body_id_4, &Pose::ZERO);
		}
		// {
		// 	let mut bodies = HashMap::new();
		// 	let size_w = 15;
		// 	let size_l = 15;
		// 	for x in 0..size_w {
		// 		for z in 0..size_l {
		// 			let physics_body_id = physics_engine.add_physics_body();
		// 			let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 			let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 			let grid = physics_body.grid_mut(grid_id).unwrap();
		// 			grid.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [(x * 255 / size_w) as u8, 0, (z * 255 / size_l) as u8, 255], mass: 50 });
		// 			physics_body.pose.translation.x += x as f32 * 1.5;
		// 			physics_body.pose.translation.y = 60.0;
		// 			physics_body.pose.translation.z += z as f32 * 1.5;
		// 			bodies.insert((x, z), physics_body_id);
		// 		}
		// 	}
		// 	for x in 0..size_w {
		// 		for z in 0..size_l {
		// 			let body_id = bodies.get(&(x, z)).unwrap();
		// 			if let Some(body_x_id) = bodies.get(&(x + 1, z)) {
		// 				physics_engine.create_ball_joint_spring_constraint(*body_id, &Pose::from_translation(Vec3::new(0.75, 0.0, 0.0)), *body_x_id, &Pose::from_translation(Vec3::new(-0.75, 0.0, 0.0)), 80000.0);
		// 			}
		// 			if let Some(body_z_id) = bodies.get(&(x, z + 1)) {
		// 				physics_engine.create_ball_joint_spring_constraint(*body_id, &Pose::from_translation(Vec3::new(0.0, 0.0, 0.75)), *body_z_id, &Pose::from_translation(Vec3::new(0.0, 0.0, -0.75)), 80000.0);
		// 			}
		// 		}
		// 	}
		// }
		// for x in -1..0 {
		// 	for y in -1..0 {
		// 		for z in -1..0 {
		// 			let r = 4;
		// 			let physics_body_id = physics_engine.add_physics_body();
		// 			let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 			physics_body.pose.translation.y = (y as f32) * (r as f32) * 2.0 + 7.0 +620.0;
		// 			physics_body.pose.translation.z += (z as f32) * (r as f32) * 2.0 + 3.0 + y as f32;
		// 			physics_body.pose.translation.x += (x as f32) * (r as f32) * 2.0;
		// 			State::make_smooth_ball(physics_body, r);
		// 		}
		// 	}
		// }

		// ------------------------------ Grid Tesing ------------------------------
		// for x in 0..5 {
		// 	for y in 0..5 {
		// 		for z in 0..5 {
		// 			{
		// 				physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.grids.first_mut().unwrap().add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, 1), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.grids.first_mut().unwrap().add_voxel(IVec3::new(1, 0, 0), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.grids.first_mut().unwrap().add_voxel(IVec3::new(1, 0, 1), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.pose.translation.x += (x * 5) as f32;
		// 				physics_body.pose.translation.y = (y * 5) as f32;
		// 				physics_body.pose.translation.z -= (z * 5) as f32;
		// 				physics_body.is_static = true;
		// 			}
		// 			{
		// 				physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.grids.first_mut().unwrap().add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.pose.translation.x += (x * 5) as f32 - 0.4;;
		// 				physics_body.pose.translation.y = (y * 5) as f32 + 0.8 + (y as f32) * 0.05;
		// 				physics_body.pose.translation.z -= (z * 5) as f32 + 0.4;
		// 				physics_body.pose.rotation = Quat::from_rotation_z(z as f32 / 5.0) * Quat::from_rotation_x(x as f32 / 5.0);
		// 				physics_body.is_static = true;
		// 			}
		// 		}
		// 	}
		// }

		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let grid = physics_body.grid_mut(grid_id).unwrap();
		// 	grid.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, 1.0, 0.0, 1.0], mass: 1.0 });
		// 	physics_body.is_static = true;
		// 	physics_body.pose.translation.x = 4.0;
		// }
		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let grid = physics_body.grid_mut(grid_id).unwrap();
		// 	grid.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [1.0, 0.0, 0.0, 1.0], mass: 1.0 });
		// 	physics_body.pose.translation.y = 2.0;
		// 	physics_body.pose.translation.x = 4.0;
		// }

		// standing block
		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let grid = physics_body.grid_mut(grid_id).unwrap();
		// 	for x in -1..2 {
		// 		for y in 0..30 {
		// 			for z in -1..2 {
		// 				grid.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 });
		// 			}
		// 		}
		// 	}
		// 	grid.add_voxel(IVec3::new(0, 30, 0), voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 });
		// 	physics_body.pose.translation.y = 100.0;
		// 	let standing_entity_id = ecs.add_entity();
		// 	let mut orientator = Orientator::new();
		// 	orientator.set(physics_engine.start_tracking(physics_body_id, grid_id, IVec3::new(0, 30, 0)));
		// 	ecs.add_component_to_entity(standing_entity_id, orientator);
		// }

		// bb8
		{
			let physics_body_uuid = ResourceUUID(Uuid::new_v4());
			let physics_body_resource = resource_manager.create_resource_blank(
				physics_body_uuid.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid.clone()) }
			).unwrap();
			let physics_body_id = physics_engine.add_physics_body(physics_body_uuid.clone());
			{
				match physics_body_resource.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id)),
					_ => unreachable!()
				};
				let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
				let grid_uuid = ResourceUUID(Uuid::new_v4());
				let grid_resource = resource_manager.create_resource_blank(
					grid_uuid.clone(),
					ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body_uuid) }
				).unwrap();
				let grid_id = physics_body.add_grid(grid_uuid, Pose::new(Vec3::ZERO, Quat::IDENTITY));
				match grid_resource.info_mut() {
					ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
					_ => unreachable!()
				}
				let grid = physics_body.grid_mut(grid_id).unwrap();
				for x in -6..7 {
					for y in 0..3 {
						for z in -6..7 {
							grid.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 }, &mut resource_manager);
						}
					}
				}
				grid.add_voxel(IVec3::new(0, 3, 0), voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 }, &mut resource_manager);
				physics_body.pose.translation.y = 120.0;
				let standing_entity_id = ecs.add_entity();
				let mut orientator = Orientator::new();
				orientator.set(physics_engine.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
				ecs.add_component_to_entity(standing_entity_id, orientator);
			}
			let ball_body_uuid = ResourceUUID(Uuid::new_v4());
			let ball_body_resource = resource_manager.create_resource_blank(
				ball_body_uuid.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(ball_body_uuid.clone()) }
			).unwrap();
			let ball_physics_body_id = physics_engine.add_physics_body(ball_body_uuid);
			{
				match ball_body_resource.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(ball_physics_body_id)),
					_ => unreachable!()
				}
				let physics_body = physics_engine.physics_body_mut(ball_physics_body_id).unwrap();
				physics_body.pose.translation.y = 108.0;
				State::make_ball(&mut resource_manager, physics_body, 10);
			}
			physics_engine.create_ball_joint_constraint(physics_body_id, &Pose::from_translation(Vec3::new(0.0, -12.0, 0.0)), ball_physics_body_id, &Pose::ZERO);
		}
		{
			let physics_body_uuid = ResourceUUID(Uuid::new_v4());
			let physics_body_resource = resource_manager.create_resource_blank(
				physics_body_uuid.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid.clone()) }
			).unwrap();
			let physics_body_id = physics_engine.add_physics_body(physics_body_uuid.clone());
			{
				match physics_body_resource.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id)),
					_ => unreachable!()
				}
				let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
				let grid_uuid = ResourceUUID(Uuid::new_v4());
				let grid_resource = resource_manager.create_resource_blank(
					grid_uuid.clone(),
					ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body_uuid) }
				).unwrap();
				let grid_id = physics_body.add_grid(grid_uuid, Pose::new(Vec3::ZERO, Quat::IDENTITY));
				match grid_resource.info_mut() {
					ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
					_ => unreachable!()
				}
				let grid = physics_body.grid_mut(grid_id).unwrap();
				for x in -6..7 {
					for y in 0..3 {
						for z in -6..7 {
							grid.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 }, &mut resource_manager);
						}
					}
				}
				grid.add_voxel(IVec3::new(0, 3, 0), voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 }, &mut resource_manager);
				physics_body.pose.translation.y = 120.0;
				physics_body.pose.translation.x = 80.0;
				let standing_entity_id = ecs.add_entity();
				let mut orientator = Orientator::new();
				orientator.set(physics_engine.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
				ecs.add_component_to_entity(standing_entity_id, orientator);
			}
			let ball_body_uuid = ResourceUUID(Uuid::new_v4());
			let ball_body_resource = resource_manager.create_resource_blank(
				ball_body_uuid.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(ball_body_uuid.clone()) }
			).unwrap();
			let ball_physics_body_id = physics_engine.add_physics_body(ball_body_uuid);
			{
				match ball_body_resource.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(ball_physics_body_id)),
					_ => unreachable!()
				}
				let physics_body = physics_engine.physics_body_mut(ball_physics_body_id).unwrap();
				physics_body.pose.translation.y = 108.0;
				physics_body.pose.translation.x = 80.0;
				State::make_ball(&mut resource_manager, physics_body, 10);
			}
			physics_engine.create_ball_joint_constraint(physics_body_id, &Pose::from_translation(Vec3::new(0.0, -12.0, 0.0)), ball_physics_body_id, &Pose::ZERO);
		}
		{
			let physics_body_uuid = ResourceUUID(Uuid::new_v4());
			let physics_body_resource = resource_manager.create_resource_blank(
				physics_body_uuid.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(physics_body_uuid.clone()) }
			).unwrap();
			let physics_body_id = physics_engine.add_physics_body(physics_body_uuid.clone());
			{
				match physics_body_resource.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(physics_body_id)),
					_ => unreachable!()
				}
				let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
				let grid_uuid = ResourceUUID(Uuid::new_v4());
				let grid_resource = resource_manager.create_resource_blank(
					grid_uuid.clone(),
					ResourceInfoType::PhysicsBodyGrid { grid_resource: PhysicsBodyGridResource::new(grid_uuid.clone(), physics_body_uuid) }
				).unwrap();
				let grid_id = physics_body.add_grid(grid_uuid, Pose::new(Vec3::ZERO, Quat::IDENTITY));
				match grid_resource.info_mut() {
					ResourceInfoType::PhysicsBodyGrid { grid_resource } => grid_resource.set_physics_body_grid_id(Some(grid_id)),
					_ => unreachable!()
				}
				let grid = physics_body.grid_mut(grid_id).unwrap();
				for x in -6..7 {
					for y in 0..3 {
						for z in -6..7 {
							grid.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 }, &mut resource_manager);
						}
					}
				}
				grid.add_voxel(IVec3::new(0, 3, 0), voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 }, &mut resource_manager);
				physics_body.pose.translation.y = 120.0;
				physics_body.pose.translation.x = 30.0;
				let standing_entity_id = ecs.add_entity();
				let mut orientator = Orientator::new();
				orientator.set(physics_engine.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
				ecs.add_component_to_entity(standing_entity_id, orientator);
				let mut player_tracker = PlayerTracker::new();
				player_tracker.set(physics_engine.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
				ecs.add_component_to_entity(standing_entity_id, player_tracker);
			}
			let ball_body_uuid = ResourceUUID(Uuid::new_v4());
			let ball_body_resource = resource_manager.create_resource_blank(
				ball_body_uuid.clone(),
				ResourceInfoType::PhysicsBody { physics_body_resource: PhysicsBodyResource::new(ball_body_uuid.clone()) }
			).unwrap();
			let ball_physics_body_id = physics_engine.add_physics_body(ball_body_uuid);
			{
				match ball_body_resource.info_mut() {
					ResourceInfoType::PhysicsBody { physics_body_resource } => physics_body_resource.set_physics_body_id(Some(ball_physics_body_id)),
					_ => unreachable!()
				}
				let physics_body = physics_engine.physics_body_mut(ball_physics_body_id).unwrap();
				physics_body.pose.translation.y = 108.0;
				physics_body.pose.translation.x = 30.0;
				State::make_ball(&mut resource_manager, physics_body, 10);
			}
			physics_engine.create_ball_joint_constraint(physics_body_id, &Pose::from_translation(Vec3::new(0.0, -12.0, 0.0)), ball_physics_body_id, &Pose::ZERO);
		}

		// terrain
		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let world_generator = crate::world_gen::WorldGenerator::new(2);
		// 	let grid_id = physics_body.add_grid(Pose::ZERO);
		// 	let grid = physics_body.grid_mut(grid_id).unwrap();
		// 	world_generator.gererate_area(glam::IVec2::new(-512, -512), glam::IVec2::new(512, 512), grid);
		// 	physics_body.is_static = true;
		// 	physics_body.pose.translation.y = -10.0;
		// }

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

		Ok(Self {
			renderer,
			mouse_captured: false,
			audio_engine: AudioEngine::new(),
			physics_engine,
			ecs,
			resource_manager,
			player_id,
			leaky_bucket: 0.0,
			raycast_pose: None,
			place_sound_cooldown: 0.0,
			break_sound_cooldown: 0.0,
			task_queue: Arc::new(Mutex::new(VecDeque::new())),
			async_task_priority_queue,
			async_task_priority_queue_threads,
			freeze_gpu_grids: false,
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::CurrentSurfaceTexture> {
		// for physics_body in self.physics_engine.physics_bodies() {
			// for grid in physics_body.grids() {
			// 	grid.get_voxels().render_debug(&(physics_body.pose * grid.pose));
			// }
			// if !physics_body.is_static {
			// 	physics_body.render_debug_inertia_box();
			// }
		// }
		if let Some(player_camera) = self.ecs.get_component::<Camera>(self.player_id) {
			let mut id_to_hit_count = HashMap::new();
			for (id, hit_count) in self.renderer.bvh_item_ids.iter().zip(self.renderer.bvh_item_hit_counts.iter()) {
				id_to_hit_count.insert(*id, *hit_count);
			}
			let mut gpu_grid_tree_id_to_id_poses: HashMap<(PhysicsBodyId, PhysicsBodyGridId, SubGridId), (u32, u32, Pose)> = HashMap::new();
			{
				let _zone = span!("Collect Voxels");
				let view_frustum = player_camera.frustum();
				for physics_body in self.physics_engine.physics_bodies().iter() {
					for (key, value) in physics_body.update_gpu_grid_tree(
						&mut self.renderer.voxel_renderer.packed_64_tree_dynamic_buffer,
						&mut self.renderer.voxel_renderer.packed_voxel_data_dynamic_buffer,
						&mut self.resource_manager,
						&self.physics_engine,
						&mut self.async_task_priority_queue,
						&mut self.task_queue,
						&id_to_hit_count,
						&view_frustum,
						player_camera.pose(),
					) {
						gpu_grid_tree_id_to_id_poses.insert((physics_body.id(), key.0, key.1), value);
					}
				}
			}
			let bvh = {
				let mut bounds = vec![];
				{
					let _zone = span!("Collect aabb for rendering");
					for ((body_id, grid_id, sub_grid_id), _) in gpu_grid_tree_id_to_id_poses.iter() {
						let physics_body = &self.physics_engine.physics_body(*body_id).unwrap();
						if let Some(bound) = physics_body.sub_grid_aabb(*grid_id, *sub_grid_id) {
							bounds.push(((*body_id, *grid_id, *sub_grid_id), bound));
						}
					}
				}
				BVH::new(bounds)
			};
			return self.renderer.render(&player_camera, &bvh, &gpu_grid_tree_id_to_id_poses);
		} else {
			println!("Error: could not find player camera!");
			return Ok(());
		}
	}
}
