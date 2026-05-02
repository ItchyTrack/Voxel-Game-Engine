#[cfg(target_arch = "wasm32")]
use web_time::Instant;
use std::{collections::HashMap, f32, sync::Arc};

use glam::{IVec3, Quat, Vec3};
use tracy_client::span;
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::{CursorGrabMode, Window}};

use crate::{audio::audio_engine::{AudioEngine, ListenerState, SoundEffect}, player::camera, world::physics_solver::bvh::BVH};
use crate::world::{world::World, entity_component_system::entity_component_system::EntityId, physics_body::PhysicsBodyId};
use crate::player::{camera::{Camera, CameraController}, player_input::PlayerInput, object_pickup::ObjectPickup, player_tracker::PlayerTracker, orientator::Orientator};
use crate::render::renderer::Renderer;
use crate::world::{voxels, pose::Pose};

const BLOCK_PLACE_SOUND_INTERVAL_SECONDS: f32 = 1.0 / (18.0 * 2.0);
const BLOCK_BREAK_SOUND_INTERVAL_SECONDS: f32 = 1.0 / (14.0 * 2.0);

pub struct State {
	pub renderer: Renderer,
	pub mouse_captured: bool,
	pub audio_engine: AudioEngine,
	pub player_id: EntityId,
	pub place_sound_cooldown: f32,
	pub break_sound_cooldown: f32,
	pub debug_enables: DebugEnables,
	pub world: World,
}

pub struct DebugEnables {
	pub freeze_gpu_grids: bool,
	pub freeze_physics: bool,
	pub inertia_boxes: bool,
	pub raycast_pose: Option<Pose>,
}

impl DebugEnables {
	pub fn new() -> Self {
		Self {
			freeze_gpu_grids: false,
			freeze_physics: false,
			inertia_boxes: false,
			raycast_pose: None,
		}
	}
}

impl State {
	pub fn handle_key(&mut self, _event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
		if code == KeyCode::Escape && is_pressed {
			self.set_mouse_captured(false);
		} else {
			self.world.ecs.write().run_on_single_component_mut::<PlayerInput, _>(self.player_id, |_entity_id, player_input|
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
		self.world.update(dt, self.player_id, &mut self.debug_enables);
		// let _zone = span!("State Update");
		// if !self.debug_enables.freeze_gpu_grids {
		// 	let _zone = span!("Do tasks");
		// 	while let Some(task) = { self.task_queue.lock().unwrap().pop_front() } {
		// 		task.run(self);
		// 	}
		// }
		// self.place_sound_cooldown = (self.place_sound_cooldown - dt).max(0.0);
		// self.break_sound_cooldown = (self.break_sound_cooldown - dt).max(0.0);
		// self.ecs.run_on_components_tripl_mut::<PlayerInput, CameraController, Camera, _>(&mut |_entity_id, player_input, camera_controller, camera|
		// 	CameraController::update_camera(camera_controller, camera, player_input, dt)
		// );

		// if let Some(camera) = self.ecs.get_component::<Camera>(self.player_id) {
		// 	let (forward, right, _) = camera.forward_right_up();
		// 	self.audio_engine.set_listener(ListenerState {
		// 		position: camera.position,
		// 		forward,
		// 		right,
		// 	});
		// 	if let Some(player_input) = self.ecs.get_component::<PlayerInput>(self.player_id) {
		// 		if player_input.key(KeyCode::KeyM).just_pressed {
		// 			self.audio_engine.play_sound(SoundEffect::DebugBeep, camera.position + forward * 20.0);
		// 		}
		// 	}
		// }

		// self.ecs.run_on_components_tripl_mut::<PlayerInput, Camera, ObjectPickup, _>(&mut |_entity_id, player_input, camera, object_pickup| {
		// 	let ray_start = if self.raycast_pose.is_none() {
		// 		Pose::new(camera.position, Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch))
		// 	} else {
		// 		self.raycast_pose.unwrap()
		// 	};
		// 	if player_input.key(KeyCode::KeyH).just_pressed {
		// 		if self.raycast_pose.is_none() {
		// 			self.raycast_pose = Some(Pose::new(camera.position, Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch)));
		// 		} else {
		// 			self.raycast_pose = None;
		// 		}
		// 	}
		// 	let started_holding = object_pickup.is_holding();
		// 	if let Some((body_id, grid_id, hit_pos, hit_normal, distance)) = self.physics_engine.raycast(&ray_start, None) {
		// 		let physics_body = self.physics_engine.physics_body(body_id).unwrap();
		// 		let grid = physics_body.grid(grid_id).unwrap();
		// 		let globle_hit_normal = physics_body.pose.rotation * grid.pose.rotation * hit_normal.as_vec3();
		// 		let globle_hit_pos = ray_start.translation + ray_start.rotation * Vec3::Z * distance;
		// 		let globle_hit_pos_snap = physics_body.pose * grid.pose * hit_pos.as_vec3();
		// 		let place_voxel_pos = hit_pos + hit_normal.as_ivec3();
		// 		let break_voxel_pos = hit_pos;
		// 		let place_sound_pos = physics_body.pose * grid.pose * (place_voxel_pos.as_vec3() + Vec3::splat(0.5));
		// 		let break_sound_pos = physics_body.pose * grid.pose * (break_voxel_pos.as_vec3() + Vec3::splat(0.5));
		// 		debug_draw::line(globle_hit_pos, globle_hit_pos + globle_hit_normal, &Vec4::new(1.0, 0.0, 0.0, 1.0));
		// 		debug_draw::rectangular_prism(&Pose::new(globle_hit_pos_snap, physics_body.pose.rotation * grid.pose.rotation), Vec3::splat(1.0), &Vec4::new(1.0, 0.0, 1.0, 0.1), true);
		// 		if player_input.key(KeyCode::Space).just_pressed || player_input.key(KeyCode::KeyC).is_pressed {
		// 			self.physics_engine.physics_body_mut(body_id).unwrap().grid_mut(grid_id).unwrap().add_voxel(place_voxel_pos, Voxel{ color: [100, 100, 100, 1], mass: 100 }, &mut self.resource_manager);
		// 			if self.place_sound_cooldown <= 0.0 {
		// 				self.audio_engine.play_sound(SoundEffect::BlockPlace, place_sound_pos);
		// 				self.place_sound_cooldown = BLOCK_PLACE_SOUND_INTERVAL_SECONDS;
		// 			}
		// 		}
		// 		if player_input.key(KeyCode::KeyX).just_pressed || player_input.key(KeyCode::KeyZ).is_pressed {
		// 			self.physics_engine.physics_body_mut(body_id).unwrap().grid_mut(grid_id).unwrap().remove_voxel(&break_voxel_pos);
		// 			if self.break_sound_cooldown <= 0.0 {
		// 				self.audio_engine.play_sound(SoundEffect::BlockBreak, break_sound_pos);
		// 				self.break_sound_cooldown = BLOCK_BREAK_SOUND_INTERVAL_SECONDS;
		// 			}
		// 		}
		// 		if player_input.key(KeyCode::KeyR).just_pressed {
		// 			self.physics_engine.apply_impulse(
		// 				self.physics_engine.physics_body(body_id).unwrap().id(),
		// 				&globle_hit_pos,
		// 				&(ray_start.rotation * Vec3::Z * 1600000.0)
		// 			);
		// 		}
		// 		if player_input.key(KeyCode::KeyF).just_pressed {
		// 			if !started_holding {
		// 				let body = self.physics_engine.physics_body(body_id).unwrap();
		// 				if !body.is_static {
		// 					object_pickup.set(body.id());
		// 				}
		// 			}
		// 		}
		// 	}
		// 	if player_input.key(KeyCode::KeyF).just_pressed {
		// 		if started_holding {
		// 			object_pickup.reset();
		// 		}
		// 	}
		// 	if player_input.key(KeyCode::KeyT).just_pressed {
		// 		self.debug_enables.freeze_gpu_grids ^= true;
		// 	}
		// 	if player_input.key(KeyCode::KeyP).just_pressed {
		// 		self.debug_enables.freeze_physics ^= true;
		// 	}
		// });
		// self.physics_engine.update();
		// if !self.debug_enables.freeze_physics {
		// 	self.leaky_bucket += dt;
		// 	let time_step = 1.0 / 100.0;
		// 	let current_time = Instant::now();
		// 	while self.leaky_bucket >= time_step {
		// 		self.ecs.run_on_components_pair_mut::<Camera, ObjectPickup, _>(&mut |_entity_id, camera, object_pickup| {
		// 			if object_pickup.is_holding() {
		// 				object_pickup.hold_at_pos(&(camera.position + camera.forward() * 40.0), time_step, &mut self.physics_engine);
		// 			}
		// 		});
		// 		self.ecs.run_on_components_mut::<Orientator, _>(&mut |_entity_id, orientator| {
		// 			orientator.hold_at_orientation(&Quat::IDENTITY, time_step, &mut self.physics_engine);
		// 		});
		// 		let player_position = self.ecs.get_component::<Camera>(self.player_id).unwrap().position;
		// 		self.ecs.run_on_components_mut::<PlayerTracker, _>(&mut |_entity_id, player_tracker| {
		// 			player_tracker.track_pos(&player_position, time_step, &mut self.physics_engine);
		// 		});
		// 		self.physics_engine.update_physics(time_step);
		// 		self.leaky_bucket -= time_step;
		// 		let elapsed = current_time.elapsed().as_secs_f32();
		// 		if elapsed > 1.0 / 60.0 {
		// 			self.leaky_bucket = 0.0;
		// 		}
		// 	}
		// }
		self.world.ecs.write().run_on_single_component_mut::<PlayerInput, _>(self.player_id, |_entity_id, player_input|
			player_input.end_frame()
		);
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
		self.world.ecs.write().run_on_components_mut::<Camera, _>(&mut |_entity_id, camera| {
			camera.aspect = self.renderer.config.width as f32 / self.renderer.config.height as f32;
		})
	}

	fn make_ball(world: &World, physics_body_id: PhysicsBodyId, radius: i32) {
		{
			let grid_id = world.add_grid(physics_body_id, &Pose::new(Vec3::new(-0.5, -0.5, -0.5), Quat::IDENTITY)).unwrap();
			let grid = &mut world.grid_mut(grid_id).unwrap();

			for x in -radius..radius + 1 {
				for y in -0..radius + 1 {
					for z in -radius..radius + 1 {
						if IVec3::new(x, y, z).length_squared() as f32 <= (radius as f32 - 0.5).powf(2.0) {
							grid.add_voxel(
								&IVec3::new(x, y, z),
								&voxels::Voxel { color: [(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255], mass: 100 }
							);
						}
					}
				}
			}
		}
		{
			let grid_id = world.add_grid(physics_body_id, &Pose::new(Vec3::new(-0.707, -0.5, 0.0), Quat::from_rotation_y(f32::consts::PI / 4.0))).unwrap();
			let grid = &mut world.grid_mut(grid_id).unwrap();

			for x in -radius..radius + 1 {
				for y in -radius..0 {
					for z in -radius..radius + 1 {
						if IVec3::new(x, y, z).length_squared() as f32 <= (radius as f32 - 0.5).powf(2.0) {
							grid.add_voxel(
								&IVec3::new(x, y, z),
								&voxels::Voxel { color: [(x as u8 / 10) * 10, (y as u8 / 10) * 10, (z as u8 / 10) * 10, 255], mass: 100 }
							);
						}
					}
				}
			}
		}
	}

	fn make_smooth_ball(world: &World, physics_body_id: PhysicsBodyId, radius: i32){
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
					let grid_id = world.add_grid(physics_body_id, &Pose::new(translation, rotation)).unwrap();
					let grid = &mut world.grid_mut(grid_id).unwrap();
					grid.add_voxel(
						&IVec3::ZERO,
						&voxels::Voxel {
							color: [
								((normal.x * 0.5 + 0.5) * 255.0) as u8,
								((normal.y * 0.5 + 0.5) * 255.0) as u8,
								((normal.z * 0.5 + 0.5) * 255.0) as u8,
								1,
							],
							mass: 100,
						}
					);
				}
			}
		}
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<Self> {
		let renderer = Renderer::new(window).await?;

		let world = World::new(renderer.device.clone(), renderer.queue.clone());
		let player_id = {
			let ecs = &mut world.ecs.write();
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

			match crate::resources::load_binary("Church_Of_St_Sophia.vox").await {
				Ok(bytes) => {
					match dot_vox::load_bytes(&bytes) {
						Ok(dot_vox_data) => {
							let physics_body_id = world.add_physics_body();
							{
								let physics_body = &mut world.physics_body_mut(physics_body_id).unwrap();
								physics_body.pose.translation.y -= 350.0;
								physics_body.is_static = true;
							}
							let grid_id = world.add_grid(physics_body_id, &Pose::ZERO).unwrap();
							let grid = &mut world.grid_mut(grid_id).unwrap();
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
													grid.add_voxel(
														&(
															pose * Pose::from_translation(-half * flip.as_vec3()) * (
																IVec3::new(
																	voxel.x as i32,
																	voxel.z as i32,
																	voxel.y as i32,
																) * flip + flip.min(IVec3::ZERO)
															).as_vec3()
														).as_ivec3(),
														&voxels::Voxel {
															color: [
																dot_vox_data.palette[voxel.i as usize].r,
																dot_vox_data.palette[voxel.i as usize].g,
																dot_vox_data.palette[voxel.i as usize].b,
																dot_vox_data.palette[voxel.i as usize].a,
															],
															mass: 100,
														}
													);
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

			{
				let r = 5;
				let physics_body_id_main = world.add_physics_body();
				{
					let physics_body_id = {
						let physics_body = &mut world.physics_body_mut(physics_body_id_main).unwrap();
						physics_body.pose.translation.x += 0.0;
						physics_body.pose.translation.y = 80.0;
						physics_body.pose.translation.z += 40.0 - 60.0;
						physics_body.id()
					};
					State::make_ball(&world, physics_body_id, 2);
				}
				let physics_body_id_1 = world.add_physics_body();
				{
					let physics_body_id = {
						let physics_body = &mut world.physics_body_mut(physics_body_id_1).unwrap();
						physics_body.pose.translation.x += 0.0;
						physics_body.pose.translation.y = 80.0;
						physics_body.pose.translation.z += 50.0 - 60.0;
						physics_body.id()
					};
					State::make_ball(&world, physics_body_id, r);
				}
				let physics_body_id_2 = world.add_physics_body();
				{
					let physics_body_id = {
						let physics_body = &mut world.physics_body_mut(physics_body_id_2).unwrap();
						physics_body.pose.translation.x += 0.0;
						physics_body.pose.translation.y = 80.0;
						physics_body.pose.translation.z += 30.0 - 60.0;
						physics_body.id()
					};
					State::make_ball(&world, physics_body_id, r);
				}
				let physics_body_id_3 = world.add_physics_body();
				{
					let physics_body_id = {
						let physics_body = &mut world.physics_body_mut(physics_body_id_3).unwrap();
						physics_body.pose.translation.x += 10.0;
						physics_body.pose.translation.y = 80.0;
						physics_body.pose.translation.z += 40.0 - 60.0;
						physics_body.id()
					};
					State::make_ball(&world, physics_body_id, r);
				}
				let physics_body_id_4 = world.add_physics_body();
				{
					let physics_body_id = {
						let physics_body = &mut world.physics_body_mut(physics_body_id_4).unwrap();
						physics_body.pose.translation.x += -10.0;
						physics_body.pose.translation.y = 80.0;
						physics_body.pose.translation.z += 40.0 - 60.0;
						physics_body.id()
					};
					State::make_ball(&world, physics_body_id, r);
				}
				world.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(0.0, 0.0, 10.0)), physics_body_id_1, &Pose::ZERO);
				world.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(0.0, 0.0, -10.0)), physics_body_id_2, &Pose::ZERO);
				world.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(10.0, 0.0, 0.0)), physics_body_id_3, &Pose::ZERO);
				world.create_ball_joint_constraint(physics_body_id_main, &Pose::from_translation(Vec3::new(-10.0, 0.0, 0.0)), physics_body_id_4, &Pose::ZERO);
			}

			// bb8
			{
				let physics_body_id = world.add_physics_body();
				{
					let grid_id = world.add_grid(physics_body_id, &Pose::new(Vec3::ZERO, Quat::IDENTITY)).unwrap();
					let grid = &mut world.grid_mut(grid_id).unwrap();
					for x in -6..7 {
						for y in 0..3 {
							for z in -6..7 {
								grid.add_voxel(&IVec3::new(x, y, z), &voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 });
							}
						}
					}
					grid.add_voxel(&IVec3::new(0, 3, 0), &voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 });
					world.physics_body_mut(physics_body_id).unwrap().pose.translation.y = 120.0;
					let standing_entity_id = ecs.add_entity();
					let mut orientator = Orientator::new();
					orientator.set(world.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
					ecs.add_component_to_entity(standing_entity_id, orientator);
				}
				let ball_physics_body_id = world.add_physics_body();
				{
					world.physics_body_mut(ball_physics_body_id).unwrap().pose.translation.y = 108.0;
					State::make_ball(&world, ball_physics_body_id, 10);
				}
				world.create_ball_joint_constraint(physics_body_id, &Pose::from_translation(Vec3::new(0.0, -12.0, 0.0)), ball_physics_body_id, &Pose::ZERO);
			}
			// {
			// 	let physics_body_id = world.add_physics_body();
			// 	{
			// 		let physics_body = &mut world.physics_body_mut(physics_body_id).unwrap();
			// 		let grid_id = world.add_grid(physics_body_id, &Pose::new(Vec3::ZERO, Quat::IDENTITY)).unwrap();
			// 		let grid = &mut world.grid_mut(grid_id).unwrap();
			// 		for x in -6..7 {
			// 			for y in 0..3 {
			// 				for z in -6..7 {
			// 					grid.add_voxel(&IVec3::new(x, y, z), &voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 });
			// 				}
			// 			}
			// 		}
			// 		grid.add_voxel(&IVec3::new(0, 3, 0), &voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 });
			// 		physics_body.pose.translation.y = 120.0;
			// 		physics_body.pose.translation.x = 80.0;
			// 		let standing_entity_id = ecs.add_entity();
			// 		let mut orientator = Orientator::new();
			// 		orientator.set(world.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
			// 		ecs.add_component_to_entity(standing_entity_id, orientator);
			// 	}
			// 	let ball_physics_body_id = world.add_physics_body();
			// 	{
			// 		let physics_body = &mut world.physics_body_mut(ball_physics_body_id).unwrap();
			// 		physics_body.pose.translation.y = 108.0;
			// 		physics_body.pose.translation.x = 80.0;
			// 		State::make_ball(&world, physics_body.id(), 10);
			// 	}
			// 	world.create_ball_joint_constraint(physics_body_id, &Pose::from_translation(Vec3::new(0.0, -12.0, 0.0)), ball_physics_body_id, &Pose::ZERO);
			// }
			// {
			// 	let physics_body_id = world.add_physics_body();
			// 	{
			// 		let physics_body = &mut world.physics_body_mut(physics_body_id).unwrap();
			// 		let grid_id = world.add_grid(physics_body_id, &Pose::new(Vec3::ZERO, Quat::IDENTITY)).unwrap();
			// 		let grid = &mut world.grid_mut(grid_id).unwrap();
			// 		for x in -6..7 {
			// 			for y in 0..3 {
			// 				for z in -6..7 {
			// 					grid.add_voxel(&IVec3::new(x, y, z), &voxels::Voxel{ color: [128, 128, 128, 255], mass: 200 });
			// 				}
			// 			}
			// 		}
			// 		grid.add_voxel(&IVec3::new(0, 3, 0), &voxels::Voxel{ color: [255, 0, 0, 255], mass: 200 });
			// 		physics_body.pose.translation.y = 120.0;
			// 		physics_body.pose.translation.x = 30.0;
			// 		let standing_entity_id = ecs.add_entity();
			// 		let mut orientator = Orientator::new();
			// 		orientator.set(world.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
			// 		ecs.add_component_to_entity(standing_entity_id, orientator);
			// 		let mut player_tracker = PlayerTracker::new();
			// 		player_tracker.set(world.start_tracking(physics_body_id, grid_id, IVec3::new(0, 3, 0)));
			// 		ecs.add_component_to_entity(standing_entity_id, player_tracker);
			// 	}
			// 	let ball_physics_body_id = world.add_physics_body();
			// 	{
			// 		let physics_body = &mut world.physics_body_mut(ball_physics_body_id).unwrap();
			// 		physics_body.pose.translation.y = 108.0;
			// 		physics_body.pose.translation.x = 30.0;
			// 		State::make_ball(&world, physics_body.id(), 10);
			// 	}
			// 	world.create_ball_joint_constraint(physics_body_id, &Pose::from_translation(Vec3::new(0.0, -12.0, 0.0)), ball_physics_body_id, &Pose::ZERO);
			// }

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

			player_id
		};

		Ok(Self {
			renderer,
			mouse_captured: false,
			audio_engine: AudioEngine::new(),
			player_id,
			place_sound_cooldown: 0.0,
			break_sound_cooldown: 0.0,
			debug_enables: DebugEnables::new(),
			world,
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::CurrentSurfaceTexture> {
		// for grid in physics_body.grids() {
		// 	grid.get_voxels().render_debug(&(physics_body.pose * grid.pose));
		// }
		if self.debug_enables.inertia_boxes {
			for (_physics_body_id, physics_body) in self.world.physics_bodies.read().iter() {
				if !physics_body.is_static {
					physics_body.render_debug_inertia_box();
				}
			}
		}
		if let Some(player_camera) = self.world.ecs.read().get_component::<Camera>(self.player_id) {
			let mut id_to_hit_count = HashMap::new();
			for (id, hit_count) in self.renderer.bvh_item_ids.iter().zip(self.renderer.bvh_item_hit_counts.iter()) {
				id_to_hit_count.insert(*id, *hit_count);
			}
			let view_frustum = player_camera.frustum();
			let gpu_grid_tree_id_to_id_poses = self.world.update_gpu_grid_tree(&id_to_hit_count, &view_frustum, &player_camera.pose());
			let bvh = {
				let mut bounds = vec![];
				{
					let _zone = span!("Collect aabb for rendering");
					for ((body_id, grid_id, sub_grid_id), _) in gpu_grid_tree_id_to_id_poses.iter() {
						if let Some(grid) = self.world.grid_manager.read().grid(*grid_id) && let Some(body) = self.world.physics_body(*body_id) {
							if let Some(sub_grid) = grid.sub_grid(*sub_grid_id) {
								if let Some(bound) = sub_grid.aabb(&(body.pose * grid.pose() * Pose::from_translation(sub_grid.sub_grid_pos().as_vec3()))) {
									bounds.push(((*body_id, *grid_id, *sub_grid_id), bound));
								}
							}
						}
					}
				}
				BVH::new(bounds)
			};
			let world_gpu_data = self.world.get_rendering_buffers();
			return self.renderer.render(
				&player_camera,
				&bvh,
				&gpu_grid_tree_id_to_id_poses,
				&mut self.debug_enables,
				&world_gpu_data.packed_64_tree_dynamic_buffer.read(),
				&world_gpu_data.packed_voxel_data_dynamic_buffer.read()
			);
		} else {
			println!("Error: could not find player camera!");
			return Ok(());
		}
	}
}
