use std::{f32, sync::Arc};
use std::time::Instant;

use glam::{I16Vec3, IVec2, Mat4, Quat, Vec3, Vec4};
use tracy_client::span;
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::{CursorGrabMode, Window}};

use crate::debug_draw;
use crate::voxels::Voxel;
use crate::{player::{camera, player_input}, entity_component_system, gpu_objects::mesh, physics::{physics_body::PhysicsBody, physics_engine::PhysicsEngine}, pose::Pose, renderer::Renderer, voxels, world_gen::WorldGenerator};

pub struct State {
	pub renderer: Renderer,
	pub mouse_captured: bool,
	pub physics_engine: PhysicsEngine,
	pub ecs: entity_component_system::EntityComponentSystem,
	pub player_id: u32,
	pub leaky_bucket: f32,
}

impl State {
	pub fn handle_key(&mut self, _event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
		if code == KeyCode::Escape && is_pressed {
			self.set_mouse_captured(false);
		} else {
			self.ecs.run_on_single_component_mut::<player_input::PlayerInput, _>(self.player_id, |_entity_id, player_input|
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
		self.ecs.run_on_components_tripl_mut::<player_input::PlayerInput, camera::CameraController, camera::Camera, _>(|_entity_id, player_input, camera_controller, camera|
			camera::CameraController::update_camera(camera_controller, camera, player_input, dt)
		);
		self.ecs.run_on_components_pair_mut::<player_input::PlayerInput, camera::Camera, _>(&mut |_entity_id, player_input, camera| {
				let ray_start = Pose::new(camera.position, Quat::from_euler(glam::EulerRot::ZYX, 0.0, camera.yaw, camera.pitch));
			if let Some((body_index, grid_index, hit_pos, hit_normal, distance)) = self.physics_engine.raycast(&ray_start, None) {
				let physics_body = self.physics_engine.physics_body_by_index(body_index).unwrap();
				let grid = physics_body.grid(grid_index).unwrap();
				let globle_hit_normal = physics_body.pose.rotation * grid.pose.rotation * hit_normal.as_vec3();
				let globle_hit_pos = ray_start.translation + ray_start.rotation * Vec3::Z * distance;
				let globle_hit_pos_snap = physics_body.pose * grid.pose * hit_pos.as_vec3();
				debug_draw::line(globle_hit_pos, globle_hit_pos + globle_hit_normal, &Vec4::new(1.0, 0.0, 0.0, 1.0));
				debug_draw::rectangular_prism(&Pose::new(globle_hit_pos_snap, physics_body.pose.rotation * grid.pose.rotation), Vec3::splat(1.0), &Vec4::new(1.0, 0.0, 1.0, 0.1), true);
				if player_input.key(KeyCode::Space).just_pressed || player_input.key(KeyCode::KeyC).is_pressed {
					self.physics_engine.physics_body_by_index_mut(body_index).unwrap().grid_by_index_mut(grid_index).unwrap().add_voxel(hit_pos + hit_normal.as_i16vec3(), Voxel{ color: [100, 100, 100, 1], mass: 100 });
				}
				if player_input.key(KeyCode::KeyX).just_pressed || player_input.key(KeyCode::KeyZ).is_pressed {
					self.physics_engine.physics_body_by_index_mut(body_index).unwrap().grid_by_index_mut(grid_index).unwrap().remove_voxel(&(hit_pos));
				}
			}
		});
		self.leaky_bucket += dt;
		let time_step = 1.0 / 120.0;
		let current_time = Instant::now();
		while self.leaky_bucket >= time_step {
			self.physics_engine.update(time_step);
			self.leaky_bucket -= time_step;
			let elapsed = current_time.elapsed().as_secs_f32();
			if elapsed > 1.0 / 60.0 {
				self.leaky_bucket = 0.0;
			}
		}
		self.ecs.run_on_single_component_mut::<player_input::PlayerInput, _>(self.player_id, |_entity_id, player_input|
			player_input.end_frame()
		);
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
		self.ecs.run_on_components_mut::<camera::Camera, _>(|_entity_id, camera,| {
			camera.aspect = self.renderer.config.width as f32 / self.renderer.config.height as f32;
		})
	}

	fn make_ball(physics_body: &mut PhysicsBody, radius: i16) {
		{
			let sub_grid_id = physics_body.add_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
			let sub_grid = physics_body.grid_mut(sub_grid_id).unwrap();

			for x in -radius..radius + 1 {
				for y in -0..radius + 1 {
					for z in -radius..radius + 1 {
						if I16Vec3::new(x, y, z).length_squared() as f32 <= (radius as f32 - 0.5).powf(2.0)  {
							sub_grid.add_voxel(
								I16Vec3::new(x, y + 2, z),
								voxels::Voxel{ color: [x as u8, y as u8, z as u8, 1], mass: 100 }
							);
						}
					}
				}
			}
		}
		{
			let sub_grid_id = physics_body.add_grid(Pose::new(Vec3::new(-0.707 + 0.5, 0.0, 0.707 - 0.5), Quat::from_rotation_y(f32::consts::PI/4.0)));
			let sub_grid = physics_body.grid_mut(sub_grid_id).unwrap();
			for x in -radius..radius + 1 {
				for y in -radius..0 {
					for z in -radius..radius + 1 {
						if I16Vec3::new(x, y, z).length_squared() as f32 <= (radius as f32 - 0.5).powf(2.0)  {
							sub_grid.add_voxel(
								I16Vec3::new(x, y + 2, z),
								voxels::Voxel{ color: [x as u8, y as u8, z as u8, 1], mass: 100 }
							);
						}
					}
				}
			}
		}
	}

	fn make_smooth_ball(physics_body: &mut PhysicsBody, radius: i32){
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
					let sub_grid_id = physics_body.add_grid(Pose::new(translation, rotation));
					let sub_grid = physics_body.grid_mut(sub_grid_id).unwrap();
					sub_grid.add_voxel(
						I16Vec3::ZERO,
						voxels::Voxel {
							color: [
								((normal.x * 0.5 + 0.5) * 255.0) as u8,
								((normal.y * 0.5 + 0.5) * 255.0) as u8,
								((normal.z * 0.5 + 0.5) * 255.0) as u8,
								1,
							],
							mass: 100,
						},
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
		ecs.add_component_to_entity(player_id, player_input::PlayerInput::new());
		ecs.add_component_to_entity(player_id, camera::Camera {
			position: Vec3::new(-30.0, 15.0, 0.0),
			yaw: f32::consts::PI / 2.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			zfar: 5000.0,
		});
		ecs.add_component_to_entity(player_id, camera::CameraController::new(20.0, 1.5, 0.0015));

		// match load_binary("#treehouse.vox").await {
		// 	Ok(bytes) => {
		// 		match dot_vox::load_bytes(&bytes) {
		// 			Ok(dot_vox_data) => {
		// 				for model in dot_vox_data.models {
		// 					physics_bodies.push(physics_body::PhysicsBody::new());
		// 					let physics_body = physics_bodies.last_mut().unwrap();
		// 					physics_body.is_static = true;
		// 					for voxel in model.voxels {
		// 						physics_body.add_voxel(IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32), voxels::Voxel{ color: [
		// 							dot_vox_data.palette[voxel.i as usize].r as f32 / 255.0,
		// 							dot_vox_data.palette[voxel.i as usize].g as f32 / 255.0,
		// 							dot_vox_data.palette[voxel.i as usize].b as f32 / 255.0,
		// 							dot_vox_data.palette[voxel.i as usize].a as f32 / 255.0
		// 						], mass: 1.0 });
		// 					}
		// 				}
		// 			},
		// 			Err(err) => println!("dot_vox error: {err}"),
		// 		};
		// 	},
		// 	Err(err) => println!("load_string error: {err}"),
		// }

		// ------------------------------ Static Box ------------------------------
		// {
		// 	physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 	let physics_body = physics_bodies.last_mut().unwrap();
		// 	for x in -2..3 {
		// 		for y in -14..-6 {
		// 			for z in -2..3 {
		// 				if (x as i32).abs() == 2 || (y as i32).abs() == 14 || (z as i32).abs() == 2 {
		// 					physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(x, y+7, z), voxels::Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1.0 });
		// 				}
		// 			}
		// 		}
		// 	}
		// 	physics_body.position.z -= 8.0;
		// 	physics_body.position.y += 2.0;
		// 	physics_body.is_static = true;
		// }
		// ------------------------------ Cube Stack ------------------------------
		// {
		// 	for _x in 0..1 {
		// 		for y in 0..10 {
		// 			for _z in 0..1 {
		// 				physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, (y % 4) as f32 / 4.0, 0.5, 1.0], mass: 1.0 });
		// 				// physics_body.position.x += y as f32 * 0.01;
		// 				physics_body.position.y += y as f32 * 1.0;
		// 				physics_body.position.y += 0.0;
		// 				physics_body.position.z -= 8.0;
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
		// 	let sub_grid_id = physics_body.add_sub_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let sub_grid = physics_body.sub_grid_mut(sub_grid_id).unwrap();
		// 	for x in -15..16 {
		// 		for y in 0..20 {
		// 			sub_grid.add_voxel(I16Vec3::new(x, y, -10), voxels::Voxel{ color: [(x * 32 % 255 * 0) as u8, 0, (y * 32 % 255) as u8, 1], mass: 100 });
		// 		}
		// 	}
		// 	for x in -15..16 {
		// 		for z in -10..200 {
		// 			sub_grid.add_voxel(I16Vec3::new(x, -(z + 10) / 3, z), voxels::Voxel{ color: [(x * 32 % 255 * 0) as u8, 0, (z * 32 % 255) as u8, 1], mass: 100 });
		// 		}
		// 	}
		// 	for y in 0..15 {
		// 		for z in -10..200 {
		// 			sub_grid.add_voxel(I16Vec3::new(-16, -(z + 10) / 3 + y, z), voxels::Voxel{ color: [0, 0, (z * 32 % 255) as u8, 1], mass: 100 });
		// 			sub_grid.add_voxel(I16Vec3::new(16, -(z + 10) / 3 + y, z), voxels::Voxel{ color: [0, 0, (z * 32 % 255) as u8, 1], mass: 100 });
		// 		}
		// 	}
		// 	physics_body.is_static = true;
		// 	physics_body.pose.translation.y += 2.0;
		// }
		// ------------------------------ Ball ------------------------------
		for x in -1..2 {
			for y in -1..2 {
				for z in -1..2 {
					let r = 4;
					let physics_body_id = physics_engine.add_physics_body();
					let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
					physics_body.pose.translation.y += (y as f32) * (r as f32) * 2.0 + 7.0 + 40.0;
					physics_body.pose.translation.z += (z as f32) * (r as f32) * 2.0 + 3.0 + y as f32;
					physics_body.pose.translation.x += (x as f32) * (r as f32) * 2.0;
					State::make_ball(physics_body, r);
				}
			}
		}
		// for x in -1..0 {
		// 	for y in -1..0 {
		// 		for z in -1..0 {
		// 			let r = 4;
		// 			let physics_body_id = physics_engine.add_physics_body();
		// 			let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 			physics_body.pose.translation.y += (y as f32) * (r as f32) * 2.0 + 7.0 +620.0;
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
		// 				physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, 1), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(1, 0, 0), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(1, 0, 1), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.pose.translation.x += (x * 5) as f32;
		// 				physics_body.pose.translation.y += (y * 5) as f32;
		// 				physics_body.pose.translation.z -= (z * 5) as f32;
		// 				physics_body.is_static = true;
		// 			}
		// 			{
		// 				physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.pose.translation.x += (x * 5) as f32 - 0.4;;
		// 				physics_body.pose.translation.y += (y * 5) as f32 + 0.8 + (y as f32) * 0.05;
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
		// 	let sub_grid_id = physics_body.add_sub_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let sub_grid = physics_body.sub_grid_mut(sub_grid_id).unwrap();
		// 	sub_grid.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, 1.0, 0.0, 1.0], mass: 1.0 });
		// 	physics_body.is_static = true;
		// 	physics_body.pose.translation.x = 4.0;
		// }
		// {
		// 	let physics_body_id = physics_engine.add_physics_body();
		// 	let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
		// 	let sub_grid_id = physics_body.add_sub_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
		// 	let sub_grid = physics_body.sub_grid_mut(sub_grid_id).unwrap();
		// 	sub_grid.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [1.0, 0.0, 0.0, 1.0], mass: 1.0 });
		// 	physics_body.pose.translation.y = 2.0;
		// 	physics_body.pose.translation.x = 4.0;
		// }

		{
			let physics_body_id = physics_engine.add_physics_body();
			let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
			let world_generator = WorldGenerator::new(2);
			for x in -10..11 {
				for z in -10..11 {
					let sub_grid_id = physics_body.add_grid(Pose::new(
						Vec3::new(
							(x * WorldGenerator::CHUNK_SIZE as i32) as f32,
							0.0,
							(z * WorldGenerator::CHUNK_SIZE as i32) as f32
						),
						Quat::IDENTITY
					));
					let sub_grid = physics_body.grid_mut(sub_grid_id).unwrap();
					world_generator.create_chunk(IVec2::new(x, z), sub_grid);
				}
			}
			physics_body.is_static = true;
			physics_body.pose.translation.y = -10.0;
		}

		Ok(Self {
			renderer,
			mouse_captured: false,
			physics_engine,
			ecs,
			player_id,
			leaky_bucket: 0.0,
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {

		// for physics_body in self.physics_engine.physics_bodies() {
			// for sub_grid in physics_body.sub_grids() {
			// 	sub_grid.get_voxels().render_debug(&(physics_body.pose * sub_grid.pose));
			// }
			// physics_body.render_debug_inertia_box();
		// }
		if let Some(player_camera) = self.ecs.get_component(self.player_id) {
			let mut rendering_meshes: Vec<(Arc<mesh::Mesh>, Mat4)> = vec![];
			{
				let _zone = span!("Collect Meshes");
				for physics_body in self.physics_engine.physics_bodies() {
					rendering_meshes.extend(physics_body.get_rendering_meshes(&self.renderer.device, &player_camera));
				}
			}
			let _zone = span!("Render");
			return self.renderer.render(&player_camera.build_view_projection_matrix(), &rendering_meshes);
		} else {
			println!("Error: could not find player camera!");
			return Ok(());
		}
	}
}
