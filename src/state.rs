use std::{f32, sync::Arc};

use glam::{IVec3, Mat4, Quat, Vec3};
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::{CursorGrabMode, Window}};

use crate::{camera, entity_component_system, gpu_objects::mesh, physics::physics_engine::PhysicsEngine, pose::Pose, renderer::Renderer, voxels};

pub struct State {
	pub renderer: Renderer,
	pub mouse_captured: bool,
	pub physics_engine: PhysicsEngine,
	pub ecs: entity_component_system::EntityComponentSystem,
	pub player_id: u32,
}

impl State {
	pub fn handle_key(&mut self, _event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
		if code == KeyCode::Escape && is_pressed {
			self.set_mouse_captured(false);
		} else {
			self.ecs.run_on_single_component_mut::<camera::CameraController, _>(self.player_id, |_entity_id, player_camera_controller|
				{ player_camera_controller.handle_key(code, is_pressed); }
			);
		}
	}

	pub fn set_mouse_captured(&mut self, captured: bool) {
		if self.mouse_captured == captured {
			return;
		}
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
		self.ecs.run_on_components_pair_mut::<camera::CameraController, camera::Camera, _>(|_entity_id, camera_controller, camera|
			camera::CameraController::update_camera(camera_controller, camera, dt)
		);
		self.physics_engine.update(1.0/100.0);
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
		let renderer = Renderer::new(window).await?;
		let mut physics_engine = PhysicsEngine::new();
		let mut ecs = entity_component_system::EntityComponentSystem::new();

		// create player entity
		let player_id = ecs.add_entity();
		ecs.add_component_to_entity(player_id, camera::Camera {
			position: Vec3::new(20.0, 0.0, 0.0),
			yaw: f32::consts::PI / 2.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			zfar: 500.0,
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
		{
			let physics_body_id = physics_engine.add_physics_body();
			let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
			let sub_grid_id = physics_body.add_sub_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
			let sub_grid = physics_body.sub_grid_mut(sub_grid_id).unwrap();
			for x in -15..16 {
				for y in 0..20 {
					sub_grid.add_voxel(IVec3::new(x, y, -10), voxels::Voxel{ color: [(x % 10) as f32 / 9.0, 0.0, (y % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			for x in -15..16 {
				for z in -10..200 {
					sub_grid.add_voxel(IVec3::new(x, -(z + 10) / 4, z), voxels::Voxel{ color: [(x + 15) as f32 / 30.0, 0.0, ((z + 10) % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			for y in 0..15 {
				for z in -10..200 {
					sub_grid.add_voxel(IVec3::new(-16, -(z + 10) / 4 + y, z), voxels::Voxel{ color: [0.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
					sub_grid.add_voxel(IVec3::new(16, -(z + 10) / 4 + y, z), voxels::Voxel{ color: [0.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			physics_body.is_static = true;
			physics_body.pose.translation.y += 2.0;
		}
		// ------------------------------ Ball ------------------------------
		for x in -1..2 {
			for y in -1..4 {
				for z in -1..2 {
					let r = 4;

					let physics_body_id = physics_engine.add_physics_body();
					let physics_body = physics_engine.physics_body_mut(physics_body_id).unwrap();
					physics_body.pose.translation.y += (y as f32) * (r as f32) * 2.0 + 7.0 + 20.0;
					physics_body.pose.translation.z += (z as f32) * (r as f32) * 2.0 + 3.0 + y as f32;
					physics_body.pose.translation.x += (x as f32) * (r as f32) * 2.0;
					{
						let sub_grid_id = physics_body.add_sub_grid(Pose::new(Vec3::ZERO, Quat::IDENTITY));
						let sub_grid = physics_body.sub_grid_mut(sub_grid_id).unwrap();

						for x in -r..r + 1 {
							for y in -0..r + 1 {
								for z in -r..r + 1 {
									if IVec3::new(x, y, z).length_squared() as f32 <= (r as f32 - 0.5).powf(2.0)  {
										sub_grid.add_voxel(
											IVec3::new(x, y + 2, z),
											voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 }
										);
									}
								}
							}
						}
					}
					{
						let sub_grid_id = physics_body.add_sub_grid(Pose::new(Vec3::new(-0.707 + 0.5, 0.0, 0.707 - 0.5), Quat::from_rotation_y(f32::consts::PI/4.0)));
						let sub_grid = physics_body.sub_grid_mut(sub_grid_id).unwrap();
						let r = 4;
						for x in -r..r + 1 {
							for y in -r..0 {
								for z in -r..r + 1 {
									if IVec3::new(x, y, z).length_squared() as f32 <= (r as f32 - 0.5).powf(2.0)  {
										sub_grid.add_voxel(
											IVec3::new(x, y + 2, z),
											voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 }
										);
									}
								}
							}
						}
					}
				}
			}
		}

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

		Ok(Self {
			renderer,
			mouse_captured: false,
			physics_engine,
			ecs,
			player_id,
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
		let mut rendering_meshes: Vec<(Arc<mesh::Mesh>, Mat4)> = vec![];

		if let Some(player_camera) = self.ecs.get_component(self.player_id) {
			for physics_body in self.physics_engine.physics_bodies() {
				rendering_meshes.extend(physics_body.get_rendering_meshes(&self.renderer.device, &player_camera));
			}
			return self.renderer.render(&player_camera.build_view_projection_matrix(), &rendering_meshes);
		} else {
			println!("Error: could not find player camera!");
			return Ok(());
		}
	}
}
