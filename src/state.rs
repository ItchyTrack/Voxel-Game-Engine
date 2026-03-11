use std::{f32, sync::Arc};

use glam::{IVec3, Mat4, Vec3, Vec4};
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::{CursorGrabMode, Window}};

use crate::{camera, debug_draw, gpu_objects::mesh, physics, renderer::Renderer, voxels};

pub struct State {
	pub renderer: Renderer,
	pub camera: camera::Camera,
	pub camera_controller: camera::CameraController,
	pub physics_bodies: Vec<physics::physics_body::PhysicsBody>,
	pub mouse_captured: bool,
	pub solver: physics::solver::Solver,
}

impl State {
	pub fn handle_key(&mut self, _event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
		if code == KeyCode::Escape && is_pressed {
			self.set_mouse_captured(false);
		} else {
			self.camera_controller.handle_key(code, is_pressed);
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
		self.camera_controller.update_camera(&mut self.camera, dt);
		self.solver.solve(&mut self.physics_bodies, 1.0/100.0);
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
		let renderer = Renderer::new(window).await?;

		let camera = camera::Camera {
			position: Vec3::new(10.0, 0.0, 0.0),
			yaw: f32::consts::PI / 2.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			zfar: 500.0,
		};

		let camera_controller = camera::CameraController::new(20.0, 1.5, 0.0015);

		let mut physics_bodies: Vec<physics::physics_body::PhysicsBody> = vec![];

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
			physics_bodies.push(physics::physics_body::PhysicsBody::new());
			let physics_body = physics_bodies.last_mut().unwrap();
			for x in -15..16 {
				for y in 0..20 {
					physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(x, y, -10), voxels::Voxel{ color: [(x % 10) as f32 / 9.0, 0.0, (y % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			for x in -15..16 {
				for z in -10..200 {
					physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(x, -(z + 10) / 4, z), voxels::Voxel{ color: [(x + 15) as f32 / 30.0, 0.0, ((z + 10) % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			for y in 0..15 {
				for z in -10..200 {
					physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(-16, -(z + 10) / 4 + y, z), voxels::Voxel{ color: [0.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
					physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(16, -(z + 10) / 4 + y, z), voxels::Voxel{ color: [0.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			physics_body.is_static = true;
			physics_body.position.y += 2.0;
		}
		// ------------------------------ Ball ------------------------------
		for x in -1..2 {
			for y in -1..5 {
				for z in -1..2 {
					physics_bodies.push(physics::physics_body::PhysicsBody::new());
					let physics_body = physics_bodies.last_mut().unwrap();
					let r = 4;
					for x in -r..r + 1 {
						for y in -r..r + 1 {
							for z in -r..r + 1 {
								if IVec3::new(x, y, z).length_squared() as f32 <= (r as f32 - 0.5).powf(2.0)  {
									physics_body.sub_grids.first_mut().unwrap().add_voxel(
										IVec3::new(x, y + 2, z),
										voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 }
									);
								}
							}
						}
					}
					physics_body.position.y += (y as f32) * (r as f32) * 2.0 + 7.0 + 20.0;
					physics_body.position.z += (z as f32) * (r as f32) * 2.0 + 3.0 + y as f32;
					physics_body.position.x += (x as f32) * (r as f32) * 2.0;
				}
			}
		}


		// ------------------------------ Grid Tesing ------------------------------
		// for x in 0..5 {
		// 	for y in 0..5 {
		// 		for z in 0..5 {
		// 			{
		// 				physics_bodies.push(physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.add_voxel(IVec3::new(0, 0, 1), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.add_voxel(IVec3::new(1, 0, 0), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.add_voxel(IVec3::new(1, 0, 1), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.position.x += (x * 5) as f32;
		// 				physics_body.position.y += (y * 5) as f32;
		// 				physics_body.position.z -= (z * 5) as f32;
		// 				physics_body.is_static = true;
		// 			}
		// 			{
		// 				physics_bodies.push(physics_body::PhysicsBody::new());
		// 				let physics_body = physics_bodies.last_mut().unwrap();
		// 				physics_body.add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				physics_body.position.x += (x * 5) as f32 - 0.4;;
		// 				physics_body.position.y += (y * 5) as f32 + 0.8 + (y as f32) * 0.05;
		// 				physics_body.position.z -= (z * 5) as f32 + 0.4;
		// 				physics_body.orientation = Quat::from_rotation_z(z as f32 / 5.0) * Quat::from_rotation_x(x as f32 / 5.0);
		// 				physics_body.is_static = true;
		// 			}
		// 		}
		// 	}
		// }


		// {
		// 	physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 	let physics_body = physics_bodies.last_mut().unwrap();
		// 	physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, 0.0, 0.5, 1.0], mass: 1.0 });
		// 	// physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(1, 0, 0), voxels::Voxel{ color: [0.0, 0.0, 0.5, 1.0], mass: 1.0 });
		// 	// physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, 1), voxels::Voxel{ color: [0.0, 0.0, 0.5, 1.0], mass: 1.0 });
		// 	// physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(-1, 0, 0), voxels::Voxel{ color: [0.0, 0.0, 0.5, 1.0], mass: 1.0 });
		// 	// physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, -1), voxels::Voxel{ color: [0.0, 0.0, 0.5, 1.0], mass: 1.0 });
		// 	physics_body.position.y += 0.0;
		// 	physics_body.is_static = true;
		// }
		// {
		// 	physics_bodies.push(physics::physics_body::PhysicsBody::new());
		// 	let physics_body = physics_bodies.last_mut().unwrap();
		// 	physics_body.sub_grids.first_mut().unwrap().add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, 1.0, 0.5, 1.0], mass: 1.0 });
		// 	physics_body.position.y += 1.0;
		// 	physics_body.position.x += 0.5;
		// }


		Ok(Self {
			renderer,
			camera,
			camera_controller,
			physics_bodies,
			mouse_captured: false,
			solver: physics::solver::Solver::new(),
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
		let mut rendering_meshes: Vec<(Arc<mesh::Mesh>, Mat4)> = vec![];

		for physics_body in self.physics_bodies.iter_mut() {
			// physics_body.get_voxels().render_debug(physics_body.position + physics_body.orientation * physics_body.get_voxels_local_pos(), &physics_body.orientation);
			rendering_meshes.extend(physics_body.get_rendering_meshes(&self.renderer.device, &self.camera));
		}

		self.renderer.render(&self.camera.build_view_projection_matrix(), &rendering_meshes)
	}
}
