use std::{f32::consts::PI, sync::Arc};

use glam::{IVec3, Mat4, Quat, Vec3};
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::{CursorGrabMode, Window}};

use crate::{camera, entity, gpu_objects::mesh, physics::{self}, renderer::Renderer, voxels};

pub struct State {
	pub renderer: Renderer,
	pub camera: camera::Camera,
	pub camera_controller: camera::CameraController,
	pub entities: Vec<entity::Entity>,
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
		self.solver.solve(&mut self.entities, 1.0/60.0);
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
		let renderer = Renderer::new(window).await?;

		let camera = camera::Camera {
			position: Vec3::new(-20.0, 15.0, 10.0),
			yaw: -90.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			zfar: 500.0,
		};

		let camera_controller = camera::CameraController::new(40.0, 1.5, 0.0015);

		let mut entities: Vec<entity::Entity> = vec![];

		// ------------------------------ Static Box ------------------------------
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	for x in -2..3 {
		// 		for y in -14..-6 {
		// 			for z in -2..3 {
		// 				if (x as i32).abs() == 2 || (y as i32).abs() == 14 || (z as i32).abs() == 2 {
		// 					entity.add_voxel(IVec3::new(x, y+7, z), voxels::Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1.0 });
		// 				}
		// 			}
		// 		}
		// 	}
		// 	entity.position.z -= 8.0;
		// 	entity.position.y += 2.0;
		// 	entity.is_static = true;
		// }
		// ------------------------------ Cube Stack ------------------------------
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.5, 0.0, 0.5, 1.0], mass: 1.0 });
		// 	entity.position.z += -8.0;
		// 	entity.is_static = true;
		// }
		// {
		// 	for _x in 0..1 {
		// 		for y in 0..10 {
		// 			for _z in 0..1 {
		// 				entities.push(entity::Entity::new());
		// 				let entity = entities.last_mut().unwrap();
		// 				entity.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, (y % 4) as f32 / 4.0, 0.5, 1.0], mass: 1.0 });
		// 				// entity.position.x += y as f32 * 0.01;
		// 				entity.position.y += y as f32 * 1.0;
		// 				entity.position.y += 0.0;
		// 				entity.position.z -= 8.0;
		// 				entity.orientation = Quat::from_rotation_y(y as f32);
		// 			}
		// 		}
		// 	}
		// }
		// ------------------------------
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	for x in -20..21 {
		// 		for z in -20..21 {
		// 			entity.add_voxel(IVec3::new(x, 0, z), voxels::Voxel{ color: [0.0, (z + 20) as f32 / 40.0, (x + 20) as f32 / 40.0, 1.0], mass: 1.0 });
		// 		}
		// 	}
		// 	entity.position.y -= 4.0;
		// 	entity.orientation = Quat::from_rotation_z(0.2);
		// 	entity.is_static = true;
		// }

		// {
		// 	for _x in 0..1 {
		// 		for y in 0..8 {
		// 			for _z in 0..1 {
		// 				entities.push(entity::Entity::new());
		// 				let entity = entities.last_mut().unwrap();
		// 				entity.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, (y % 4) as f32 / 4.0, 0.5, 1.0], mass: 1.0 });
		// 				// entity.position.x += y as f32 * 0.01;
		// 				entity.position.z += y as f32 * 4.0;
		// 				entity.position.z -= 8.0;
		// 				// entity.orientation = Quat::from_rotation_y(y as f32);
		// 			}
		// 		}
		// 	}
		// }

		{
			entities.push(entity::Entity::new());
			let entity = entities.last_mut().unwrap();
			// for x in -15..16 {
			// 	for z in -10..0 {
			// 		entity.add_voxel(IVec3::new(x, -z, z), voxels::Voxel{ color: [(x % 10) as f32 / 9.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
			// 	}
			// }
			for x in -15..16 {
				for z in -10..200 {
					entity.add_voxel(IVec3::new(x, -(z + 10) / 4, z), voxels::Voxel{ color: [(x % 10) as f32 / 9.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			for y in 0..10 {
				for z in -10..200 {
					entity.add_voxel(IVec3::new(-16, -(z + 10) / 4 + y, z), voxels::Voxel{ color: [0.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
					entity.add_voxel(IVec3::new(16, -(z + 10) / 4 + y, z), voxels::Voxel{ color: [0.0, 0.0, (z % 10) as f32 / 9.0, 1.0], mass: 1.0 });
				}
			}
			entity.is_static = true;
			entity.position.y -= 2.0;
		}
		{
			entities.push(entity::Entity::new());
			let entity = entities.last_mut().unwrap();
			let r = 13;
			for x in -r..r + 1 {
				for y in -r..r + 1 {
					for z in -r..r + 1 {
						if IVec3::new(x, y, z).length_squared() as f32 <= (r as f32 - 0.5).powf(2.0)  {
							entity.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
						}
					}
				}
			}
			entity.position.y += 30.0;
		}
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	for x in -6..7 {
		// 		for y in -6..7 {
		// 			for z in -6..7 {
		// 				if IVec3::new(x, y, z).length_squared() <= 37 {
		// 					entity.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				}
		// 			}
		// 		}
		// 	}
		// 	entity.position.y += 60.0;
		// 	entity.position.x += 5.0;
		// }
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	for x in -6..7 {
		// 		for y in -6..7 {
		// 			for z in -6..7 {
		// 				if IVec3::new(x, y, z).length_squared() <= 37 {
		// 					entity.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				}
		// 			}
		// 		}
		// 	}
		// 	entity.position.y += 90.0;
		// 	entity.position.x -= 5.0;
		// }

		// ------------------------------ Grid Tesing ------------------------------
		// for x in 0..5 {
		// 	for y in 0..5 {
		// 		for z in 0..5 {
		// 			{
		// 				entities.push(entity::Entity::new());
		// 				let entity = entities.last_mut().unwrap();
		// 				entity.add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				entity.position.x += (x * 5) as f32;
		// 				entity.position.y += (y * 5) as f32;
		// 				entity.position.z -= (z * 5) as f32;
		// 				entity.is_static = true;
		// 			}
		// 			{
		// 				entities.push(entity::Entity::new());
		// 				let entity = entities.last_mut().unwrap();
		// 				entity.add_voxel(IVec3::ZERO, voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
		// 				entity.position.x += (x * 5) as f32;
		// 				entity.position.y += (y * 5) as f32 + 0.8 + (y as f32) * 0.05;
		// 				entity.position.z -= (z * 5) as f32 + 0.8;
		// 				entity.orientation = Quat::from_rotation_z(z as f32 / 5.0) * Quat::from_rotation_x(x as f32 / 5.0);
		// 				entity.is_static = true;
		// 			}
		// 		}
		// 	}
		// }


		Ok(Self {
			renderer,
			camera,
			camera_controller,
			entities,
			mouse_captured: false,
			solver: physics::solver::Solver::new(),
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
		let mut rendering_meshes: Vec<(&mesh::Mesh, Mat4)> = vec![];

		for entity in self.entities.iter_mut() {
			rendering_meshes.extend(entity.get_rendering_meshes(&self.renderer.device, &self.camera));
		}

		self.renderer.render(&self.camera.build_view_projection_matrix(), &rendering_meshes)
	}
}
