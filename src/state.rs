use std::sync::Arc;

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
		self.solver.solve(&mut self.entities, dt);
		// for collision in collisions {
		// 	println!("Collision: {0}-{1}, pos: {2}, norm: {3}", collision.id1, collision.id2, collision.pos, collision.normal);
		// }
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
		let renderer = Renderer::new(window).await?;

		let camera = camera::Camera {
			position: Vec3::new(0.0, 2.0, 0.0),
			yaw: 0.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			zfar: 500.0,
		};

		let camera_controller = camera::CameraController::new(3.0, 1.5, 0.0015);

		let mut entities: Vec<entity::Entity> = vec![];

		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.position.x = 6.0;
		// 	for x in 0..15 {
		// 		for z in 0..15 {
		// 			for y in 0..3 {
		// 				entity.add_voxel(IVec3::new(x - 10, z/2 - (x*x)/9 + y, z + 10), Voxel{ color: [0.0, (x as f32)/20.0, (z as f32)/20.0, 1.0], mass: 1.0 });
		// 			}
		// 		}
		// 	}
		// }
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.position.z = -4.0;
		// 	entity.position.x = -12.0;
		// 	entity.position.y = -5.0;
		// 	entity.angular_momentum.z = 50000.0;
		// 	for x in 0..6 {
		// 		for y in 0..6 {
		// 			for z in 0..6 {
		// 				entity.add_voxel(IVec3::new(x, y, z), Voxel{ color: [(x as f32)/6.0, (y as f32)/6.0, (z as f32)/6.0, 1.0], mass: 1.0 });
		// 			}
		// 		}
		// 	}
		// 	entity.add_voxel(IVec3::new(6, 6, 6), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 5000.0 });
		// }
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.position.z = -10.0;
		// 	entity.position.x = 4.0;
		// 	for x in 0..1 {
		// 		for y in -4..5 {
		// 			for z in -4..5 {
		// 				entity.add_voxel(IVec3::new(x, y, z), Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1.0 });
		// 			}
		// 		}
		// 	}
		// }
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.position.y = 12.0;
		// 	for x in -6..7 {
		// 		for z in -6..7 {
		// 			entity.add_voxel(IVec3::new(x, 0, z), Voxel{ color: [1.0, (x + 6) as f32 / 13., (z + 6) as f32 / 13., 1.0], mass: 1.0 });
		// 		}
		// 	}
		// 	entity.add_voxel(IVec3::new(0, 1, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
		// 	entity.add_voxel(IVec3::new(0, 2, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
		// 	entity.add_voxel(IVec3::new(0, 3, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
		// 	entity.add_voxel(IVec3::new(0, 4, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
		// 	entity.angular_momentum = Vec3::new(200., 2000., 0.);
		// }
		{
			entities.push(entity::Entity::new());
			let entity = entities.last_mut().unwrap();
			entity.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, 0.0, 1.0, 1.0], mass: 1.5 });
			entity.add_voxel(IVec3::new(1, 0, 0), voxels::Voxel{ color: [0.0, 0.0, 1.0, 1.0], mass: 1.5 });
			entity.add_voxel(IVec3::new(0, 0, 1), voxels::Voxel{ color: [0.0, 0.0, 1.0, 1.0], mass: 1.5 });
			entity.add_voxel(IVec3::new(1, 0, 1), voxels::Voxel{ color: [0.0, 0.0, 1.0, 1.0], mass: 1.5 });
			entity.position.y = 100.0;
			entity.position.z = -6.5;
			entity.position.x = -0.5;
			entity.velocity.y = -10.0;
			entity.orientation = Quat::from_rotation_z(0.1);
		}
		for x in -2..2 {
			for y in -2..1 {
				for z in -1..2 {
					entities.push(entity::Entity::new());
					let entity = entities.last_mut().unwrap();
					entity.add_voxel(IVec3::new(x, y, z), voxels::Voxel{ color: [x as f32 / 8.0 + 0.5, y as f32 / 8.0 + 0.5, z as f32 / 8.0 + 0.5, 1.0], mass: 1.0 });
					entity.position.z = -6.0;
				}
			}
		}
		{
			entities.push(entity::Entity::new());
			let entity = entities.last_mut().unwrap();
			entity.position.z = -6.0;
			for y in -3..4 {
				for z in -3..4 {
					entity.add_voxel(IVec3::new(-3, y, z), voxels::Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1000.0 });
				}
			}
			for y in -3..4 {
				for z in -3..4 {
					entity.add_voxel(IVec3::new(3, y, z), voxels::Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1000.0 });
				}
			}
			for x in -3..4 {
				for z in -3..4 {
					entity.add_voxel(IVec3::new(x, -3, z), voxels::Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1000.0 });
				}
			}
			for y in -3..4 {
				for x in -3..4 {
					entity.add_voxel(IVec3::new(x, y, -3), voxels::Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1000.0 });
				}
			}
		}

		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [1.0, 0.0, 0.0, 1.0], mass: 1.0 });
		// 	entity.position.z = -6.0;
		// }
		// {
		// 	entities.push(entity::Entity::new());
		// 	let entity = entities.last_mut().unwrap();
		// 	entity.add_voxel(IVec3::new(0, 0, 0), voxels::Voxel{ color: [0.0, 0.0, 1.0, 1.0], mass: 1.0 });
		// 	entity.position.z = -6.0;
		// 	entity.position.y = 0.9;
		// 	// entity.position.x = 0.9;
		// 	entity.orientation = Quat::from_rotation_z(0.7853981634);
		// }

		Ok(Self {
			renderer,
			camera,
			camera_controller,
			entities,
			mouse_captured: false,
			solver: physics::solver::Solver {
				collision_stiffness: 200.0,
			}
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
