use std::sync::Arc;

use glam::{IVec3, Mat3, Mat4, Quat, Vec3};
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::Window};

use crate::{camera, entity, gpu_objects::mesh, renderer::Renderer, voxels::{Voxel, Voxels}};

pub struct State {
	pub renderer: Renderer,
	pub camera: camera::Camera,
	pub camera_controller: camera::CameraController,
	pub entities: Vec<entity::Entity>,
}

impl State {
	pub fn handle_key(&mut self, event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
		if code == KeyCode::Escape && is_pressed {
			event_loop.exit();
		} else {
			self.camera_controller.handle_key(code, is_pressed);
		}
	}

	pub fn update(&mut self) {
		self.camera_controller.update_camera(&mut self.camera);
		for entity in self.entities.iter_mut() {
			entity.update();
		}
	}

	pub fn resize(&mut self, width: u32, height: u32) {
		self.renderer.resize(width, height);
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
		let renderer = Renderer::new(window).await?;

		let camera = camera::Camera {
			position: Vec3::ZERO,
			yaw: 0.0,
			pitch: 0.0,
			aspect: renderer.config.width as f32 / renderer.config.height as f32,
			fovy: 45.0,
			znear: 0.1,
			zfar: 500.0,
		};

		let camera_controller = camera::CameraController::new(0.1, 0.05);

		let mut entities: Vec<entity::Entity> = vec![];

		entities.push(entity::Entity{
			position: Vec3::new(4., 0., 0.),
			orientation: Quat::IDENTITY,
			momentum: Vec3::ZERO,
			angular_momentum: Vec3::ZERO,
			id: entities.len() as u32,
			voxels: {
				let mut voxels = Voxels::new();
				voxels.set_voxel(IVec3::new(0, 0, 0), Voxel{ color: [1.0, 0.0, 0.0, 1.0], mass: 1.0 });
				voxels.set_voxel(IVec3::new(0, 1, 0), Voxel{ color: [0.0, 1.0, 0.0, 1.0], mass: 1.0 });
				voxels.set_voxel(IVec3::new(0, 0, 2), Voxel{ color: [0.0, 0.0, 1.0, 1.0], mass: 1.0 });
				voxels.set_voxel(IVec3::new(1, 0, 2), Voxel{ color: [1.0, 1.0, 1.0, 1.0], mass: 1.0 });
				voxels.set_voxel(IVec3::new(1, 0, 0), Voxel{ color: [1.0, 0.0, 1.0, 1.0], mass: 1.0 });
				for x in 0..15 {
					for z in 0..15 {
						for y in 0..3 {
							voxels.set_voxel(IVec3::new(x - 10, z/2 - (x*x)/9 + y, z + 10), Voxel{ color: [0.0, (x as f32)/20.0, (z as f32)/20.0, 1.0], mass: 1.0 });
						}
					}
				}
				voxels
			},
			mesh: None,
			update_mesh: true,
		});
		entities.push(entity::Entity{
			position: Vec3::new(5., 5., 5.),
			orientation: Quat::from_rotation_y(0.785),
			momentum: Vec3::ZERO,
			angular_momentum: Vec3::X * 600000.,
			id: entities.len() as u32,
			voxels: {
				let mut voxels = Voxels::new();
				for x in 0..6 {
					for y in 0..6 {
						for z in 0..6 {
							voxels.set_voxel(IVec3::new(x, y, z), Voxel{ color: [(x as f32)/6.0, (y as f32)/6.0, (z as f32)/6.0, 1.0], mass: 1.0 });
						}
					}
				}
				voxels.set_voxel(IVec3::new(6, 6, 6), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 5000.0 });
				voxels
			},
			mesh: None,
			update_mesh: true,
		});
		entities.push(entity::Entity{
			position: Vec3::new(0., -7., 0.),
			orientation: Quat::from_rotation_x(0.985),
			momentum: Vec3::ZERO,
			angular_momentum: Vec3::Z * 200.,
			id: entities.len() as u32,
			voxels: {
				let mut voxels = Voxels::new();
				for x in 0..1 {
					for y in -4..5 {
						for z in -4..5 {
							voxels.set_voxel(IVec3::new(x, y, z), Voxel{ color: [0.0, 0.4, 0.0, 1.0], mass: 1.0 });
						}
					}
				}
				voxels
			},
			mesh: None,
			update_mesh: true,
		});
		entities.push(entity::Entity{
			position: Vec3::new(0., 20., 0.),
			orientation: Quat::IDENTITY,
			momentum: -0.2 * Vec3::Y,
			angular_momentum: Vec3::Y * 1000. + Vec3::X * 40.,
			id: entities.len() as u32,
			voxels: {
				let mut voxels = Voxels::new();
				for x in -6..7 {
					for z in -6..7 {
						voxels.set_voxel(IVec3::new(x, 0, z), Voxel{ color: [1.0, (x + 6) as f32 / 13., (z + 6) as f32 / 13., 1.0], mass: 1.0 });
					}
				}
				voxels.set_voxel(IVec3::new(0, 1, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
				voxels.set_voxel(IVec3::new(0, 2, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
				voxels.set_voxel(IVec3::new(0, 3, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
				voxels.set_voxel(IVec3::new(0, 4, 0), Voxel{ color: [0.0, 0.0, 0.0, 1.0], mass: 0.01 });
				voxels
			},
			mesh: None,
			update_mesh: true,
		});

		Ok(Self {
			renderer,
			camera,
			camera_controller,
			entities,
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
