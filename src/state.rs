use std::sync::Arc;

use glam::{IVec3, Mat3, Mat4, Quat, Vec3};
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::Window};

use crate::{camera, entity, gpu_objects::mesh, renderer::Renderer, voxels::{Voxel, Voxels}};

pub struct State {
	pub renderer: Renderer,
	pub camera: camera::Camera,
	pub camera_controller: camera::CameraController,
	pub entities: Vec<entity::Entity>,
	pub meshes: Vec<(mesh::Mesh, u32)>,
	pub regenerate_meshes: bool,
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
		// for id_1 in 0..(self.entities.len()-1) {
		// 	for id_2 in (id_1+1)..(self.entities.len()) {
		// 		let [entity_1, entity_2] = self.entities.get_disjoint_mut([id_1, id_2]).unwrap();
		// 		let vec_1_to_2 = (entity_2.orientation * entity_2.voxels.center_of_mass() + entity_2.position) - (entity_1.orientation * entity_1.voxels.center_of_mass() + entity_1.position);
		// 		let p = 0.00001 * 1.0/60.0 * entity_1.voxels.mass() * entity_2.voxels.mass() / vec_1_to_2.length_squared();
		// 		entity_1.momentum += p * vec_1_to_2;
		// 		entity_2.momentum += p * -vec_1_to_2;
		// 	}
		// }
		// for id_1 in 0..(self.entities.len()-1) {
		// 	for id_2 in (id_1+1)..(self.entities.len()) {
		// 		let [mut entity_1, mut entity_2] = self.entities.get_disjoint_mut([id_1, id_2]).unwrap();
		// 		if entity_1.voxels.get_voxels().len() > entity_2.voxels.get_voxels().len() {
		// 			std::mem::swap(&mut entity_1, &mut entity_2);
		// 		}
		// 		let from_1_to_2 = Mat4::from_quat(entity_2.orientation.inverse()) * Mat4::from_translation(entity_1.position - entity_2.position) * Mat4::from_quat(entity_1.orientation);
		// 		for voxel in entity_1.voxels.get_voxels() {
		// 			let other_gird_voxel_center_pos = (from_1_to_2 * (voxel.0.as_vec3() + Vec3::new(0.5, 0.5, 0.5)).extend(1.0)).truncate();
		// 			let vec_to_check = [
		// 				Vec3::new(0.5, 0.5, 0.5),
		// 				Vec3::new(-0.5, 0.5, 0.5),
		// 				Vec3::new(0.5, -0.5, 0.5),
		// 				Vec3::new(0.5, 0.5, -0.5),
		// 				Vec3::new(-0.5, -0.5, 0.5),
		// 				Vec3::new(-0.5, 0.5, -0.5),
		// 				Vec3::new(0.5, -0.5, -0.5),
		// 				Vec3::new(-0.5, -0.5, -0.5),
		// 			];
		// 			vec_to_check.iter().for_each(|vec| {
		// 				match entity_2.voxels.get_voxel((other_gird_voxel_center_pos + vec).as_ivec3()) {
		// 					Some(_) => {
		// 						entity_1.momentum += 0.0001 * entity_1.voxels.mass() * (Mat3::from_quat(entity_2.orientation) * -vec);
		// 						entity_2.momentum += 0.0001 * entity_2.voxels.mass() * (Mat3::from_quat(entity_2.orientation) * vec);
		// 					},
		// 					None => {},
		// 				};
		// 			});

		// 		}
		// 	}
		// }
		// update pos
		for entity in self.entities.iter_mut() {
			entity.position += entity.momentum / entity.voxels.mass();
			let com = entity.voxels.center_of_mass();
			entity.position += entity.orientation * com;
			let rotational_inertia = Mat3::from_quat(entity.orientation) * entity.voxels.rotational_inertia() * Mat3::from_quat(entity.orientation.inverse());
			let rotationa_velocity = rotational_inertia.inverse() * entity.angular_momentum;
			entity.orientation = Quat::from_scaled_axis(rotationa_velocity * (1./60.)) * entity.orientation;
			entity.position -= entity.orientation * com;
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
		});

		Ok(Self {
			renderer,
			camera,
			camera_controller,
			entities,
			meshes: vec![],
			regenerate_meshes: true,
		})
	}

	pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
		if self.regenerate_meshes {
			self.regenerate_meshes = false;
			for entity in self.entities.iter() {
				use mesh::GetMesh;
				self.meshes.push((entity.voxels.get_mesh(&self.renderer.device), entity.id));
			}
		}

		let mut rendering_meshes: Vec<(&mesh::Mesh, Mat4)> = vec![];

		for mesh in self.meshes.iter() {
			let entity = &self.entities[mesh.1 as usize];
			let matrix = Mat4::from_translation(entity.position) * Mat4::from_quat(entity.orientation);
			rendering_meshes.push(( &mesh.0, matrix ));
		}

		self.renderer.render(&self.camera.build_view_projection_matrix(), &rendering_meshes)
	}
}
