use std::sync::Arc;

use glam::{IVec3, Mat3, Mat4, Quat, Vec3};
use winit::{event_loop::ActiveEventLoop, keyboard::KeyCode, window::Window};

use crate::{camera, entity, gpu_objects::{matrix, mesh, texture}, voxels::{Voxel, Voxels}};

pub struct State {
    pub surface: wgpu::Surface<'static>,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub config: wgpu::SurfaceConfiguration,
    pub is_surface_configured: bool,
    pub window: Arc<Window>,
    pub render_pipeline: wgpu::RenderPipeline,
    pub camera: camera::Camera,
    pub camera_controller: camera::CameraController,
    pub camera_uniform: matrix::MatrixUniform,
    pub camera_buffer: wgpu::Buffer,
    pub camera_bind_group: wgpu::BindGroup,
    pub depth_texture: texture::Texture,
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
        self.camera_uniform = matrix::MatrixUniform::from_mat4(self.camera.build_view_projection_matrix());
        self.queue.write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[self.camera_uniform]));
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
        if width > 0 && height > 0 {
			let max = 2048;
			if width < height {
				self.config.width = (height.min(max) * width) / height;
				self.config.height = height.min(max);
			} else {
				self.config.width = width.min(max);
				self.config.height = (width.min(max) * height) / width;
			}

            self.is_surface_configured = true;
            self.camera.aspect = self.config.width as f32 / self.config.height as f32;
            self.surface.configure(&self.device, &self.config);
            self.depth_texture =
                texture::Texture::create_depth_texture(&self.device, &self.config, "depth_texture");
        }
    }

    pub async fn new(window: Arc<Window>) -> anyhow::Result<State> {
        let mut size = window.inner_size();
		let max = 2048;
		size.width = size.width.max(1);
		size.height = size.height.max(1);
		if size.height > size.width {
			size.width = (size.height.min(max) * size.width) / size.height;
			size.height = size.height.min(max);
		} else {
			size.width = size.width.min(max);
			size.height = (size.width.min(max) * size.height) / size.width;
		}
        // The instance is a handle to our GPU
        // BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            #[cfg(not(target_arch = "wasm32"))]
            backends: wgpu::Backends::PRIMARY,
            #[cfg(target_arch = "wasm32")]
            backends: wgpu::Backends::GL,
            ..Default::default()
        });

        let surface = instance.create_surface(window.clone()).unwrap();

        let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await?;

        let (device, queue) = adapter.request_device(&wgpu::DeviceDescriptor {
                label: None,
                required_features: wgpu::Features::empty(),
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                // WebGL doesn't support all of wgpu's features, so if
                // we're building for the web we'll have to disable some.
                required_limits: if cfg!(target_arch = "wasm32") { wgpu::Limits::downlevel_webgl2_defaults() } else { wgpu::Limits::default() },
                memory_hints: Default::default(),
                trace: wgpu::Trace::Off,
            })
            .await?;

        let surface_caps = surface.get_capabilities(&adapter);
        // Shader code in this tutorial assumes an sRGB surface texture. Using a different
        // one will result in all the colors coming out darker. If you want to support non
        // sRGB surfaces, you'll need to account for that when drawing to the frame.
        let surface_format = surface_caps.formats.iter().find(|f| f.is_srgb()).copied().unwrap_or(surface_caps.formats[0]);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: surface_caps.present_modes[0],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let camera = camera::Camera {
			position: Vec3::ZERO,
			yaw: 0.0,
			pitch: 0.0,
            aspect: config.width as f32 / config.height as f32,
            fovy: 45.0,
            znear: 0.1,
            zfar: 500.0,
        };

        let camera_uniform = matrix::MatrixUniform::from_mat4(camera.build_view_projection_matrix());

		let camera_buffer = matrix::MatrixUniform::get_buffer(&device, 0);

        let camera_controller = camera::CameraController::new(0.1, 0.05);

        let depth_texture = texture::Texture::create_depth_texture(&device, &config, "depth_texture");

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

        let render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Render Pipeline Layout"),
            bind_group_layouts: &[&camera_buffer.2, &matrix::MatrixUniform::get_bind_group_layout(&device, 0)],
            push_constant_ranges: &[],
        });
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),

            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[<mesh::MeshVertex as mesh::Vertex>::desc()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList, // 1.
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw, // 2.
                cull_mode: Some(wgpu::Face::Back),
                // Setting this to anything other than Fill requires Features::NON_FILL_POLYGON_MODE
                polygon_mode: wgpu::PolygonMode::Fill,
                // Requires Features::DEPTH_CLIP_CONTROL
                unclipped_depth: false,
                // Requires Features::CONSERVATIVE_RASTERIZATION
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: texture::Texture::DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less, // 1.
                stencil: wgpu::StencilState::default(),     // 2.
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: 1,                         // 2.
                mask: !0,                         // 3.
                alpha_to_coverage_enabled: false, // 4.
            },
            multiview: None, // 5.
            cache: None,     // 6.
        });

        Ok(Self {
            surface,
            device,
            queue,
            config,
            is_surface_configured: false,
            window,
            render_pipeline,
            camera,
            camera_controller,
            camera_uniform,
            camera_buffer: camera_buffer.0,
            camera_bind_group: camera_buffer.1,
            depth_texture,
            entities,
            meshes: vec![],
			regenerate_meshes: true,
        })
    }

    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.window.request_redraw();

        // We can't render unless the surface is configured
        if !self.is_surface_configured {
            return Ok(());
        }

        let output = self.surface.get_current_texture()?;

        let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Render Encoder") });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.1, g: 0.2, b: 0.3, a: 1.0 }), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_texture.view,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
                    stencil_ops: None,
                }),
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.render_pipeline);
			if self.regenerate_meshes {
				self.regenerate_meshes = false;
				for entity in self.entities.iter() {
					use mesh::GetMesh;
					self.meshes.push((entity.voxels.get_mesh(&self.device), entity.id));
				}
			}

			for mesh in self.meshes.iter() {
				let entity = &self.entities[mesh.1 as usize];

				let matrix = Mat4::from_translation(entity.position) * Mat4::from_quat(entity.orientation);
				self.queue.write_buffer(&mesh.0.matrix_buffer, 0, bytemuck::cast_slice(&[matrix::MatrixUniform::from_mat4(matrix)]));

				use mesh::DrawMesh;
				render_pass.draw_mesh(&mesh.0, &self.camera_bind_group);
			}
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
