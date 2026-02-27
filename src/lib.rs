use std::{sync::Arc};
mod texture;
mod mesh;
mod voxels;
mod resources;
mod entity;
mod camera;

use glam::{IVec3, Mat3, Mat4, Quat, Vec3, Vec4};
use winit::{
    application::ApplicationHandler,
    event::*,
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::Window,
};

use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]

struct Vertex {
    position: [f32; 3],
    color: [f32; 4],
}

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: Mat4 = Mat4::from_cols(
    Vec4::new(1.0, 0.0, 0.0, 0.0),
    Vec4::new(0.0, 1.0, 0.0, 0.0),
    Vec4::new(0.0, 0.0, 0.5, 0.0),
    Vec4::new(0.0, 0.0, 0.5, 1.0),
);

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

impl CameraUniform {
    fn new() -> Self {
        Self { view_proj: Mat4::IDENTITY.to_cols_array_2d() }
    }

    fn update_view_proj(&mut self, camera: &camera::Camera) {
        self.view_proj = camera.build_view_projection_matrix().to_cols_array_2d();
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct MatrixUniform {
    matrix: [[f32; 4]; 4],
}

pub fn get_matrix_buffer(device: &wgpu::Device) -> (wgpu::Buffer, wgpu::BindGroup) {
	let matrix_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::VERTEX,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Uniform,
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("matrix_bind_group_layout"),
		});
	let matrix_buffer = device.create_buffer(&wgpu::BufferDescriptor {
		label: Some("Matrix Buffer"),
		size: std::mem::size_of::<MatrixUniform>() as u64,
		usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
		mapped_at_creation: false,
	});
	let matrix_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
		layout: &matrix_bind_group_layout,
		entries: &[wgpu::BindGroupEntry {
			binding: 0,
			resource: matrix_buffer.as_entire_binding(),
		}],
		label: Some("matrix_bind_group"),
	});
	(matrix_buffer, matrix_bind_group)
}

pub struct State {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    is_surface_configured: bool,
    window: Arc<Window>,
    render_pipeline: wgpu::RenderPipeline,
    camera: camera::Camera,
    camera_controller: camera::CameraController,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    depth_texture: texture::Texture,
    entities: Vec<entity::Entity>,
    meshes: Vec<(mesh::Mesh, u32)>,
	regenerate_meshes: bool,
}

impl State {
    fn handle_key(&mut self, event_loop: &ActiveEventLoop, code: KeyCode, is_pressed: bool) {
        if code == KeyCode::Escape && is_pressed {
            event_loop.exit();
        } else {
            self.camera_controller.handle_key(code, is_pressed);
        }
    }

    fn update(&mut self) {
        self.camera_controller.update_camera(&mut self.camera);
        self.camera_uniform.update_view_proj(&self.camera);
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

	fn resize(&mut self, width: u32, height: u32) {
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

    async fn new(window: Arc<Window>) -> anyhow::Result<State> {
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

        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await?;

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
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

        let mut camera_uniform = CameraUniform::new();
        camera_uniform.update_view_proj(&camera);

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
            label: Some("camera_bind_group_layout"),
        });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: camera_buffer.as_entire_binding() }],
            label: Some("camera_bind_group"),
        });

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
				let mut voxels = voxels::Voxels::new();
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
				let mut voxels = voxels::Voxels::new();
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
				let mut voxels = voxels::Voxels::new();
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
				let mut voxels = voxels::Voxels::new();
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

		let matrix_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
				entries: &[wgpu::BindGroupLayoutEntry {
					binding: 0,
					visibility: wgpu::ShaderStages::VERTEX,
					ty: wgpu::BindingType::Buffer {
						ty: wgpu::BufferBindingType::Uniform,
						has_dynamic_offset: false,
						min_binding_size: None,
					},
					count: None,
				}],
				label: Some("matrix_bind_group_layout"),
			});

        let render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Render Pipeline Layout"),
            bind_group_layouts: &[&camera_bind_group_layout, &matrix_bind_group_layout],
            push_constant_ranges: &[],
        });
        use crate::mesh::Vertex;
        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),

            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[mesh::MeshVertex::desc()],
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
            camera_buffer,
            camera_bind_group,
            depth_texture,
            entities,
            meshes: vec![],
			regenerate_meshes: true,
        })
    }

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
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
					self.meshes.push((entity.voxels.get_mesh(&self.device), entity.id));
				}
			}

			for mesh in self.meshes.iter() {
				let entity = &self.entities[mesh.1 as usize];

				let matrix = Mat4::from_translation(entity.position) * Mat4::from_quat(entity.orientation);
				self.queue.write_buffer(&mesh.0.matrix_buffer, 0, bytemuck::cast_slice(&[MatrixUniform { matrix: matrix.to_cols_array_2d() }]));

				use mesh::DrawMesh;
				render_pass.draw_mesh(&mesh.0, &self.camera_bind_group);
			}
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}

pub struct App {
    #[cfg(target_arch = "wasm32")]
    proxy: Option<winit::event_loop::EventLoopProxy<State>>,
    state: Option<State>,
}

impl App {
    pub fn new(#[cfg(target_arch = "wasm32")] event_loop: &EventLoop<State>) -> Self {
        #[cfg(target_arch = "wasm32")]
        let proxy = Some(event_loop.create_proxy());
        Self {
            state: None,
            #[cfg(target_arch = "wasm32")]
            proxy,
        }
    }
}

impl ApplicationHandler<State> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        #[allow(unused_mut)]
        let mut window_attributes = Window::default_attributes();

        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowAttributesExtWebSys;

            const CANVAS_ID: &str = "canvas";

            let window = wgpu::web_sys::window().unwrap_throw();
            let document = window.document().unwrap_throw();
            let canvas = document.get_element_by_id(CANVAS_ID).unwrap_throw();
            let html_canvas_element = canvas.unchecked_into();
            window_attributes = window_attributes.with_canvas(Some(html_canvas_element));
        }

        let window = Arc::new(event_loop.create_window(window_attributes).unwrap());

        #[cfg(not(target_arch = "wasm32"))]
        {
            // If we are not on web we can use pollster to
            // await the
            self.state = Some(pollster::block_on(State::new(window)).unwrap());
        }

        #[cfg(target_arch = "wasm32")]
        {
            // Run the future asynchronously and use the
            // proxy to send the results to the event loop
            if let Some(proxy) = self.proxy.take() {
                wasm_bindgen_futures::spawn_local(async move {
                    assert!(proxy.send_event(State::new(window).await.expect("Unable to create canvas!!!")).is_ok())
                });
            }
        }
    }

    #[allow(unused_mut)]
    fn user_event(&mut self, _event_loop: &ActiveEventLoop, mut event: State) {
        // This is where proxy.send_event() ends up
        #[cfg(target_arch = "wasm32")]
        {
            event.window.request_redraw();
            event.resize(event.window.inner_size().width, event.window.inner_size().height);
        }

        self.state = Some(event);
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _window_id: winit::window::WindowId, event: WindowEvent) {
        let state = match &mut self.state {
            Some(canvas) => canvas,
            None => return,
        };

        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => state.resize(size.width, size.height),
            WindowEvent::RedrawRequested => {
                state.update();
                match state.render() {
                    Ok(_) => {}
                    // Reconfigure the surface if it's lost or outdated
                    Err(wgpu::SurfaceError::Lost | wgpu::SurfaceError::Outdated) => {
                        let size = state.window.inner_size();
                        state.resize(size.width, size.height);
                    }
                    Err(e) => {
                        log::error!("Unable to render {}", e);
                    }
                }
            }
            WindowEvent::KeyboardInput { event: KeyEvent { physical_key: PhysicalKey::Code(code), state: key_state, .. }, .. } => {
                state.handle_key(event_loop, code, key_state.is_pressed())
            }
            _ => {}
        }
    }
}

pub fn run() -> anyhow::Result<()> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        env_logger::init();
    }
    #[cfg(target_arch = "wasm32")]
    {
        console_log::init_with_level(log::Level::Info).unwrap_throw();
    }

    let event_loop = EventLoop::with_user_event().build()?;
    let mut app = App::new(
        #[cfg(target_arch = "wasm32")]
        &event_loop,
    );
    event_loop.run_app(&mut app)?;

    Ok(())
}

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

use crate::{mesh::GetMesh, voxels::Voxel};

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub fn run_web() -> Result<(), JsValue> {
    console_error_panic_hook::set_once();

    run().unwrap_throw();
    Ok(())
}
