use std::collections::HashMap;
use std::sync::Arc;
use std::vec;

use tracy_client::span;
use winit::{window::Window};

use crate::gpu_objects::matrix;
use crate::physics::bvh;
use crate::player::camera;
use crate::pose::Pose;
use crate::render::{crosshair_renderer, debug_draw_renderer, voxel_renderer};

pub struct Renderer {
	pub surface: wgpu::Surface<'static>,
	pub device: wgpu::Device,
	pub queue: wgpu::Queue,
	pub config: wgpu::SurfaceConfiguration,
	pub is_surface_configured: bool,
	pub window: Arc<Window>,
	pub imgui: imgui::Context,
	pub imgui_renderer: imgui_wgpu::Renderer,
	pub imgui_platform: imgui_winit_support::WinitPlatform,
	pub camera_buffer: wgpu::Buffer,
	pub camera_bind_group: wgpu::BindGroup,
	pub camera_transform_buffer: wgpu::Buffer,
	pub camera_transform_bind_group: wgpu::BindGroup,
	pub voxel_renderer: voxel_renderer::VoxelRenderer,
	pub crosshair_renderer: crosshair_renderer::CrosshairRenderer,
	pub debug_draw_renderer: debug_draw_renderer::DebugDrawRenderer,
	pub dt_avg: f32,
}

#[cfg(target_arch = "wasm32")]
const MAX_SCREEN_SIZE: u32 = 2048;

#[cfg(target_arch = "wasm32")]
const INSTANCE_BACKENDS: wgpu::Backends = wgpu::Backends::BROWSER_WEBGPU;
// #[cfg(all(not(target_arch = "wasm32"), target_os = "windows"))]
// const INSTANCE_BACKENDS: wgpu::Backends = wgpu::Backends::DX12;
// #[cfg(all(not(target_arch = "wasm32"), not(target_os = "windows")))]
// const INSTANCE_BACKENDS: wgpu::Backends = wgpu::Backends::PRIMARY;
#[cfg(not(target_arch = "wasm32"))]
const INSTANCE_BACKENDS: wgpu::Backends = wgpu::Backends::PRIMARY;
#[cfg(feature = "gpu_profiling")]
const INSTANCE_FLAGS: wgpu::InstanceFlags = wgpu::InstanceFlags::DEBUG;
#[cfg(not(feature = "gpu_profiling"))]
const INSTANCE_FLAGS: wgpu::InstanceFlags = wgpu::InstanceFlags::empty();

impl Renderer {
	pub fn resize(&mut self, width: u32, height: u32) {
		if width > 0 && height > 0 {
			#[cfg(target_arch = "wasm32")]
			if width < height {
				self.config.width = (height.min(MAX_SCREEN_SIZE) * width) / height;
				self.config.height = height.min(MAX_SCREEN_SIZE);
			} else {
				self.config.width = width.min(MAX_SCREEN_SIZE);
				self.config.height = (width.min(MAX_SCREEN_SIZE) * height) / width;
			}
			#[cfg(not(target_arch = "wasm32"))]
			{
				self.config.width = width;
				self.config.height = height;
			}

			self.is_surface_configured = true;
			self.surface.configure(&self.device, &self.config);
			self.voxel_renderer.resize(&self.device, &self.config);
			// self.depth_texture = texture::Texture::create_depth_texture(&self.device, &self.config, "depth_texture");
		}
	}

	pub async fn new(window: Arc<Window>) -> anyhow::Result<Renderer> {
		// ----------------------------- Window and instance setup -----------------------------
		let mut size = window.inner_size();
		size.width = size.width.max(1);
		size.height = size.height.max(1);
		#[cfg(target_arch = "wasm32")]
		if size.width < size.height {
			size.width = (size.height.min(MAX_SCREEN_SIZE) * size.width) / size.height;
			size.height = size.height.min(MAX_SCREEN_SIZE);
		} else {
			size.width = size.width.min(MAX_SCREEN_SIZE);
			size.height = (size.width.min(MAX_SCREEN_SIZE) * size.height) / size.width;
		}

		// The instance is a handle to our GPU
		// BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
		let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
			backends: INSTANCE_BACKENDS,
			flags: INSTANCE_FLAGS,
			..wgpu::InstanceDescriptor::new_without_display_handle()
		});

		let surface = instance.create_surface(window.clone()).unwrap();

		let adapter = instance.request_adapter(&wgpu::RequestAdapterOptions {
				power_preference: wgpu::PowerPreference::default(),
				compatible_surface: Some(&surface),
				force_fallback_adapter: false,
			}).await?;

		let (device, queue) = adapter.request_device(&wgpu::DeviceDescriptor {
				label: None,
				required_features: wgpu::Features::default(),
				experimental_features: wgpu::ExperimentalFeatures::disabled(),
				required_limits: wgpu::Limits {
					max_bind_groups: 5,
					..wgpu::Limits::defaults()
				},
				memory_hints: Default::default(),
				trace: wgpu::Trace::Off,
			})
			.await?;

		let surface_caps = surface.get_capabilities(&adapter);

		let surface_format = surface_caps.formats.iter().find(|f| f.is_srgb()).copied().unwrap_or(surface_caps.formats[0]);

		let config = wgpu::SurfaceConfiguration {
			usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
			format: surface_format,
			width: size.width,
			height: size.height,
			present_mode: if surface_caps.present_modes.contains(&wgpu::PresentMode::Fifo) { wgpu::PresentMode::Fifo } else { surface_caps.present_modes[0] },
			alpha_mode: surface_caps.alpha_modes[0],
			view_formats: vec![surface_format],
			desired_maximum_frame_latency: 2,
		};

		// ----------------------------- ImGui setup -----------------------------

		let mut imgui = imgui::Context::create();
		let mut imgui_platform = imgui_winit_support::WinitPlatform::new(&mut imgui);
		imgui_platform.attach_window(
			imgui.io_mut(),
			&*window,
			imgui_winit_support::HiDpiMode::Default,//Locked(1.0),
		);
		imgui.set_ini_filename(None);
		let renderer_config = imgui_wgpu::RendererConfig {
			texture_format: surface_format, // must match your surface
			..imgui_wgpu::RendererConfig::new()
		};
		let imgui_renderer = imgui_wgpu::Renderer::new(&mut imgui, &device, &queue, renderer_config);

		// ----------------------------- Camera setup -----------------------------

		let camera_buffer = matrix::MatrixUniform::get_buffer(&device, 0);
		let camera_transform_buffer = camera::CameraUniform::get_buffer(&device, 0);

		// ----------------------------- Renderers setup -----------------------------
		let voxel_renderer = voxel_renderer::VoxelRenderer::new(&device, &config, &camera_transform_buffer.2)?;
		let crosshair_renderer = crosshair_renderer::CrosshairRenderer::new(&device, &config)?;
		let debug_draw_renderer = debug_draw_renderer::DebugDrawRenderer::new(&device, &config, &camera_buffer.2)?;

		Ok(Self {
			surface,
			device,
			queue,
			config,
			is_surface_configured: false,
			window,
			imgui,
			imgui_renderer,
			imgui_platform,
			camera_buffer: camera_buffer.0,
			camera_bind_group: camera_buffer.1,
			camera_transform_buffer: camera_transform_buffer.0,
			camera_transform_bind_group: camera_transform_buffer.1,
			voxel_renderer,
			crosshair_renderer,
			debug_draw_renderer,
			dt_avg: 0.0,
		})
	}

	pub fn render(&mut self, camera: &camera::Camera, bvh: &bvh::BVH<(u32, u32, u32)>, gpu_grid_tree_id_to_id_poses: &HashMap<(u32, u32, u32), (u32, u32, Pose)>) -> Result<(), wgpu::CurrentSurfaceTexture> {
		self.window.request_redraw();
		tracy_client::plot!("64 tree bytes", self.voxel_renderer.packed_64_tree_dynamic_buffer.held_bytes() as f64);
		tracy_client::plot!("voxel data bytes", self.voxel_renderer.packed_voxel_data_dynamic_buffer.held_bytes() as f64);

		if !self.is_surface_configured { return Ok(()); }
		self.queue.write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[matrix::MatrixUniform::from_mat4(&camera.build_view_projection_matrix())]));
		self.queue.write_buffer(&self.camera_transform_buffer, 0, bytemuck::cast_slice(&[camera::CameraUniform::from_camera(camera)]));

		let output = {
			let _zone = span!("Finish Last Render");
			match self.surface.get_current_texture() {
				wgpu::CurrentSurfaceTexture::Success(surface_texture) => surface_texture,
				wgpu::CurrentSurfaceTexture::Suboptimal(surface_texture) => return Err(wgpu::CurrentSurfaceTexture::Suboptimal(surface_texture)),
				wgpu::CurrentSurfaceTexture::Timeout => 					return Err(wgpu::CurrentSurfaceTexture::Timeout),
				wgpu::CurrentSurfaceTexture::Occluded => 					return Err(wgpu::CurrentSurfaceTexture::Occluded),
				wgpu::CurrentSurfaceTexture::Outdated => 					return Err(wgpu::CurrentSurfaceTexture::Outdated),
				wgpu::CurrentSurfaceTexture::Lost => 						return Err(wgpu::CurrentSurfaceTexture::Lost),
				wgpu::CurrentSurfaceTexture::Validation => 					return Err(wgpu::CurrentSurfaceTexture::Validation),
			}
		};

		let _zone = span!("Render");

		let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Render Encoder") });
		self.voxel_renderer.render(&self.device, &mut encoder, &view, &self.camera_transform_bind_group, bvh, gpu_grid_tree_id_to_id_poses);
		self.debug_draw_renderer.render(&self.device, &mut encoder, &self.camera_bind_group, &view);
		self.crosshair_renderer.render(&self.config, &mut encoder, &self.queue, &output);
		// imgui
		{
			let io = self.imgui.io_mut();
			self.imgui_platform.prepare_frame(io, &*self.window).unwrap();
			let ui = self.imgui.frame();
			ui.window("Hello").size([300.0, 100.0], imgui::Condition::FirstUseEver).build(|| {
					ui.text("Hello world!");
					ui.text(format!("FPS: {:.2}", 1.0 / self.dt_avg));
				});

			self.imgui_platform.prepare_render(ui, &*self.window);
			let draw_data = self.imgui.render();
			let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
				label: Some("ImGui"),
				color_attachments: &[Some(wgpu::RenderPassColorAttachment {
					view: &view,
					resolve_target: None,
					depth_slice: None,
					ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
				})],
				depth_stencil_attachment: None,
				timestamp_writes: None,
				occlusion_query_set: None,
				multiview_mask: None,
			});
			self.imgui_renderer.render(draw_data, &self.queue, &self.device, &mut rpass).unwrap();
		}
		{
			let _zone = span!("Submit");
			self.queue.submit(std::iter::once(encoder.finish()));
		}
		{
			let _zone = span!("Present");
			output.present();
		}

		Ok(())
	}

	pub fn set_dt_avg(&mut self, dt_avg: f32) {
		self.dt_avg = dt_avg;
	}
}
