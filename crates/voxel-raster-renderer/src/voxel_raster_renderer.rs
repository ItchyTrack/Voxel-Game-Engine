use bevy::render::renderer::WgpuWrapper;

use crate::camera::CameraUniform;
use crate::model::ModelUniform;

type GpuBindGroupLayout = WgpuWrapper<wgpu::BindGroupLayout>;
type GpuRenderPipeline = WgpuWrapper<wgpu::RenderPipeline>;

pub struct VoxelRasterRenderer {
	pub pipeline: GpuRenderPipeline,
	pub face_bind_group_layout: GpuBindGroupLayout,
}

impl VoxelRasterRenderer {
	pub fn new(
		device: &wgpu::Device,
		width: u32,
		height: u32,
		color_format: wgpu::TextureFormat,
		camera_bind_group_layout: &GpuBindGroupLayout,
		model_bind_group_layout: &GpuBindGroupLayout,
	) -> anyhow::Result<Self> {
		let face_bind_group_layout = WgpuWrapper::new(device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			entries: &[wgpu::BindGroupLayoutEntry {
				binding: 0,
				visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
				ty: wgpu::BindingType::Buffer {
					ty: wgpu::BufferBindingType::Storage { read_only: true },
					has_dynamic_offset: false,
					min_binding_size: None,
				},
				count: None,
			}],
			label: Some("raster_face_bind_group_layout"),
		}));

		let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("Raster Voxel Shader"),
			source: wgpu::ShaderSource::Wgsl(include_str!("shaders/raster_voxel.wgsl").into()),
		});
		let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
			label: Some("Raster Voxel Pipeline Layout"),
			bind_group_layouts: &[
				Some(camera_bind_group_layout),
				Some(model_bind_group_layout),
				Some(&face_bind_group_layout),
			],
			immediate_size: 0,
		});
		let pipeline = WgpuWrapper::new(device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
			label: Some("Raster Voxel Pipeline"),
			layout: Some(&pipeline_layout),
			vertex: wgpu::VertexState {
				module: &shader,
				entry_point: Some("vs_main"),
				buffers: &[],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			},
			fragment: Some(wgpu::FragmentState {
				module: &shader,
				entry_point: Some("fs_main"),
				targets: &[Some(wgpu::ColorTargetState {
					format: color_format,
					blend: Some(wgpu::BlendState::REPLACE),
					write_mask: wgpu::ColorWrites::ALL,
				})],
				compilation_options: wgpu::PipelineCompilationOptions::default(),
			}),
			primitive: wgpu::PrimitiveState {
				topology: wgpu::PrimitiveTopology::TriangleList,
				strip_index_format: None,
				front_face: wgpu::FrontFace::Ccw,
				cull_mode: Some(wgpu::Face::Back),
				polygon_mode: wgpu::PolygonMode::Fill,
				unclipped_depth: false,
				conservative: false,
			},
			depth_stencil: Some(wgpu::DepthStencilState {
				format: wgpu::TextureFormat::Depth32Float,
				depth_write_enabled: Some(true),
				depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState::default(),
			multiview_mask: None,
			cache: None,
		}));

		let _ = CameraUniform::from_camera;
		let _ = ModelUniform::from_mat4;
		let _ = (width, height);

		Ok(Self {
			pipeline,
			face_bind_group_layout,
		})
	}
}
