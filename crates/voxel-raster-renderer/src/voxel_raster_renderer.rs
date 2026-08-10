use bevy::asset::Handle;
use bevy::render::render_resource::{
	CachedRenderPipelineId, FragmentState, PipelineCache, RenderPipelineDescriptor, VertexState,
};
use bevy::shader::Shader;

use crate::voxel_raster_renderer_resource::VoxelRasterRendererResource;

impl VoxelRasterRendererResource {
	pub fn pipeline(
		&mut self,
		pipeline_cache: &PipelineCache,
		color_format: wgpu::TextureFormat,
		shader: &Handle<Shader>,
	) -> CachedRenderPipelineId {
		if let Some(&pipeline) = self.pipelines.get(&color_format) {
			return pipeline;
		}

		let pipeline = pipeline_cache.queue_render_pipeline(RenderPipelineDescriptor {
			label: Some("raster_voxel_pipeline".into()),
			layout: vec![
				self.view_bind_group_layout.clone(),
				self.model_bind_group_layout.clone(),
				self.face_bind_group_layout.clone(),
			],
			vertex: VertexState {
				shader: shader.clone(),
				entry_point: Some("vs_main".into()),
				..Default::default()
			},
			fragment: Some(FragmentState {
				shader: shader.clone(),
				entry_point: Some("fs_main".into()),
				targets: vec![Some(wgpu::ColorTargetState {
					format: color_format,
					blend: Some(wgpu::BlendState::REPLACE),
					write_mask: wgpu::ColorWrites::ALL,
				})],
				..Default::default()
			}),
			primitive: wgpu::PrimitiveState {
				topology: wgpu::PrimitiveTopology::TriangleList,
				cull_mode: Some(wgpu::Face::Back),
				..Default::default()
			},
			depth_stencil: Some(wgpu::DepthStencilState {
				format: wgpu::TextureFormat::Depth32Float,
				depth_write_enabled: Some(true),
				depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
				stencil: wgpu::StencilState::default(),
				bias: wgpu::DepthBiasState::default(),
			}),
			multisample: wgpu::MultisampleState::default(),
			..Default::default()
		});
		self.pipelines.insert(color_format, pipeline);
		pipeline
	}
}
