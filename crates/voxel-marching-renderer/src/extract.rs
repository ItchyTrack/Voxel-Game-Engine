use bevy::camera::{Camera, primitives::{Aabb, Frustum}};
use bevy::prelude::*;
use bevy::render::{Extract, sync_world::RenderEntity};
use tile_data::DynamicTileData;

use crate::{
	MarchingTileCapabilityRegistry,
	gpu_data::MarchingGpuBuffer,
	voxel_camera::VoxelMarchingCamera,
};

#[derive(Clone, Debug)]
pub struct ExtractedMarchingItem {
	pub vertex_count: u32,
	pub transform: Transform,
	pub vertex_buffer: MarchingGpuBuffer,
}

#[derive(Component)]
pub struct ExtractedMarchingScene {
	pub items: Vec<ExtractedMarchingItem>,
}

const PRIORITY_BUCKET_SCALE: f32 = 16.0;
fn priority_bucket(priority: f32) -> usize { priority.max(0.0).ln_1p().mul_add(PRIORITY_BUCKET_SCALE, 0.0) as usize }

pub fn extract_marching_scene(
	mut commands: Commands,
	existing: Query<Entity, With<ExtractedMarchingScene>>,
	capabilities: Extract<Res<MarchingTileCapabilityRegistry>>,
	cameras: Extract<Query<(RenderEntity, &VoxelMarchingCamera, &Camera, &GlobalTransform, &Frustum)>>,
	tiles: Extract<Query<(&DynamicTileData, &GlobalTransform)>>,
) {
	for entity in &existing { commands.entity(entity).remove::<ExtractedMarchingScene>(); }
	for (render_entity, marching_camera, camera, camera_transform, frustum) in &cameras {
		if !camera.is_active { continue; }
		let mut items = Vec::new();
		for entity in &marching_camera.tiles_to_render {
			let Ok((data, tile_global)) = tiles.get(*entity) else { continue };
			let Some(tile) = capabilities.read(data.data()) else { continue };
			let scale = (1u32 << tile.voxel_lod) as f32;
			let transform = tile_global.compute_transform() * Transform::from_scale(Vec3::splat(scale));
			let aabb = Aabb::from_min_max(tile.bounds_min, tile.bounds_max);
			if !frustum.intersects_obb(&aabb, &transform.compute_affine(), true, true) { continue; }
			items.push((
				priority_bucket(camera_transform.translation().distance(transform.translation)),
				entity.to_bits(),
				ExtractedMarchingItem { vertex_count: tile.vertex_count, transform, vertex_buffer: tile.vertices },
			));
		}
		items.sort_unstable_by_key(|(priority, entity, _)| (*priority, *entity));
		commands.entity(render_entity).insert(ExtractedMarchingScene {
			items: items.into_iter().map(|(_, _, item)| item).collect(),
		});
	}
}
