#![allow(dead_code)]

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::types::{PolicyDebugBoxKind, TileStatus};

pub(crate) fn draw_lod_policy_bounds_gizmos(
	mut gizmos: Gizmos,
	camera_voxel_loaders: Query<&CameraVoxelLoader, With<Camera3d>>,
	grids: Query<(GridId, &GlobalTransform), With<GridStreaming>>,
) {
	for camera_voxel_loader in &camera_voxel_loaders {
		for (grid, grid_transform) in &grids {
			if let Some((min_chunk, max_chunk)) = lod0_chunk_bounds(camera_voxel_loader, grid) {
				draw_lod_bound_box(&mut gizmos, grid_transform, min_chunk, max_chunk, lod_bound_color(0, camera_voxel_loader.settings.max_lod));
			}

			for key in camera_voxel_loader.desired_tiles.iter().filter(|key| key.grid == grid) {
				draw_lod_bound_box(&mut gizmos, grid_transform, key.min, key.min + key.size(), lod_bound_color(key.lod, camera_voxel_loader.settings.max_lod));
			}
		}
	}
}

fn lod0_chunk_bounds(camera_voxel_loader: &CameraVoxelLoader, grid: GridId) -> Option<(IVec3, IVec3)> {
	let mut min = IVec3::splat(i32::MAX);
	let mut max = IVec3::splat(i32::MIN);
	let mut any = false;
	for chunk in camera_voxel_loader.desired_chunks.iter().filter(|chunk| chunk.grid == grid) {
		any = true;
		min = min.min(chunk.chunk);
		max = max.max(chunk.chunk + IVec3::ONE);
	}
	any.then_some((min, max))
}

fn draw_centered_chunk_box(gizmos: &mut Gizmos, grid_transform: &GlobalTransform, center: IVec3, radius: i32, color: Color) {
	let min_chunk = center - IVec3::splat(radius);
	let max_chunk = center + IVec3::splat(radius + 1);
	draw_lod_bound_box(gizmos, grid_transform, min_chunk, max_chunk, color);
}

fn draw_lod_bound_box(gizmos: &mut Gizmos, grid_transform: &GlobalTransform, min_chunk: IVec3, max_chunk: IVec3, color: Color) {
	draw_transformed_box(
		gizmos,
		grid_transform,
		(min_chunk * CHUNK_SIZE).as_vec3(),
		(max_chunk * CHUNK_SIZE).as_vec3(),
		color,
	);
}

fn lod_bound_color(lod: u8, max_lod: u8) -> Color {
	let t = if max_lod == 0 { 0.0 } else { lod as f32 / max_lod as f32 };
	Color::srgba(0.05 + 0.25 * t, 0.20 + 0.45 * t, 1.0, 0.45 + 0.35 * (1.0 - t))
}

pub(crate) fn draw_policy_delta_gizmos(
	mut gizmos: Gizmos,
	cameras: Query<&CameraVoxelLoader>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	for camera_voxel_loader in &cameras {
		for debug_box in &camera_voxel_loader.policy_debug_boxes {
			let Ok(grid_transform) = grid_transforms.get(debug_box.grid) else { continue };
			draw_transformed_box(
				&mut gizmos,
				grid_transform,
				(debug_box.min * CHUNK_SIZE).as_vec3(),
				(debug_box.max * CHUNK_SIZE).as_vec3(),
				policy_delta_color(debug_box.kind, debug_box.entering),
			);
		}
	}
}

fn policy_delta_color(kind: PolicyDebugBoxKind, entering: bool) -> Color {
	let alpha = match kind {
		PolicyDebugBoxKind::NearChunks => 0.95,
		PolicyDebugBoxKind::LodOuter(_) => 0.75,
		PolicyDebugBoxKind::LodInner(_) => 0.55,
		PolicyDebugBoxKind::LodNearExclusion(_) => 0.35,
	};
	if entering {
		Color::srgba(0.1, 1.0, 0.15, alpha)
	} else {
		Color::srgba(1.0, 0.1, 0.05, alpha)
	}
}

fn draw_retiring_lod_gizmos(
	mut gizmos: Gizmos,
	cameras: Query<&CameraVoxelLoader>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	let color = Color::srgba(1.0, 0.15, 0.0, 0.9);
	for camera_voxel_loader in &cameras {
		for (key, record) in &camera_voxel_loader.tiles {
			if record.status != TileStatus::Retiring {
				continue;
			}
			let Ok(grid_transform) = grid_transforms.get(key.grid) else { continue };
			let min = (key.min * CHUNK_SIZE).as_vec3();
			let max = ((key.min + key.size()) * CHUNK_SIZE).as_vec3();
			draw_transformed_box(&mut gizmos, grid_transform, min, max, color);
		}
	}
}

fn draw_transformed_box(gizmos: &mut Gizmos, transform: &GlobalTransform, min: Vec3, max: Vec3, color: Color) {
	let corner = |x: f32, y: f32, z: f32| transform.transform_point(Vec3::new(x, y, z));
	let c000 = corner(min.x, min.y, min.z);
	let c100 = corner(max.x, min.y, min.z);
	let c010 = corner(min.x, max.y, min.z);
	let c001 = corner(min.x, min.y, max.z);
	let c110 = corner(max.x, max.y, min.z);
	let c101 = corner(max.x, min.y, max.z);
	let c011 = corner(min.x, max.y, max.z);
	let c111 = corner(max.x, max.y, max.z);

	for (a, b) in [
		(c000, c100),
		(c010, c110),
		(c001, c101),
		(c011, c111),
		(c000, c010),
		(c100, c110),
		(c001, c011),
		(c101, c111),
		(c000, c001),
		(c100, c101),
		(c010, c011),
		(c110, c111),
	] {
		gizmos.line(a, b, color);
	}
}

